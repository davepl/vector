#!/usr/bin/env python3
"""
tekview.py - Parse Tektronix 4010/4014 .plt files and stream to HP 1345A.

This parses the Tektronix vector graphics stream (GS/FS/RS/US and packed
coordinates) and translates move/draw operations into HP1345A "sendword"
commands using the same binary protocol as client/hptest.py.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import serial

# Serial baud rate - must match ESP32 Serial.begin() and platformio.ini monitor_speed
BAUD_RATE = 921600

PREAMBLE = bytes([0xA5, 0x5A, 0xC3, 0x3C])

CMD_WRITE_WORDS = 0x01
CMD_REPLACE_WORDS = 0x02
CMD_CLEAR = 0x03
CMD_COMMIT_ONCE = 0x04
CMD_COMMIT_LOOP = 0x05
CMD_STOP_LOOP = 0x06

HP_MAX = 2047


def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def clamp(v: float, lo: int = 0, hi: int = HP_MAX) -> int:
    iv = int(round(v))
    if iv < lo:
        return lo
    if iv > hi:
        return hi
    return iv


@dataclass
class Op:
    kind: str  # "move", "draw", "point"
    x: int
    y: int


class VectorBuilder:
    def __init__(self) -> None:
        self.words: List[int] = []
        self.cur_x: int = 0
        self.cur_y: int = 0
        self.pen_down: bool = False

    def make_x_word(self, x: int, pen_down: bool) -> int:
        pen_bit = 0x0800 if pen_down else 0x0000
        return pen_bit | (x & 0x07FF)

    def make_y_word(self, y: int, pen_down: bool) -> int:
        pen_bit = 0x0800 if pen_down else 0x0000
        return 0x1000 | pen_bit | (y & 0x07FF)

    def make_text_word(self, ch: int, set_size: bool, size: int, rot: int) -> int:
        s1 = (size >> 1) & 0x1
        s0 = size & 0x1
        r1 = (rot >> 1) & 0x1
        r0 = rot & 0x1
        return (
            0x4000
            | (s1 << 12)
            | (s0 << 11)
            | (r1 << 10)
            | (r0 << 9)
            | (0x0100 if set_size else 0x0000)
            | (ch & 0xFF)
        )

    def move_to(self, x: int, y: int) -> None:
        self.words.append(self.make_x_word(x, False))
        self.words.append(self.make_y_word(y, False))
        self.cur_x = x
        self.cur_y = y
        self.pen_down = False

    def line_to(self, x: int, y: int) -> None:
        if self.pen_down:
            self.words.append(self.make_x_word(x, True))
            self.words.append(self.make_y_word(y, True))
        else:
            self.words.append(self.make_x_word(self.cur_x, False))
            self.words.append(self.make_y_word(self.cur_y, False))
            self.words.append(self.make_x_word(x, True))
            self.words.append(self.make_y_word(y, True))
            self.pen_down = True
        self.cur_x = x
        self.cur_y = y

    def point_at(self, x: int, y: int) -> None:
        # Zero-length vector at (x,y).
        self.words.append(self.make_x_word(x, False))
        self.words.append(self.make_y_word(y, False))
        self.words.append(self.make_x_word(x, True))
        self.words.append(self.make_y_word(y, True))
        self.cur_x = x
        self.cur_y = y
        self.pen_down = True


class TekPlotParser:
    """Parse Tektronix 4010/4014 plot stream into move/draw/point ops."""

    def __init__(self, debug: bool = False) -> None:
        self.debug = debug
        self.mode = 0  # 0=alpha, 1-8=graphics, 30=ESC, 40=incremental, 50=special point intensity
        self.escape_return_mode = 0
        self.plot_point_mode = False
        self.special_plot_mode = False
        self.pen_down = True  # incremental mode pen state

        self.xh = 0
        self.xl = 0
        self.yh = 0
        self.yl = 0
        self.xy4014 = 0  # extra 4 bits (2 x + 2 y) for 4014 EGM
        self.extended = False

        self.last_x = 0
        self.last_y = 0

        self.ops: List[Op] = []
        self.max_x = 0
        self.max_y = 0

    def reset_drawing(self) -> None:
        self.ops.clear()
        self.max_x = 0
        self.max_y = 0
        self.last_x = 0
        self.last_y = 0
        self.xh = 0
        self.xl = 0
        self.yh = 0
        self.yl = 0
        self.xy4014 = 0
        self.extended = False

    def _emit(self, kind: str, x: int, y: int) -> None:
        self.ops.append(Op(kind, x, y))
        if x > self.max_x:
            self.max_x = x
        if y > self.max_y:
            self.max_y = y

    def _compute_coord(self) -> Tuple[int, int]:
        base_x = self.xh + self.xl
        base_y = self.yh + self.yl
        if self.extended:
            xb = self.xy4014 & 0x03
            yb = (self.xy4014 >> 2) & 0x03
            return ((base_x << 2) | xb, (base_y << 2) | yb)
        return base_x, base_y

    def _handle_escape(self, ch: int) -> None:
        # ESC FF clears screen.
        if ch == 0x0C:  # FF
            self.reset_drawing()
            self.mode = 0
            self.plot_point_mode = False
            self.special_plot_mode = False
            return
        # ESC FS enters special point plot (intensity byte after each point).
        if ch == 0x1C:  # FS
            self.plot_point_mode = True
            self.special_plot_mode = True
            self.mode = 5
            return
        # Unhandled escape sequences: return to prior mode.
        self.mode = self.escape_return_mode if self.escape_return_mode != 30 else 0

    def _handle_control(self, ch: int) -> None:
        if ch == 0x1B:  # ESC
            self.escape_return_mode = self.mode
            self.mode = 30
            return
        if ch == 0x1D:  # GS - graphics mode (vector)
            self.mode = 1
            self.plot_point_mode = False
            self.special_plot_mode = False
            return
        if ch == 0x1C:  # FS - point plot mode
            self.mode = 5
            self.plot_point_mode = True
            self.special_plot_mode = False
            return
        if ch == 0x1E:  # RS - incremental mode
            self.mode = 40
            self.pen_down = True
            return
        if ch == 0x1F:  # US - alpha mode
            self.mode = 0
            self.plot_point_mode = False
            self.special_plot_mode = False
            return
        if ch == 0x0D:  # CR
            if 1 <= self.mode <= 8:
                self.mode = 0
            return
        # Other control characters ignored.

    def _handle_incremental(self, ch: int) -> None:
        if ch < 0x20:
            self._handle_control(ch)
            return
        if ch == 0x20:  # space = pen up
            self.pen_down = False
            return
        if ch == 0x50:  # 'P' = pen down
            self.pen_down = True
            return
        if (ch & 0x70) == 0x40:
            dx = 0
            dy = 0
            if ch & 0x01:
                dx += 1
            if ch & 0x02:
                dx -= 1
            if ch & 0x04:
                dy += 1
            if ch & 0x08:
                dy -= 1
            nx = self.last_x + dx
            ny = self.last_y + dy
            if nx < 0:
                nx = 0
            if ny < 0:
                ny = 0
            if self.pen_down:
                self._emit("draw", nx, ny)
            else:
                self._emit("move", nx, ny)
            self.last_x, self.last_y = nx, ny

    def _handle_graphics_byte(self, ch: int) -> None:
        tag = (ch >> 5) & 0x03
        if 1 <= self.mode <= 8:
            if tag != 0:
                if self.mode == 1 and tag != 1:
                    self.mode = 2
                if self.mode == 3 and tag == 3:
                    # 4014 EGM extra data byte (between high Y and low Y).
                    self.mode = 2
                    self.xy4014 = self.yl
                    self.extended = True
                if self.mode == 2 and tag != 3:
                    self.mode = 3
                if self.mode == 3 and tag != 1:
                    self.mode = 4

                if self.mode == 5 and tag != 1:
                    self.mode = 6
                if self.mode == 7 and tag == 3:
                    self.mode = 6
                    self.xy4014 = self.yl
                    self.extended = True
                if self.mode == 6 and tag != 3:
                    self.mode = 7
                if self.mode == 7 and tag != 1:
                    self.mode = 8
            else:
                # Control byte in graphics; should be handled by _handle_control.
                return

        if self.mode == 1:
            self.plot_point_mode = False
            self.yh = 32 * (ch & 31)
            self.mode = 2
            return
        if self.mode == 2:
            self.yl = (ch & 31)
            self.mode = 3
            return
        if self.mode == 3:
            if tag == 1:
                self.xh = 32 * (ch & 31)
                self.mode = 4
            return
        if self.mode == 4:
            self.xl = (ch & 31)
            x, y = self._compute_coord()
            self._emit("move", x, y)
            self.last_x, self.last_y = x, y
            self.mode = 5
            return
        if self.mode == 5:
            if tag != 0:
                self.yh = 32 * (ch & 31)
                self.mode = 6
            return
        if self.mode == 6:
            self.yl = (ch & 31)
            self.mode = 7
            return
        if self.mode == 7:
            if tag == 1:
                self.xh = 32 * (ch & 31)
                self.mode = 8
            return
        if self.mode == 8:
            self.xl = (ch & 31)
            x, y = self._compute_coord()
            if self.plot_point_mode:
                self._emit("point", x, y)
            else:
                self._emit("draw", x, y)
            self.last_x, self.last_y = x, y
            if self.special_plot_mode:
                self.mode = 50
            else:
                self.mode = 5
            return

    def parse(self, data: bytes) -> List[Op]:
        for ch in data:
            if self.mode == 30:
                self._handle_escape(ch)
                continue
            if self.mode == 50:
                # Intensity byte in special point plot mode; ignore.
                self.mode = 5
                continue
            if self.mode == 40:
                self._handle_incremental(ch)
                continue

            if ch < 0x20:
                self._handle_control(ch)
                continue

            if self.mode == 0:
                # Alpha mode: ignore printable characters for now.
                continue

            self._handle_graphics_byte(ch)

        return self.ops


def pack_packet(cmd: int, words: Optional[List[int]] = None, with_crc: bool = True) -> bytes:
    if words is None:
        words = []
    flags = 0x01 if with_crc else 0x00
    length = len(words)
    payload = bytearray()
    for w in words:
        payload.append(w & 0xFF)
        payload.append((w >> 8) & 0xFF)
    header = bytearray(PREAMBLE)
    header.append(cmd & 0xFF)
    header.append(flags)
    header.append(length & 0xFF)
    header.append((length >> 8) & 0xFF)
    packet = header + payload
    if with_crc:
        crc_input = bytes([cmd & 0xFF, flags, length & 0xFF, (length >> 8) & 0xFF]) + payload
        crc = crc16_ccitt(crc_input)
        packet.append(crc & 0xFF)
        packet.append((crc >> 8) & 0xFF)
    return bytes(packet)


def send_words(ser: serial.Serial, words: List[int]) -> None:
    # Replace staging buffer with new content (chunked if needed).
    chunk_words = 20000
    if not words:
        ser.write(pack_packet(CMD_REPLACE_WORDS, [], with_crc=True))
        return
    offset = 0
    first = True
    while offset < len(words):
        chunk = words[offset:offset + chunk_words]
        cmd = CMD_REPLACE_WORDS if first else CMD_WRITE_WORDS
        ser.write(pack_packet(cmd, chunk, with_crc=True))
        first = False
        offset += len(chunk)


def commit_loop(ser: serial.Serial, hz: int) -> None:
    ser.write(pack_packet(CMD_COMMIT_LOOP, [hz & 0xFFFF], with_crc=True))


def transform_ops(
    ops: List[Op],
    tek_max: int,
    scale: float,
    x_scale: Optional[float],
    y_scale: Optional[float],
    x_offset: float,
    y_offset: float,
    invert_y: bool,
    center: bool,
) -> List[Op]:
    base_scale = (HP_MAX / tek_max) if tek_max > 0 else 1.0
    sx = base_scale * (x_scale if x_scale is not None else scale)
    sy = base_scale * (y_scale if y_scale is not None else scale)

    def apply_transform(x: int, y: int) -> Tuple[float, float]:
        yy = (tek_max - y) if invert_y else y
        return (x * sx, yy * sy)

    cx_off = 0.0
    cy_off = 0.0
    if center and ops:
        min_x = float("inf")
        max_x = float("-inf")
        min_y = float("inf")
        max_y = float("-inf")
        for op in ops:
            tx, ty = apply_transform(op.x, op.y)
            if tx < min_x:
                min_x = tx
            if tx > max_x:
                max_x = tx
            if ty < min_y:
                min_y = ty
            if ty > max_y:
                max_y = ty
        span_x = max_x - min_x
        span_y = max_y - min_y
        if span_x > 0:
            cx_off = (HP_MAX - span_x) * 0.5 - min_x
        if span_y > 0:
            cy_off = (HP_MAX - span_y) * 0.5 - min_y

    # No default vertical shift.

    out: List[Op] = []
    for op in ops:
        tx, ty = apply_transform(op.x, op.y)
        tx += cx_off + x_offset
        ty += cy_off + y_offset
        out.append(Op(op.kind, clamp(tx), clamp(ty)))
    return out


def build_words(ops: List[Op]) -> List[int]:
    b = VectorBuilder()
    for op in ops:
        if op.kind == "move":
            b.move_to(op.x, op.y)
        elif op.kind == "draw":
            b.line_to(op.x, op.y)
        elif op.kind == "point":
            b.point_at(op.x, op.y)
    return b.words


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stream Tektronix .plt to HP 1345A")
    parser.add_argument("plt", help="Tektronix .plt file to parse")
    parser.add_argument("--port", default="/dev/cu.usbserial-0001")
    parser.add_argument("--hz", type=int, default=60, help="refresh loop rate (0=fast)")
    parser.add_argument("--loop", action="store_true", help="keep redrawing until Ctrl-C")
    parser.add_argument("--no-send", action="store_true", help="parse only; do not send to serial")
    parser.add_argument("--clear", action="store_true", help="send CLEAR before streaming")
    parser.add_argument("--tek-max", type=int, choices=[1023, 4095], help="override Tek coordinate max")
    parser.add_argument("--scale", type=float, default=1.0, help="additional scale factor")
    parser.add_argument("--x-scale", type=float, help="override X scale multiplier")
    parser.add_argument("--y-scale", type=float, help="override Y scale multiplier")
    parser.add_argument("--x-offset", type=float, default=0.0, help="X offset (HP coords)")
    parser.add_argument("--y-offset", type=float, default=0.0, help="Y offset (HP coords)")
    parser.add_argument("--invert-y", action="store_true", help="invert Y axis")
    parser.add_argument("--center", action="store_true", help="center drawing in HP coords")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    try:
        with open(args.plt, "rb") as f:
            data = f.read()
    except OSError as exc:
        print(f"Failed to read {args.plt}: {exc}", file=sys.stderr)
        sys.exit(1)

    parser = TekPlotParser()
    ops = parser.parse(data)

    tek_max = args.tek_max
    if tek_max is None:
        tek_max = 4095 if parser.extended or max(parser.max_x, parser.max_y) > 1023 else 1023

    ops_hp = transform_ops(
        ops,
        tek_max=tek_max,
        scale=args.scale,
        x_scale=args.x_scale,
        y_scale=args.y_scale,
        x_offset=args.x_offset,
        y_offset=args.y_offset,
        invert_y=args.invert_y,
        center=args.center,
    )

    words = build_words(ops_hp)

    print(
        f"Parsed {len(data)} bytes -> {len(ops)} ops, "
        f"{len(words)} words, tek_max={tek_max}, extended={parser.extended}"
    )

    if args.no_send:
        return

    ser = serial.Serial(args.port, BAUD_RATE, timeout=0.05)
    try:
        if args.clear:
            ser.write(pack_packet(CMD_CLEAR, [], with_crc=True))
            time.sleep(0.05)
        send_words(ser, words)
        commit_loop(ser, args.hz)
        if args.loop:
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                ser.write(pack_packet(CMD_STOP_LOOP, [], with_crc=True))
    finally:
        ser.close()


if __name__ == "__main__":
    main()
