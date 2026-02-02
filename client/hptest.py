#!/usr/bin/env python3
"""
HP1345A test pattern host app.

Generates vector word lists for test patterns 0-9, then sends them to the
ESP32 streamer over serial using the binary protocol defined in src/main.cpp.
"""

import argparse
import math
import random
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from typing import Callable, List, Optional

import serial

PREAMBLE = bytes([0xA5, 0x5A, 0xC3, 0x3C])

CMD_WRITE_WORDS = 0x01
CMD_REPLACE_WORDS = 0x02
CMD_CLEAR = 0x03
CMD_COMMIT_ONCE = 0x04
CMD_COMMIT_LOOP = 0x05
CMD_STOP_LOOP = 0x06

MAXC = 2047


def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def clamp(v: float) -> int:
    iv = int(round(v))
    if iv < 0:
        return 0
    if iv > MAXC:
        return MAXC
    return iv


def coord_from_norm(n: float, center: float, radius: float) -> int:
    return clamp(center + n * radius)


@dataclass
class VectorBuilder:
    words: List[int]
    cur_x: int = 0
    cur_y: int = 0
    pen_down: bool = False

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
        # If pen is already down, we can just send X/Y with pen down.
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

    def text_at(self, x: int, y: int, text: str, size: int = 0, rot: int = 0) -> None:
        if not text:
            return
        self.move_to(x, y)
        first = True
        for ch in text:
            self.words.append(self.make_text_word(ord(ch), first, size, rot))
            first = False


# ------------------------------------------------------------
# Test patterns 0-9
# ------------------------------------------------------------

def test_6(b: VectorBuilder, t: float) -> None:
    m = 80
    minv = m
    maxv = MAXC - m
    b.move_to(minv, minv)
    b.line_to(maxv, minv)
    b.line_to(maxv, maxv)
    b.line_to(minv, maxv)
    b.line_to(minv, minv)
    b.move_to(minv, minv)
    b.line_to(maxv, maxv)
    b.move_to(minv, maxv)
    b.line_to(maxv, minv)


def test_1(b: VectorBuilder, t: float) -> None:
    steps = 6
    margin = 60
    span = MAXC - (margin * 2)
    step = span // (steps * 2)
    for i in range(steps):
        minv = margin + i * step
        maxv = MAXC - margin - i * step
        b.move_to(minv, minv)
        b.line_to(maxv, minv)
        b.line_to(maxv, maxv)
        b.line_to(minv, maxv)
        b.line_to(minv, minv)


def test_2(b: VectorBuilder, t: float) -> None:
    cx = MAXC // 2
    cy = MAXC // 2
    r = (MAXC // 2) - 40
    rays = 36
    for i in range(rays):
        a = (2.0 * math.pi * i) / rays
        x = coord_from_norm(math.cos(a), cx, r)
        y = coord_from_norm(math.sin(a), cy, r)
        b.move_to(cx, cy)
        b.line_to(x, y)


def test_3(b: VectorBuilder, t: float) -> None:
    left = 60
    right = MAXC - 60
    top = 60
    bottom = MAXC - 60
    step = 35
    b.move_to(left, top)
    while left + step < right and top + step < bottom:
        b.line_to(right, top)
        b.line_to(right, bottom)
        b.line_to(left, bottom)
        left += step
        top += step
        right -= step
        bottom -= step
        b.line_to(left, top)


def test_4(b: VectorBuilder, t: float) -> None:
    # Grid of static 3D cubes.
    cx = MAXC * 0.5
    cy = MAXC * 0.5
    cols, rows = 4, 3
    grid_w = MAXC * 0.7
    grid_h = MAXC * 0.6
    start_x = cx - grid_w * 0.5
    start_y = cy - grid_h * 0.5
    step_x = grid_w / (cols - 1)
    step_y = grid_h / (rows - 1)

    verts = [
        (-1, -1, -1),
        (1, -1, -1),
        (1, 1, -1),
        (-1, 1, -1),
        (-1, -1, 1),
        (1, -1, 1),
        (1, 1, 1),
        (-1, 1, 1),
    ]
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    ax, ay, az = 0.6 + t * 0.35, 0.4 + t * 0.25, 0.3 + t * 0.2
    cax, sax = math.cos(ax), math.sin(ax)
    cay, say = math.cos(ay), math.sin(ay)
    caz, saz = math.cos(az), math.sin(az)

    def rotate(x: float, y: float, z: float):
        # rotate around X
        y, z = y * cax - z * sax, y * sax + z * cax
        # rotate around Y
        x, z = x * cay + z * say, -x * say + z * cay
        # rotate around Z
        x, y = x * caz - y * saz, x * saz + y * caz
        return x, y, z

    cube_size = MAXC * 0.06
    focal = MAXC * 1.4
    zoff = MAXC * 2.2

    for r in range(rows):
        for c in range(cols):
            ox = start_x + c * step_x
            oy = start_y + r * step_y
            proj = []
            for vx, vy, vz in verts:
                x, y, z = rotate(vx, vy, vz)
                x *= cube_size
                y *= cube_size
                z *= cube_size
                z += zoff
                factor = focal / z
                px = ox + x * factor
                py = oy + y * factor
                proj.append((px, py))
            for a, bidx in edges:
                x0, y0 = proj[a]
                x1, y1 = proj[bidx]
                b.move_to(clamp(x0), clamp(y0))
                b.line_to(clamp(x1), clamp(y1))


def test_5(b: VectorBuilder, t: float) -> None:
    # Ripple/sombrero mesh (static).
    half = 8.0
    step = 2.0
    tilt = 0.65
    cosx, sinx = math.cos(tilt), math.sin(tilt)
    scale = 85.0

    def height(x: float, y: float) -> float:
        r = math.hypot(x, y)
        return math.cos(r * 0.65 - t * 2.5) * math.exp(-r * 0.12) * 5.0

    xs = [x for x in frange(-half, half, step)]
    ys = [y for y in frange(-half, half, step)]

    # Lines along X
    for y in ys:
        first = True
        for x in xs:
            z = height(x, y)
            yt = y * cosx - z * sinx
            px = MAXC * 0.5 + x * scale
            py = MAXC * 0.5 + yt * scale
            if first:
                b.move_to(clamp(px), clamp(py))
                first = False
            else:
                b.line_to(clamp(px), clamp(py))

    # Lines along Y
    for x in xs:
        first = True
        for y in ys:
            z = height(x, y)
            yt = y * cosx - z * sinx
            px = MAXC * 0.5 + x * scale
            py = MAXC * 0.5 + yt * scale
            if first:
                b.move_to(clamp(px), clamp(py))
                first = False
            else:
                b.line_to(clamp(px), clamp(py))


def test_0(b: VectorBuilder, t: float) -> None:
    # Static starfield streaks.
    random.seed(1)
    cx = MAXC * 0.5
    cy = MAXC * 0.5
    focal = MAXC * 1.2
    z_near, z_far = 0.2, 2.0
    count = 200
    for _ in range(count):
        angle = random.random() * 2 * math.pi
        radius = random.random() * 0.25
        x = math.cos(angle) * radius
        y = math.sin(angle) * radius
        # Simple animation by advancing z with time.
        z = z_near + ((random.random() + t * 0.15) % 1.0) * (z_far - z_near)
        px = cx + (x * focal) / z
        py = cy + (y * focal) / z
        trail = 4 + (1.0 - (z - z_near) / (z_far - z_near)) * 24
        dx = cx - px
        dy = cy - py
        dist = math.hypot(dx, dy) or 1.0
        dx /= dist
        dy /= dist
        tx = px + dx * trail
        ty = py + dy * trail
        b.move_to(clamp(px), clamp(py))
        b.line_to(clamp(tx), clamp(ty))


def test_7(b: VectorBuilder, t: float) -> None:
    # Spirograph (hypotrochoid) static.
    cx = MAXC // 2
    cy = MAXC // 2
    R, r, d = 150.0, 105.0, 75.0
    segments = 240
    gcd = 15.0
    turns = r / gcd
    t_max = 2.0 * math.pi * turns
    span = (R - r) + d
    scale = (MAXC * 0.42) / span

    spin = t * 0.6
    cos_s, sin_s = math.cos(spin), math.sin(spin)
    first = True
    for i in range(segments + 1):
        t = (t_max * i) / segments
        k = (R - r) / r
        x = (R - r) * math.cos(t) + d * math.cos(k * t)
        y = (R - r) * math.sin(t) - d * math.sin(k * t)
        xr = x * cos_s - y * sin_s
        yr = x * sin_s + y * cos_s
        sx = clamp(cx + xr * scale)
        sy = clamp(cy + yr * scale)
        if first:
            b.move_to(sx, sy)
            first = False
        else:
            b.line_to(sx, sy)


def test_8(b: VectorBuilder, t: float) -> None:
    # Checkerboard outlines.
    margin = 120
    rows, cols = 6, 6
    cell_w = (MAXC - margin * 2) // cols
    cell_h = (MAXC - margin * 2) // rows
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                x0 = margin + c * cell_w
                y0 = margin + r * cell_h
                x1 = x0 + cell_w
                y1 = y0 + cell_h
                b.move_to(x0, y0)
                b.line_to(x1, y0)
                b.line_to(x1, y1)
                b.line_to(x0, y1)
                b.line_to(x0, y0)


def test_9(b: VectorBuilder, t: float) -> None:
    # Matrix-style static characters.
    random.seed(2)
    chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ#$%&@*+-"
    columns = 18
    margin = 60
    col_spacing = (MAXC - margin * 2) / (columns - 1)
    y_top = MAXC - 120
    drift = (t * 80.0) % 180.0
    for i in range(columns):
        x = margin + i * col_spacing
        y = y_top - (i % 5) * 90 - drift
        ch = random.choice(chars)
        b.text_at(clamp(x), clamp(y), ch, size=1, rot=0)


def frange(start: float, stop: float, step: float):
    x = start
    while x <= stop + 1e-6:
        yield x
        x += step


TESTS: List[Callable[[VectorBuilder, float], None]] = [
    test_0,
    test_1,
    test_2,
    test_3,
    test_4,
    test_5,
    test_6,
    test_7,
    test_8,
    test_9,
]


# ------------------------------------------------------------
# Serial protocol helpers
# ------------------------------------------------------------

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


def commit_once(ser: serial.Serial) -> None:
    ser.write(pack_packet(CMD_COMMIT_ONCE, [], with_crc=True))


def stop_loop(ser: serial.Serial) -> None:
    ser.write(pack_packet(CMD_STOP_LOOP, [], with_crc=True))


# ------------------------------------------------------------
# Console / UI
# ------------------------------------------------------------

def build_words(idx: int, t: float) -> List[int]:
    b = VectorBuilder(words=[])
    TESTS[idx](b, t)
    return b.words


def write_console(message: str) -> None:
    # Normalize all local console output to CRLF for VSCode's serial console.
    sys.stdout.write(message.replace("\n", "\r\n") + "\r\n")
    sys.stdout.flush()


def print_help() -> None:
    write_console(
        "\nKeys:\n"
        "  0-9  send test pattern\n"
        "  n    next pattern\n"
        "  a    toggle animation\n"
        "  ?    help\n"
        "  (other keys are passed through to the ESP32)"
    )


def print_status(message: str) -> None:
    write_console(message)


def run_ui(port: str, hz: int) -> None:
    ser = serial.Serial(port, 921600, timeout=0.05)
    console_mode = threading.Event()
    running = True

    def reader() -> None:
        buf = ""
        while running:
            data = ser.read(256)
            if not data:
                time.sleep(0.01)
                continue
            text = data.decode("latin1", errors="replace")
            sys.stdout.write(text)
            sys.stdout.flush()
            buf_local = buf + text
            if "== HP1345A console ==" in buf_local:
                console_mode.set()
            if "Returning to binary mode" in buf_local or "streamer ready" in buf_local:
                console_mode.clear()
            buf_local = buf_local[-2000:]
            buf = buf_local

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    print_help()
    current = 0
    words = build_words(current, 0.0)
    send_words(ser, words)
    commit_loop(ser, hz)
    print_status(f"Sent test {current} ({len(words)} words), loop={hz} Hz")

    # Animation: 30 FPS when enabled, press 'a' to toggle.
    anim_fps = 30
    frame_interval = None  # Start with animation disabled
    next_frame = time.time()

    if not sys.stdin.isatty():
        write_console("stdin is not a TTY; running headless. Use Ctrl+C to exit.")
        try:
            while True:
                now = time.time()
                if frame_interval is not None and now >= next_frame and not console_mode.is_set():
                    t = now
                    words = build_words(current, t)
                    send_words(ser, words)
                    commit_once(ser)
                    if frame_interval > 0:
                        next_frame = now + frame_interval
                time.sleep(0.002)
        finally:
            ser.close()
        return

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)
    try:
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            now = time.time()
            if frame_interval is not None and now >= next_frame and not console_mode.is_set():
                t = now
                words = build_words(current, t)
                send_words(ser, words)
                commit_once(ser)
                if frame_interval > 0:
                    next_frame = now + frame_interval
            if not rlist:
                continue
            ch = sys.stdin.read(1)
            if ch == "\x03":
                break
            if ch == "?":
                print_help()
                continue
            if ch in "0123456789":
                current = int(ch)
                if console_mode.is_set():
                    ser.write(b"r\n")
                    console_mode.clear()
                    time.sleep(0.1)
                words = build_words(current, time.time())
                send_words(ser, words)
                commit_loop(ser, hz)
                print_status(
                    f"Sent test {current} ({len(words)} words), loop={hz} Hz"
                )
                continue
            if ch in "nN":
                current = (current + 1) % 10
                if console_mode.is_set():
                    ser.write(b"r\n")
                    console_mode.clear()
                    time.sleep(0.1)
                words = build_words(current, time.time())
                send_words(ser, words)
                commit_loop(ser, hz)
                print_status(
                    f"Sent test {current} ({len(words)} words), loop={hz} Hz"
                )
                continue
            if ch in "aA":
                if frame_interval is None:
                    frame_interval = 1.0 / anim_fps
                    next_frame = time.time()
                    print_status(f"Animation ON ({anim_fps} FPS)")
                else:
                    frame_interval = None
                    print_status("Animation OFF")
                continue

            # Pass-through to ESP32 (for +++ and console commands).
            ser.write(ch.encode("latin1", errors="ignore"))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description="HP1345A test pattern sender")
    parser.add_argument("--port", default="/dev/cu.usbserial-0001")
    parser.add_argument("--hz", type=int, default=60, help="loop rate (0=fast)")
    args = parser.parse_args()
    run_ui(args.port, args.hz)


if __name__ == "__main__":
    main()
