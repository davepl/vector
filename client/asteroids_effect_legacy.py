"""Asteroids demo effect for HP1345A vector display.

Designed to be used from client/hptest.py Test 1.
Keeps a hard segment budget so it stays renderable.

Effect goals:
- Player ship spins and fires repeatedly
- Rocks drift with varied silhouettes
- Flying saucer enters and fires randomly
- Shots are overdrawn for brightness
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Sequence, Tuple


Point = Tuple[float, float]


def _clamp(v: float, maxc: int) -> int:
    iv = int(round(v))
    if iv < 0:
        return 0
    if iv > maxc:
        return maxc
    return iv


def _rot(x: float, y: float, a: float) -> Point:
    ca, sa = math.cos(a), math.sin(a)
    return x * ca - y * sa, x * sa + y * ca


def _wrap(p: float, low: float, high: float) -> float:
    span = high - low
    while p < low:
        p += span
    while p > high:
        p -= span
    return p


@dataclass
class Rock:
    x: float
    y: float
    vx: float
    vy: float
    ang: float
    vang: float
    r: float
    shape: List[Point]  # normalized around origin


@dataclass
class Bullet:
    x: float
    y: float
    vx: float
    vy: float
    life: float
    bright_passes: int


@dataclass
class Saucer:
    active: bool
    x: float
    y: float
    vx: float
    vy: float
    t_until_shot: float
    t_until_spawn: float


@dataclass
class Ship:
    x: float
    y: float
    ang: float
    spin: float
    t_until_shot: float


@dataclass
class State:
    maxc: int
    aspect_x: float
    last_t: float
    ship: Ship
    saucer: Saucer
    rocks: List[Rock]
    bullets: List[Bullet]


def _make_rock_shape(rng: random.Random, verts: int) -> List[Point]:
    # Generate a jagged but closed polygon around the unit circle.
    pts: List[Point] = []
    a0 = rng.random() * 2.0 * math.pi
    for i in range(verts):
        a = a0 + (2.0 * math.pi * i) / verts
        rr = 1.0 * (0.70 + 0.45 * rng.random())
        pts.append((math.cos(a) * rr, math.sin(a) * rr))
    return pts


def _init_state(maxc: int, aspect_x: float, seed: int) -> State:
    rng = random.Random(seed)

    ship = Ship(
        x=maxc * 0.5,
        y=maxc * 0.18,
        ang=0.0,
        spin=1.6,  # rad/sec
        t_until_shot=0.10,
    )

    saucer = Saucer(
        active=False,
        x=-200.0,
        y=maxc * 0.75,
        vx=0.0,
        vy=0.0,
        t_until_shot=0.0,
        t_until_spawn=2.5,
    )

    rocks: List[Rock] = []
    rock_count = 6
    for i in range(rock_count):
        r = rng.uniform(maxc * 0.030, maxc * 0.080)
        x = rng.uniform(r, maxc - r)
        y = rng.uniform(maxc * 0.30, maxc - r)
        sp = rng.uniform(maxc * 0.015, maxc * 0.045)
        a = rng.random() * 2.0 * math.pi
        vx = math.cos(a) * sp
        vy = math.sin(a) * sp
        vang = rng.uniform(-1.0, 1.0)
        shape = _make_rock_shape(rng, verts=rng.randint(8, 12))
        rocks.append(Rock(x=x, y=y, vx=vx, vy=vy, ang=rng.random() * 2.0 * math.pi, vang=vang, r=r, shape=shape))

    return State(
        maxc=maxc,
        aspect_x=aspect_x,
        last_t=0.0,
        ship=ship,
        saucer=saucer,
        rocks=rocks,
        bullets=[],
    )


def draw(
    b,
    t: float,
    *,
    maxc: int,
    max_vectors: int = 400,
    aspect_x: float = 0.75,
) -> None:
    """Render/update an Asteroids-like scene into VectorBuilder `b`.

    `max_vectors` caps line segments (line_to ops) to keep it renderable.
    """

    # Module-level persistent state
    if not hasattr(draw, "_state"):
        # Stable-ish seed: pick something deterministic but not constant across runs.
        seed = int((t * 1000.0)) & 0xFFFFFFFF
        draw._state = _init_state(maxc=maxc, aspect_x=aspect_x, seed=seed)
        draw._state.last_t = t

    st: State = draw._state
    st.maxc = maxc
    st.aspect_x = aspect_x

    dt = t - st.last_t
    st.last_t = t
    if dt < 0.0:
        dt = 0.0
    if dt > 0.10:
        dt = 0.10

    # Segment budget wrappers
    seg_budget = int(max_vectors)

    def mv(x: float, y: float) -> None:
        b.move_to(_clamp(x, maxc), _clamp(y, maxc))

    def ln(x: float, y: float) -> bool:
        nonlocal seg_budget
        if seg_budget <= 0:
            return False
        b.line_to(_clamp(x, maxc), _clamp(y, maxc))
        seg_budget -= 1
        return True

    def bright_segment(x0: float, y0: float, x1: float, y1: float, passes: int) -> None:
        nonlocal seg_budget
        for _ in range(max(1, int(passes))):
            if seg_budget <= 0:
                return
            mv(x0, y0)
            ln(x1, y1)

    def draw_poly(origin_x: float, origin_y: float, pts: Sequence[Point], ang: float, scale: float) -> None:
        if seg_budget <= 0 or not pts:
            return
        p0x, p0y = pts[0]
        rx, ry = _rot(p0x * scale * aspect_x, p0y * scale, ang)
        mv(origin_x + rx, origin_y + ry)
        for px, py in pts[1:]:
            rx, ry = _rot(px * scale * aspect_x, py * scale, ang)
            if not ln(origin_x + rx, origin_y + ry):
                return
        # close
        rx, ry = _rot(p0x * scale * aspect_x, p0y * scale, ang)
        ln(origin_x + rx, origin_y + ry)

    # ----------------------------
    # Update world
    # ----------------------------
    # Ship: constant spin
    st.ship.ang = (st.ship.ang + st.ship.spin * dt) % (2.0 * math.pi)

    # Fire ship bullets periodically
    st.ship.t_until_shot -= dt
    if st.ship.t_until_shot <= 0.0:
        st.ship.t_until_shot = 0.28  # cadence
        speed = maxc * 0.20
        vx = math.cos(st.ship.ang) * speed
        vy = math.sin(st.ship.ang) * speed
        st.bullets.append(
            Bullet(
                x=st.ship.x + math.cos(st.ship.ang) * (maxc * 0.025),
                y=st.ship.y + math.sin(st.ship.ang) * (maxc * 0.025),
                vx=vx,
                vy=vy,
                life=1.1,
                bright_passes=4,
            )
        )

    # Saucer spawn/update
    st.saucer.t_until_spawn -= dt
    if not st.saucer.active and st.saucer.t_until_spawn <= 0.0:
        st.saucer.active = True
        from_left = (random.random() < 0.5)
        st.saucer.x = -maxc * 0.10 if from_left else maxc * 1.10
        st.saucer.y = maxc * (0.55 + 0.25 * random.random())
        st.saucer.vx = (maxc * 0.10) * (1.0 if from_left else -1.0)
        st.saucer.vy = maxc * (0.02 * (random.random() - 0.5))
        st.saucer.t_until_shot = 0.35 + 0.30 * random.random()

    if st.saucer.active:
        st.saucer.x += st.saucer.vx * dt
        st.saucer.y += st.saucer.vy * dt
        st.saucer.y = maxc * 0.15 + (maxc * 0.70) * (0.5 + 0.5 * math.sin(t * 0.35 + 1.7))

        # Shoot randomly (slightly biased toward ship)
        st.saucer.t_until_shot -= dt
        if st.saucer.t_until_shot <= 0.0:
            st.saucer.t_until_shot = 0.55 + 0.55 * random.random()
            dx = st.ship.x - st.saucer.x
            dy = st.ship.y - st.saucer.y
            base = math.atan2(dy, dx)
            ang = base + (random.random() - 0.5) * 1.6
            speed = maxc * 0.16
            st.bullets.append(
                Bullet(
                    x=st.saucer.x,
                    y=st.saucer.y,
                    vx=math.cos(ang) * speed,
                    vy=math.sin(ang) * speed,
                    life=1.3,
                    bright_passes=3,
                )
            )

        # Despawn when off-screen
        if st.saucer.x < -maxc * 0.20 or st.saucer.x > maxc * 1.20:
            st.saucer.active = False
            st.saucer.t_until_spawn = 4.0 + 3.0 * random.random()

    # Rocks drift/rotate
    for r in st.rocks:
        r.x += r.vx * dt
        r.y += r.vy * dt
        r.ang = (r.ang + r.vang * dt) % (2.0 * math.pi)
        margin = r.r * 1.4
        if r.x < -margin:
            r.x = maxc + margin
        elif r.x > maxc + margin:
            r.x = -margin
        if r.y < -margin:
            r.y = maxc + margin
        elif r.y > maxc + margin:
            r.y = -margin

    # Bullets update
    new_bullets: List[Bullet] = []
    for bu in st.bullets:
        bu.life -= dt
        if bu.life <= 0.0:
            continue
        bu.x += bu.vx * dt
        bu.y += bu.vy * dt
        bu.x = _wrap(bu.x, -maxc * 0.05, maxc * 1.05)
        bu.y = _wrap(bu.y, -maxc * 0.05, maxc * 1.05)
        new_bullets.append(bu)
    st.bullets = new_bullets

    # ----------------------------
    # Render
    # ----------------------------
    # Rocks
    for r in st.rocks:
        if seg_budget <= 0:
            break
        draw_poly(r.x, r.y, r.shape, r.ang, r.r)

    # Ship
    ship_scale = maxc * 0.030
    ship_pts = [(0.0, 1.25), (-0.85, -0.9), (-0.25, -0.55), (0.25, -0.55), (0.85, -0.9)]
    draw_poly(st.ship.x, st.ship.y, ship_pts, st.ship.ang, ship_scale)
    # Tail notch
    if seg_budget > 0:
        a = st.ship.ang
        x0, y0 = _rot(-0.25 * ship_scale * aspect_x, -0.55 * ship_scale, a)
        x1, y1 = _rot(0.25 * ship_scale * aspect_x, -0.55 * ship_scale, a)
        mv(st.ship.x + x0, st.ship.y + y0)
        ln(st.ship.x + x1, st.ship.y + y1)

    # Saucer
    if st.saucer.active and seg_budget > 0:
        sx, sy = st.saucer.x, st.saucer.y
        w = maxc * 0.060
        h = maxc * 0.018
        # classic-ish saucer: two arcs approximated with polylines
        top = [(-1.0, 0.0), (-0.55, 0.35), (0.0, 0.45), (0.55, 0.35), (1.0, 0.0)]
        bot = [(-1.1, 0.0), (-0.55, -0.25), (0.0, -0.35), (0.55, -0.25), (1.1, 0.0)]
        mv(sx + top[0][0] * w * aspect_x, sy + top[0][1] * w)
        for px, py in top[1:]:
            ln(sx + px * w * aspect_x, sy + py * w)
        mv(sx + bot[0][0] * w * aspect_x, sy + bot[0][1] * w)
        for px, py in bot[1:]:
            ln(sx + px * w * aspect_x, sy + py * w)
        # dome
        mv(sx + (-0.35) * w * aspect_x, sy)
        ln(sx + 0.0 * w * aspect_x, sy + h)
        ln(sx + (0.35) * w * aspect_x, sy)

    # Bullets (bright overdraw)
    for bu in st.bullets:
        if seg_budget <= 0:
            break
        # render as small segment along velocity direction
        vlen = math.hypot(bu.vx, bu.vy)
        if vlen < 1e-6:
            continue
        ux, uy = bu.vx / vlen, bu.vy / vlen
        seg = maxc * 0.010
        x0 = bu.x - ux * seg * 0.5
        y0 = bu.y - uy * seg * 0.5
        x1 = bu.x + ux * seg * 0.5
        y1 = bu.y + uy * seg * 0.5
        bright_segment(x0, y0, x1, y1, passes=bu.bright_passes)
