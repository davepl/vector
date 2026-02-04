from .asteroids import Asteroids

_game = None
_last_t = None


def draw(b, t, *, maxc, max_vectors=800, aspect_x=0.75):
    global _game, _last_t
    if _game is None or _game.maxc != float(maxc) or _game.aspect_x != float(aspect_x):
        seed = int(t * 1000.0) & 0xFFFFFFFF
        _game = Asteroids(maxc=maxc, aspect_x=aspect_x, seed=seed)
        _last_t = t
    if _last_t is None:
        dt = 0.0
    else:
        dt = t - _last_t
    _last_t = t
    _game.step(dt, b, max_vectors)
