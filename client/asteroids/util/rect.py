class Rect:
    def __init__(self, *args):
        if len(args) == 2:
            (x, y), (w, h) = args
        elif len(args) == 4:
            x, y, w, h = args
        else:
            raise TypeError("Rect expects (x, y, w, h) or ((x, y), (w, h))")
        self.x = float(x)
        self.y = float(y)
        self.width = float(w)
        self.height = float(h)

    @property
    def left(self):
        return self.x

    @property
    def right(self):
        return self.x + self.width

    @property
    def top(self):
        return self.y

    @property
    def bottom(self):
        return self.y + self.height

    @property
    def centerx(self):
        return self.x + self.width * 0.5

    @property
    def centery(self):
        return self.y + self.height * 0.5

    def normalize(self):
        if self.width < 0:
            self.x += self.width
            self.width = -self.width
        if self.height < 0:
            self.y += self.height
            self.height = -self.height

    def colliderect(self, other):
        if other is None:
            return False
        return not (
            self.right < other.left
            or self.left > other.right
            or self.bottom < other.top
            or self.top > other.bottom
        )

    def collidepoint(self, point):
        px, py = point
        return (self.left <= px <= self.right) and (self.top <= py <= self.bottom)

    @classmethod
    def from_points(cls, points):
        if not points:
            return cls(0.0, 0.0, 0.0, 0.0)
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        minx = min(xs)
        maxx = max(xs)
        miny = min(ys)
        maxy = max(ys)
        return cls(minx, miny, maxx - minx, maxy - miny)
