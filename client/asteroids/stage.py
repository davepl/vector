#
#    Headless stage for vector rendering.
#

from .util.rect import Rect


def _rect_from_points(points):
    if not points:
        return Rect(0.0, 0.0, 0.0, 0.0)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    minx = min(xs)
    maxx = max(xs)
    miny = min(ys)
    maxy = max(ys)
    return Rect(minx, miny, maxx - minx, maxy - miny)


class Stage:
    def __init__(self, caption, dimensions=None):
        if dimensions is None:
            dimensions = (2048, 2048)
        self.spriteList = []
        self.width = float(dimensions[0])
        self.height = float(dimensions[1])
        self.showBoundingBoxes = False
        self.caption = caption

    def addSprite(self, sprite):
        self.spriteList.append(sprite)
        sprite.boundingRect = _rect_from_points(sprite.draw())

    def removeSprite(self, sprite):
        if sprite in self.spriteList:
            self.spriteList.remove(sprite)

    def drawSprites(self, draw_cb=None):
        for sprite in list(self.spriteList):
            points = sprite.draw()
            sprite.boundingRect = _rect_from_points(points)
            if draw_cb is not None and points:
                draw_cb(sprite, points)

    def moveSprites(self, dt=1.0):
        step = (dt * 60.0) if dt is not None else 1.0
        for sprite in list(self.spriteList):
            try:
                sprite.move(step)
            except TypeError:
                sprite.move()

            if sprite.position.x < 0:
                sprite.position.x = self.width
            if sprite.position.x > self.width:
                sprite.position.x = 0.0

            if sprite.position.y < 0:
                sprite.position.y = self.height
            if sprite.position.y > self.height:
                sprite.position.y = 0.0
