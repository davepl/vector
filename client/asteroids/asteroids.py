#
# Headless Asteroids game logic adapted for vector rendering.
#

import math
import random

from .badies import Debris, Rock, Saucer
from .ship import Ship
from .soundManager import playSound, playSoundContinuous, stopSound
from .stage import Stage
from .util.vector2d import Vector2d


class VectorRenderer:
    def __init__(self, builder, maxc, aspect_x, max_vectors):
        self.builder = builder
        self.maxc = float(maxc)
        self.aspect_x = float(aspect_x)
        self.max_vectors = int(max_vectors) if max_vectors is not None else 1_000_000
        self.segments_used = 0
        self.cx = self.maxc * 0.5

    def _clamp(self, v):
        if v < 0:
            return 0
        if v > self.maxc:
            return int(self.maxc)
        return int(round(v))

    def _map(self, x, y):
        sx = self.cx + (x - self.cx) * self.aspect_x
        return self._clamp(sx), self._clamp(y)

    def draw_poly(self, sprite, points):
        if self.segments_used >= self.max_vectors:
            return
        if not points or len(points) < 2:
            return
        color = getattr(sprite, "color", (255, 255, 255))
        if isinstance(color, tuple) and len(color) >= 3:
            if color[0] <= 0 and color[1] <= 0 and color[2] <= 0:
                return
        passes = max(1, int(getattr(sprite, "bright_passes", 1)))

        def draw_once():
            x0, y0 = self._map(points[0][0], points[0][1])
            self.builder.move_to(x0, y0)
            for p in points[1:]:
                if self.segments_used >= self.max_vectors:
                    return False
                x1, y1 = self._map(p[0], p[1])
                self.builder.line_to(x1, y1)
                self.segments_used += 1
            if self.segments_used >= self.max_vectors:
                return False
            x1, y1 = self._map(points[0][0], points[0][1])
            self.builder.line_to(x1, y1)
            self.segments_used += 1
            return True

        for _ in range(passes):
            if self.segments_used >= self.max_vectors:
                return
            if not draw_once():
                return


class Asteroids:
    explodingTtl = 180

    def __init__(self, maxc=2048, aspect_x=0.75, seed=None):
        self.maxc = float(maxc)
        self.aspect_x = float(aspect_x)
        self._rng = random.Random(seed)
        self.stage = Stage("Asteroids", (self.maxc, self.maxc))
        self.attract_mode = True
        self._time = 0.0
        self.reset_attract()

    def reset_attract(self):
        self.gameState = "playing"
        self.score = 0
        self.lives = 1
        self.livesList = []
        self.rockList = []
        self.numRocks = 3
        self.nextLife = 10000
        self.saucer = None
        self._saucer_spawn_timer = self._rng.uniform(4.0, 7.0)
        self.explodingCount = 0.0
        self.ship = None
        self.createNewShip()
        self.createRocks(self.numRocks)
        self._reset_ai()

    def _reset_ai(self):
        self._ai_turn_timer = 0.0
        self._ai_thrust_timer = 0.0
        self._ai_fire_timer = 0.0
        self._ai_turn_dir = 1
        self._ai_thrusting = False

    def createNewShip(self):
        if self.ship:
            for debris in list(self.ship.shipDebrisList):
                self.stage.removeSprite(debris)
            self.ship.shipDebrisList = []
            self.stage.removeSprite(self.ship)
            self.stage.removeSprite(self.ship.thrustJet)
        self.ship = Ship(self.stage)
        self.stage.addSprite(self.ship.thrustJet)
        self.stage.addSprite(self.ship)

    def createRocks(self, numRocks):
        margin = self.maxc * 0.1
        for _ in range(0, numRocks):
            position = Vector2d(
                self._rng.uniform(margin, self.maxc - margin),
                self._rng.uniform(margin, self.maxc - margin),
            )
            newRock = Rock(self.stage, position, Rock.largeRockType)
            self.stage.addSprite(newRock)
            self.rockList.append(newRock)

    def levelUp(self):
        self.numRocks += 1
        self.createRocks(self.numRocks)

    def attractControl(self, dt):
        if not self.ship:
            return
        step = dt * 60.0
        self._ai_turn_timer -= dt
        if self._ai_turn_timer <= 0.0:
            self._ai_turn_timer = self._rng.uniform(0.25, 1.0)
            self._ai_turn_dir = -1 if self._rng.random() < 0.5 else 1
        if self._ai_turn_dir < 0:
            self.ship.rotateLeft(step)
        else:
            self.ship.rotateRight(step)

        self._ai_thrust_timer -= dt
        if self._ai_thrust_timer <= 0.0:
            self._ai_thrust_timer = self._rng.uniform(0.35, 1.2)
            self._ai_thrusting = not self._ai_thrusting
        if self._ai_thrusting:
            self.ship.increaseThrust(step)
            self.ship.thrustJet.accelerating = True
        else:
            self.ship.thrustJet.accelerating = False

        self._ai_fire_timer -= dt
        if self._ai_fire_timer <= 0.0:
            self._ai_fire_timer = self._rng.uniform(0.2, 0.6)
            self.ship.fireBullet()

    def doSaucerLogic(self, dt):
        if self.saucer is not None and self.saucer.laps >= 2:
            self.killSaucer()

        if self.saucer is None:
            self._saucer_spawn_timer -= dt
            if self._saucer_spawn_timer <= 0.0:
                rand_val = self._rng.random()
                saucer_type = Saucer.smallSaucerType if rand_val <= 0.4 else Saucer.largeSaucerType
                self.saucer = Saucer(self.stage, saucer_type, self.ship)
                self.stage.addSprite(self.saucer)
                self._saucer_spawn_timer = self._rng.uniform(6.0, 12.0)

    def exploding(self, dt):
        self.explodingCount += dt * 60.0
        if self.explodingCount > self.explodingTtl:
            self.gameState = "playing"
            for debris in list(self.ship.shipDebrisList):
                self.stage.removeSprite(debris)
            self.ship.shipDebrisList = []
            if self.attract_mode:
                self.createNewShip()
                self.lives = 1
            else:
                if self.lives <= 0:
                    self.ship.visible = False
                else:
                    self.createNewShip()

    def checkCollisions(self):
        if not self.ship:
            return
        shipHit, saucerHit = False, False

        for rock in list(self.rockList):
            rockHit = False

            if not self.ship.inHyperSpace and rock.collidesWith(self.ship):
                p = rock.checkPolygonCollision(self.ship)
                if p is not None:
                    shipHit = True
                    rockHit = True

            if self.saucer is not None:
                if rock.collidesWith(self.saucer):
                    saucerHit = True
                    rockHit = True

                if self.saucer.bulletCollision(rock):
                    rockHit = True

                if self.ship.bulletCollision(self.saucer):
                    saucerHit = True
                    self.score += self.saucer.scoreValue

            if self.ship.bulletCollision(rock):
                rockHit = True

            if rockHit:
                self.rockList.remove(rock)
                self.stage.removeSprite(rock)

                if rock.rockType == Rock.largeRockType:
                    playSound("explode1")
                    newRockType = Rock.mediumRockType
                    self.score += 50
                elif rock.rockType == Rock.mediumRockType:
                    playSound("explode2")
                    newRockType = Rock.smallRockType
                    self.score += 100
                else:
                    playSound("explode3")
                    self.score += 200

                if rock.rockType != Rock.smallRockType:
                    for _ in range(0, 2):
                        position = Vector2d(rock.position.x, rock.position.y)
                        newRock = Rock(self.stage, position, newRockType)
                        self.stage.addSprite(newRock)
                        self.rockList.append(newRock)

                self.createDebris(rock)

        if self.saucer is not None:
            if not self.ship.inHyperSpace:
                if self.saucer.bulletCollision(self.ship):
                    shipHit = True

                if self.saucer.collidesWith(self.ship):
                    shipHit = True
                    saucerHit = True

            if saucerHit:
                self.createDebris(self.saucer)
                self.killSaucer()

        if shipHit:
            self.killShip()

    def killShip(self):
        stopSound("thrust")
        playSound("explode2")
        self.explodingCount = 0.0
        if not self.attract_mode:
            self.lives -= 1
            if self.livesList:
                ship = self.livesList.pop()
                self.stage.removeSprite(ship)
        self.stage.removeSprite(self.ship)
        self.stage.removeSprite(self.ship.thrustJet)
        self.gameState = "exploding"
        self.ship.explode()

    def killSaucer(self):
        stopSound("lsaucer")
        stopSound("ssaucer")
        playSound("explode2")
        self.stage.removeSprite(self.saucer)
        self.saucer = None

    def createDebris(self, sprite):
        for _ in range(0, 25):
            position = Vector2d(sprite.position.x, sprite.position.y)
            debris = Debris(position, self.stage)
            self.stage.addSprite(debris)

    def checkScore(self):
        if self.attract_mode:
            return
        if self.score > 0 and self.score > self.nextLife:
            playSound("extralife")
            self.nextLife += 10000
            self.addLife(self.lives)

    def addLife(self, lifeNumber):
        self.lives += 1
        ship = Ship(self.stage)
        self.stage.addSprite(ship)
        ship.position.x = self.stage.width - (lifeNumber * 20) - 10
        ship.position.y = 20
        self.livesList.append(ship)

    def step(self, dt, builder, max_vectors):
        if dt < 0:
            dt = 0.0
        if dt > 0.2:
            dt = 0.2
        self._time += dt

        self.stage.moveSprites(dt)

        renderer = VectorRenderer(builder, self.maxc, self.aspect_x, max_vectors)
        self.stage.drawSprites(renderer.draw_poly)

        self.doSaucerLogic(dt)
        self.checkScore()

        if self.gameState == "playing":
            if self.attract_mode:
                self.attractControl(dt)
            self.checkCollisions()
            if len(self.rockList) == 0:
                self.levelUp()
        elif self.gameState == "exploding":
            self.exploding(dt)

        self.render_ui(builder)

    def render_ui(self, builder):
        def map_x(x):
            cx = self.maxc * 0.5
            return int(round(cx + (x - cx) * self.aspect_x))

        score_str = f"{self.score:06d}"
        builder.text_at(map_x(80), int(self.maxc - 80), score_str, size=1, rot=0)

        if self.attract_mode:
            title = "ASTEROIDS"
            title_x = map_x(self.maxc * 0.28)
            title_y = int(self.maxc * 0.78)
            builder.text_at(title_x, title_y, title, size=3, rot=0)

            if int(self._time * 1.5) % 2 == 0:
                builder.text_at(map_x(self.maxc * 0.32), int(self.maxc * 0.68), "INSERT COIN", size=1, rot=0)
