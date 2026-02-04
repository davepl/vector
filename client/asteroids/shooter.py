#
#    Copyright (C) 2008  Nick Redshaw
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#    Copyright (C) 2008  Nick Redshaw
#

from .util.vector2d import Vector2d
from .util.vectorsprites import Point, VectorSprite

class Shooter(VectorSprite):    
    
    def __init__(self, position, heading, pointlist, stage):
        VectorSprite.__init__(self, position, heading, pointlist)     
        self.bullets = []        
        self.stage = stage
    
    def fireBullet(self, heading, ttl, velocity):
        if (len(self.bullets) < self.maxBullets):                          
            position = Vector2d(self.position.x, self.position.y)
            newBullet = Bullet(position, heading, self, ttl, velocity, self.stage)
            self.bullets.append(newBullet)
            self.stage.addSprite(newBullet)
            return True

    def bulletCollision(self, target):
        collisionDetected = False
        for bullet in self.bullets:
            if bullet.ttl > 0 and target.collidesWith(bullet):
                collisionDetected = True
                bullet.ttl = 0
        
        return collisionDetected

# Bullet class    
class Bullet(Point):
    
    def __init__(self, position, heading, shooter, ttl, velocity, stage):
        Point.__init__(self, position, heading, stage)
        self.shooter = shooter
        self.ttl = ttl
        self.velocity = velocity
        self.bright_passes = 5
        
    def move(self, step=1.0):
        Point.move(self, step)
        if self.ttl <= 0 and self in self.shooter.bullets:
            self.shooter.bullets.remove(self)
