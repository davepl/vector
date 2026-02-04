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

import math
import random

from .shooter import Shooter
from .soundManager import playSound, playSoundContinuous, stopSound
from .util.vector2d import Vector2d
from .util.vectorsprites import VectorSprite

SHIP_SCALE = 6.0

# comment
class Ship(Shooter):
            
    # Class attributes            
    acceleration = 0.2
    decelaration= -0.005
    maxVelocity = 10
    turnAngle = 6
    bulletVelocity = 26.0
    maxBullets = 4
    bulletTtl = 105
    
    def __init__(self, stage):
        
        position = Vector2d(stage.width/2, stage.height/2)
        heading = Vector2d(0,0)
        self.thrustJet = ThrustJet(stage, self)        
        self.shipDebrisList = [] 
        self.visible = True   
        self.inHyperSpace = False
        base_points = [(0, -10), (6, 10), (3, 7), (-3, 7), (-6, 10)]
        pointlist = [(x * SHIP_SCALE, y * SHIP_SCALE) for x, y in base_points]
        
        Shooter.__init__(self, position, heading, pointlist, stage)        

    def draw(self):
        if self.visible:
            if not self.inHyperSpace:
                VectorSprite.draw(self)                
            else:                 
                self.hyperSpaceTtl -= 1                
                if self.hyperSpaceTtl == 0:
                    self.inHyperSpace = False
                    self.color = (255,255,255)
                    self.thrustJet.color = (255,255,255)
                    self.position.x = random.uniform(0.0, float(self.stage.width))
                    self.position.y = random.uniform(0.0, float(self.stage.height))
                    position = Vector2d(self.position.x, self.position.y)        
                    self.thrustJet.position = position
                    
        
        return self.transformedPointlist
    
    def rotateLeft(self, step=1.0):
        self.angle += self.turnAngle * step
        self.thrustJet.angle += self.turnAngle * step
        
    def rotateRight(self, step=1.0):
        self.angle -= self.turnAngle * step
        self.thrustJet.angle -= self.turnAngle * step
        
    def increaseThrust(self, step=1.0):
        playSoundContinuous("thrust")
        if math.hypot(self.heading.x, self.heading.y) > self.maxVelocity:
            return
        
        dx = self.acceleration * step * math.sin(math.radians(self.angle)) * -1
        dy = self.acceleration * step * math.cos(math.radians(self.angle)) * -1
        self.changeVelocity(dx, dy)
    
    def decreaseThrust(self, step=1.0):
        stopSound("thrust")
        if (self.heading.x == 0 and self.heading.y == 0):
            return;
        
        dx = self.heading.x * self.decelaration * step
        dy = self.heading.y * self.decelaration * step
        self.changeVelocity(dx, dy)
    
    def changeVelocity(self, dx, dy):
        self.heading.x += dx;
        self.heading.y += dy;
        self.thrustJet.heading.x += dx
        self.thrustJet.heading.y += dy
        
    def move(self, step=1.0):
        VectorSprite.move(self, step)
        self.decreaseThrust(step)

    # Break the shape of the ship down into several lines
    # Ship shape - [(0, -10), (6, 10), (3, 7), (-3, 7), (-6, 10)]    
    def explode(self):                                
        def scale_pts(points):
            return [(x * SHIP_SCALE, y * SHIP_SCALE) for x, y in points]

        self.addShipDebris(scale_pts([(0, -10), (6, 10)]))
        self.addShipDebris(scale_pts([(6, 10), (3, 7)]))
        self.addShipDebris(scale_pts([(3, 7), (-3, 7)]))
        self.addShipDebris(scale_pts([(-3, 7), (-6, 10)]))
        self.addShipDebris(scale_pts([(-6, 10), (0, -10)]))
        
        
    # Create a peice of ship debris    
    def addShipDebris(self, pointlist):
        heading = Vector2d(0, 0)
        position = Vector2d(self.position.x, self.position.y)
        debris = VectorSprite(position, heading, pointlist, self.angle)

        # Add debris to the stage
        self.stage.addSprite(debris)  
                
        # Calc a velocity moving away from the ship's center        
        centerX = debris.boundingRect.centerx
        centerY = debris.boundingRect.centery
        
        # Alter the random values below to change the rate of expansion
        debris.heading.x = ((centerX - self.position.x) + 0.1) / random.uniform(20, 40)
        debris.heading.y = ((centerY - self.position.y) + 0.1) / random.uniform(20, 40)        
        self.shipDebrisList.append(debris)
        

    # Set the bullet velocity and create the bullet
    def fireBullet(self):
        if self.inHyperSpace == False:
            vx = self.bulletVelocity * math.sin(math.radians(self.angle)) * -1
            vy = self.bulletVelocity * math.cos(math.radians(self.angle)) * -1
            heading = Vector2d(vx, vy)
            Shooter.fireBullet(self, heading, self.bulletTtl, self.bulletVelocity)
            playSound("fire")
            
    #         
    def enterHyperSpace(self):
      if not self.inHyperSpace:
          self.inHyperSpace = True
          self.hyperSpaceTtl = 100
          self.color = (0,0,0)
          self.thrustJet.color = (0,0,0)
                        
                        
# Exhaust jet when ship is accelerating
class ThrustJet(VectorSprite):
    pointlist = [(-3 * SHIP_SCALE, 7 * SHIP_SCALE), (0 * SHIP_SCALE, 13 * SHIP_SCALE), (3 * SHIP_SCALE, 7 * SHIP_SCALE)]
    
    def __init__(self, stage, ship):
        position = Vector2d(stage.width/2, stage.height/2)
        heading = Vector2d(0,0)
        self.accelerating = False
        self.ship = ship
        VectorSprite.__init__(self, position, heading, self.pointlist)
        
    def draw(self):
        if self.accelerating and self.ship.inHyperSpace == False:
            self.color = (255,255,255)         
        else:
            self.color = (0,0,0)
            
        VectorSprite.draw(self)
        return self.transformedPointlist
