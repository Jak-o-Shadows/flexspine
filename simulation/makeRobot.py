# -*- coding: utf-8 -*-
"""
Created on Tue May 23 17:11:23 2017

@author: Jak
"""
import math

import mathutils

import bgeDisp

def makeLeg(L1, L2, L3, resting1, resting2, resting3, label="%s", origin=mathutils.Vector([0, 0, 0])):
    """
        make a leg like:
             /
            /
           /
          *
           \
            \
             \
           ___\
           
       resting 1 is wrt to horizontal plane
       other angles are wrt above link
    """
        
    tibiaRot = mathutils.Euler((resting1, 0, 0)).to_matrix()
    tibiaOrigin = origin
    tibia =bgeDisp.makeLink(L1, tibiaOrigin, tibiaRot.to_quaternion(), label % ("tibia",))
    
    femurRot = mathutils.Euler((resting2, 0, 0)).to_matrix() * tibiaRot
    femurOriginOffset = mathutils.Vector([0, 0, L1])
    femurOriginOffset.rotate(tibiaRot)
    femurOrigin = tibiaOrigin + femurOriginOffset
    femur = bgeDisp.makeLink(L2, femurOrigin, femurRot.to_quaternion(), label % ("femur",))
    
    ankleRot = mathutils.Euler((resting3, 0, 0)).to_matrix()*femurRot
    ankleOriginOffset = mathutils.Vector([0, 0, L2])
    ankleOriginOffset.rotate(femurRot)
    ankleOrigin = femurOrigin+ankleOriginOffset
    ankle = bgeDisp.makeLink(L3, ankleOrigin, ankleRot.to_quaternion(), label % ("ankle",))
    
    
    


legLengths = [5, 7, 2]
legRestingAngles = [math.radians(x) for x in [120, 110, 40]]


W = 5
D = 30
origin_z = 15
legs = []
nameKeys = {}
nameKeys[0, 0] = "backLeft"
nameKeys[0, 1] = "frontLeft"
nameKeys[1, 0] = "backRight"
nameKeys[1, 1] = "frontRight"

for w in range(2):
    for d in range(2):
        origin=  mathutils.Vector([(W/2)*(-1)**w, (D/2)*(-1)**d, origin_z])
        name = nameKeys[w, d] + "_%s"
        legs.append(makeLeg(*legLengths, *legRestingAngles, name, origin))

#legs = [makeLeg(*legLengths, *legRestingAngles, str(x) + "%s", mathutils.Vector([5*(-1)**x, 0, 0])) for x in range(2)]
#makeLeg(5, 7,  2, math.radians(120), math.radians(110), math.radians(40))