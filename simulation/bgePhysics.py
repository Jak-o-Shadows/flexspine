# -*- coding: utf-8 -*-
"""
Created on Tue May 23 18:25:55 2017

@author: Jak
"""



import bpy




def makeRigidBody(obj):
    """Changes the object physics setting to be rigid body
    Uses operator - bad?
    """
    obj.game.physics_type = "RIGID_BODY"
    obj.game.use_ghost = True
    obj.game.use_sleep = True # disable sleeping
    
    
    
    
def makeJoint(obj1, obj2, pos, constraints)