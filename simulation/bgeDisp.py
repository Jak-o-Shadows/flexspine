# -*- coding: utf-8 -*-
"""
Created on Mon May 22 20:12:13 2017

@author: Jak
"""
from __future__ import print_function	


import math

#blender
import mathutils
import bpy

  
    
def consistentNormals(obj):
    """Make normals consistent & all pointing out"""
    bpy.context.scene.objects.active = obj
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.editmode_toggle()
    
    
def removeDoubles(obj):
    bpy.context.scene.objects.active = obj
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.remove_doubles()   
    bpy.ops.object.editmode_toggle()

    

def makePlane(w, l):
    """
        4 points make a plane about the origin of size w, l
    """
    
    return [mathutils.Vector(x) for x in [(-w/2, -l/2, 0), (-w/2, l/2, 0), (w/2, l/2, 0), (w/2, -l/2, 0)]]   
    
    
def makeHexagon(coords, thickness, z, name):
    """
        make a hexagon with vertices that are coords    
    """
    #create vertices
    vertsTop = []
    vertsBottom = []
    for c in coords:
        vertsBottom.append(c)
        vertsTop.append((c[0], c[1], c[2] + thickness))
    verts = vertsTop + vertsBottom + [(0,0,0), (0,0,thickness)]
    #create faces
    faces = []
    #top and bottom faces
    for i in range(6):
        if i != 5:
            #Top
            a = i
            b = i+1
            c = 13 #last one - center point
            faces.append((a, b, c))
           #bottom
            a = i+6
            b = i+1 + 6
            c = 12
            faces.append((a, b, c))
        else:
            faces.append((5, 0, 13))
            faces.append((11, 6, 12))

    #side faces
    
    for i in range(6):
        if i!=5:
            a = i
            b = i+6
            d = i+1
            c = i+1 + 6
            faces.append((a, b, c, d))
        else:
            faces.append((5, 5+6, 0+6, 0))

    
    
    mesh = bpy.data.meshes.new(name)
    obj = bpy.data.objects.new(name, mesh)
    
    obj.location = [0,0,z]
    bpy.context.scene.objects.link(obj)

    mesh.from_pydata(verts, [], faces)
    mesh.update(calc_edges=True)
    
    consistentNormals(obj)
    
    return obj
    
    
    
def makeRecPrism(dim, originLocal, origin, name):
    """
        makes a rectangular prism of dim=w.h.d
    """

    #make planes. They will all be in the xy plane
    up = makePlane(dim[0], dim[1]) #xy plane
    left = makePlane(dim[1], dim[2]) #yz plane
    front = makePlane(dim[0], dim[2]) #xz plane
    
    #rotate to the correct
    eul = mathutils.Euler((math.radians(90), 0, math.radians(90)), 'XYZ')
    [x.rotate(eul) for x in left]
    #then move
    left = [mathutils.Vector([-dim[0]/2, 0, 0]) + x for x in left]
    print(left)
    right = [mathutils.Vector([dim[0], 0, 0]) + x for x in left]
    #front/back
    eul = mathutils.Euler((math.radians(90), 0, 0), 'XYZ')
    [x.rotate(eul) for x in front]
    #then move
    front = [mathutils.Vector([0, -dim[1]/2, 0]) + x for x in front]
    back = [mathutils.Vector([0, dim[1], 0]) + x for x in front]
    #up/down only have to move
    up = [mathutils.Vector([0, 0, -dim[2]/2]) + x for x in up]
    down = [mathutils.Vector([0, 0, dim[2]]) + x for x in up]

    numVerts = 6*4
    faces = [(i, i+1, i+2, i+3) for i in range(0, numVerts-1, 4)]    
    verts = up + down + left + right + front + back
    
    #want the origin to be at one end:
    verts = [x + originLocal for x in verts]
    
    
    
    mesh = bpy.data.meshes.new(name)
    obj = bpy.data.objects.new(name, mesh)
    
    obj.location = origin
    bpy.context.scene.objects.link(obj)
    
    mesh.from_pydata(verts, [], faces)
    mesh.update(calc_edges=True)
    
    consistentNormals(obj)
    
    #wasn't careful about doubles
    removeDoubles(obj)
    
    return obj


def makeLink(length, origin, initialRotation, name):
    """
        makes a link of length blah, with one point at origin, and initial rotation
    """
    widthRatio = 0.05
    depthRatio = widthRatio
    size = mathutils.Vector([widthRatio*length, depthRatio*length, length])
    originLocal = mathutils.Vector([0, 0, size[2]/2])   
    
    obj = makeRecPrism(size, originLocal, origin, name)
    
    #apply the initial rotation
    #first, get current location matrix
    ownMat = obj.matrix_world    
    #then convert the rotation to a transformation matrix
    rotMat = initialRotation.to_matrix()
    rotMat.resize_4x4()
    mat = ownMat*rotMat
    #set the rotation
    obj.matrix_world = mat
    #apply the rotation to mesh data
    obj.select = True
    #bpy.ops.object.transform_apply(rotation=True)
    
    return obj
    



