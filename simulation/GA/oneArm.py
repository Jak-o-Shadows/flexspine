# -*- coding: utf-8 -*-
"""
Created on Sat May 27 20:55:32 2017

@author: Jak
"""

import math

import numpy

import matplotlib.pyplot as plt


def fitness(objective, genotypes):
    endEffect = calcPos(genotypes)
    
    
    cost = numpy.sum(numpy.square(numpy.tile(objective, [50, 1]) - endEffect.transpose()), axis=1)
    
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(cost)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Cost")
    
    return numpy.min(cost)
            
    
def calcPos(genotypes):
    """
        Given a legs chromosomes, what is the position of the end effector
        throughout the  time zero to tau?
    """
    c = genotypes
    #find bezier control points
    points1 = numpy.array([x for x in c[0:4]]).transpose()
    points2 = numpy.array([x for x in c[4:9]]).transpose()
    
    #find angles from bezier curves    
    angles1 = bezierEval(points1)[1, :] #dont' care about the x pos
    angles2 = bezierEval(points2)[1, :] # dont' care about the x pos
    
    
    #display the angle
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(angles1)
    ax.plot(angles2)
    ax.legend(["Angle 1", "Angle 2"])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (rad)")
    
    
    end = twoLeg(1, 1, angles1, angles2)

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set_xlabel("y")
    ax.set_ylabel("x")
    ax.plot(end[0, :], end[1, :])
    
    
    return end
    
    
    
def twoLeg(L1, L2, th1, th2):
    """
           B
           \
            \ang2/
             \  /
             A\/
              /
             /
            /  ang1
    ______O/_______
    """
    
    th1 = numpy.radians(th1)
    th2 = numpy.radians(th2)
    
    
    origin = numpy.array([0, 0])
    origin.shape = (2, 1) #nice to be explicit here
    A = origin + L1*numpy.array([numpy.cos(th1), numpy.sin(th1)])
    B = A + L2*numpy.array([numpy.cos(th2+th1), numpy.sin(th2+th1)])

    
    return B
    
    



def bezierEval(points):
    N = 50
    t = numpy.linspace(0, 1, N)
    if points.shape[1] == 4:
        #it is a cubic bezier curve
        c1 = 3*(points[:, 1] - points[:, 0])
        c2 = 3*(points[:, 0] - 2*points[:, 1] + points[:, 2])
        c3 = -points[:, 0] + 3*(points[:, 1] - points[:, 2]) + points[:, 3]
        
        
        c1.shape = (2, 1)
        c2.shape = (2, 1)
        c3.shape = (2, 1)
        
        
        Q = numpy.power(t, 3)*c3 + numpy.square(t)*c2 + t*c1+ numpy.tile(points[:, 0], [N, 1]).transpose()
        return Q
    else:
        raise Exception('Only cubic beziers are supported')
        

def geneticAlgorithm():
    
    numThings = 10
    numChromosomes = 2*4*2 #each point is 2, each angle has 4, two angles
    things = 90*(numpy.random.rand(numThings, numChromosomes) - 0.5)
    #fix starting and ending chromosomes
    #   The x-location of the start and end points are zero and 60 for each of the 
    #   bezier curves
    #curve 1
    things[:, 0] = 0*things[:, 1]
    things[:, 2*4-2] = 0*things[:, 2*4-2] + 60
    #curve 2
    things[:, 2*4] = 0*things[:, 2*4]
    things[:, 2*4*2-2] = 0*things[:, 2*4*2-2] + 60
    #and they are continous/looping -> y values match
    things[:, 1] = things[:, 2*4-1] #curve 1
    things[:, 2*4+1] = things[:, 2*4*2-1] #curve 2
    #hardcode one for kicks
    things[1, :] = numpy.array([0, 90, 0, 90, 0, 90, 0, 93, 0, 0, 0, 0, 0, 0, 0, 0])
    
    for i in xrange(numThings):
        genotype = things[i, :]
        genotype.shape = (8, 2) #doing this, instead of (2, 8), puts the [x, y, x, y] into the right order
        #print [list(x) for x in genotype]
        print i, fitness(numpy.array([0, 2]), genotype)
        
    plt.show()
    




        
def main():
    points = numpy.array([[0, 0], [15, 30], [45, 30], [60, 0]]).transpose()
    print points.shape
    
#    genotypes = [(0, 0), (15, 10), (45, 0), (60, 0), (0, 0), (15, 15), (45, 90), (60, 0)]
    #fitness(numpy.array([0, 2]), genotypes)
    #plt.show()
    
    
    #print twoLeg(1, 1, 90, 0)    
    
    geneticAlgorithm()



if __name__ == "__main__":
    main()


    