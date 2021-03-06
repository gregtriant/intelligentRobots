#!/usr/bin/env python

import sys
import rospy
import random
import math
import time
import numpy as np
from timeit import default_timer as timer
from utilities import RvizHandler
from utilities import OgmOperations
from utilities import Print
from brushfires import Brushfires
from topology import Topology
import scipy
from path_planning import PathPlanning


# Class for selecting the next best target
class TargetSelection:
    
    # Constructor
    def __init__(self, selection_method):
        self.goals_position = []
        self.goals_value = []
        self.omega = 0.0
        self.radius = 0
        self.method = selection_method

        self.brush = Brushfires()
        self.topo = Topology()
        self.path_planning = PathPlanning()


    def selectTarget(self, init_ogm, coverage, robot_pose, origin, \
        resolution, force_random, tries):
        
        target = [-1, -1]

        ######################### NOTE: QUESTION  ##############################
        # Implement a smart way to select the next target. You have the 
        # following tools: ogm_limits, Brushfire field, OGM skeleton,
        # topological nodes.

        # CHALLENGE 6

        # Find only the useful boundaries of OGM. Only there calculations
        # have meaning
        ogm_limits = OgmOperations.findUsefulBoundaries(init_ogm, origin, resolution)

        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)

        # Calculate Brushfire field
        tinit = time.time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        Print.art_print("Brush time: " + str(time.time() - tinit), Print.ORANGE)

        # Calculate skeletonization
        tinit = time.time()
        skeleton = self.topo.skeletonizationCffi(ogm, \
                   origin, resolution, ogm_limits)
        Print.art_print("Skeletonization time: " + str(time.time() - tinit), Print.ORANGE)

        # Find topological graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, \
                resolution, brush, ogm_limits)
        Print.art_print("Topo nodes time: " + str(time.time() - tinit), Print.ORANGE)
        
        # Visualization of topological nodes
        vis_nodes = []
        for n in nodes:
            vis_nodes.append([
                n[0] * resolution + origin['x'],
                n[1] * resolution + origin['y']
            ])
        RvizHandler.printMarker(\
            vis_nodes,\
            1, # Type: Arrow
            0, # Action: Add
            "map", # Frame
            "art_topological_nodes", # Namespace
            [0.3, 0.4, 0.7, 0.5], # Color RGBA
            0.1 # Scale
        )

        # Robot Position
        x = robot_pose['x_px'] + abs(origin['x'] / resolution)
        y = robot_pose['y_px'] + abs(origin['y'] / resolution)


        # Random point
        if self.method == 'random' and force_random == True:
          target = self.selectRandomTarget(ogm, coverage, brush, ogm_limits)
        
        # CHALLENGE 6
        # ADDED
        elif (tries == 0):
            target = self.MAXTargetSel(ogm, coverage, brush, ogm_limits, nodes, x, y)    
        elif (tries == 1):
            target = self.MINTargetSel(ogm, coverage, brush, ogm_limits, nodes, x, y)  
        else:
            target = self.selectRandomTarget(ogm, coverage, brush, ogm_limits)
        #     target = self.myTargetSelection(ogm, coverage, brush, ogm_limits, nodes, x, y) 
        ########################################################################

        return target

    def selectRandomTarget(self, ogm, coverage, brushogm, ogm_limits):
      # The next target in pixels
        tinit = time.time()
        next_target = [0, 0] 
        found = False
        while not found:
          x_rand = random.randint(0, ogm.shape[0] - 1)
          y_rand = random.randint(0, ogm.shape[1] - 1)
          if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and \
              brushogm[x_rand][y_rand] > 5:
            next_target = [x_rand, y_rand]
            found = True
        Print.art_print("Select random target time: " + str(time.time() - tinit), \
            Print.ORANGE)
        return next_target

        
    def MAXTargetSel(self, ogm, coverage, brushogm, ogm_limits, nodes, x, y):
        tinit=time.time()
        next_target=[0,0]
        a=[]
        b=[]
        d=[]
        print(x)
        print(y)
        for n in nodes:
            #The max distance topo node from robot
            temp = (math.pow(x - n[0], 2) + math.pow(y - n[1], 2))
            a.append(temp)
            b.append(n[0])
            d.append(n[1])

        c = a.index(max(a))
        next_target=[b[c],d[c]]
        
        Print.art_print("Select MAX target time: " + str(time.time() - tinit), \
                Print.ORANGE)
        print(next_target)
        return next_target

    def MINTargetSel(self, ogm, coverage, brushogm, ogm_limits, nodes, x, y):
        tinit=time.time()
        next_target=[0,0]
        a=[]
        b=[]
        d=[]
        print(x)
        print(y)
        for n in nodes:
            #The min distance topo node from robot
            temp = - (math.pow(x - n[0], 2) + math.pow(y - n[1], 2))
            a.append(temp)
            b.append(n[0])
            d.append(n[1])

        c = a.index(max(a))
        next_target=[b[c],d[c]]
        
        Print.art_print("Select MIN target time: " + str(time.time() - tinit), \
                Print.ORANGE)
        print(next_target)
        return next_target


   