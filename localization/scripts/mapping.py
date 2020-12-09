#!/usr/bin/env python2

import rospy
from webots_ros.srv import *
import numpy as np

class OccMap():
    def __init__(self, x_min, x_max, y_min, y_max, res):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.width = x_max - x_min
        self.height = y_max - y_min
        self.res = res
        self.grid_size = self.width * self.height
        self.occ_map = np.zeros((int(self.width/res), (int(self.height/res))))
        self.occupied_threshold = 0.9 # what should this be?
        self.free_threshold = 0.2 # what should this be?
        self.alpha = 1.0 # what should this be?
        self.beta = 5 * np.pi/180 # what should this be?

    def update_map(self, pose, z):
        '''
        Updates log-odd probabilities of cells in occupancy grid

        @param pose: pose of vehicle (x, y, theta)
        @param z: feature measurements (x, y, theta)
        '''
        self.curr_pose = pose
        occ = self.occ_map.copy()
        occ[0,:] -= pose[0] # find all x-positions of occupancy grid
        occ[1,:] -= pose[1] # find all y-positions of occupancy grid
        r = np.linalg.norm(occ, axis=0) # range for center of mass of cells
        phi = np.atan2(occ[1,:], occ[0,:]) - pose[2] # calculate phi 

        for z_i in z: # loop through all sensor measurements
            l_free = (abs(phi - z_i[2]) > self.beta/2) # find free cell indices
            l_occ = (abs(r - z_i) < self.alpha/2) # find occupied cell indices
            self.occ_map[l_free] += self.free_threshold # change log-odds probability of free cells
            self.occ_map[l_occ] += self.occupied_threshold # change log-odds probability of occupied cells
            
if __name__ == '__main__':
    occMap = OccMap(-2, 2, -2 , 2, 1)
