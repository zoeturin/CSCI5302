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
        self.curr_pose = None
        self.feature_pose_list = []

    def update_map(self, pose, feature_pose_buffer):
        '''
        Updates vehicle pose and stores feature poses

        @param pose: pose of vehicle (x, y, theta)
        @param feature_pose_buffer: feature poses
        '''
        self.curr_pose = pose

        for p in feature_pose_buffer:
            self.feature_pose_list.append(p)
        
    def visualize(self):
        '''
        Visualize map with feature poses
        '''
        for pt in feature_pose_list:
            plt.scatter(pt[0], pt[1], s=20)
        plt.imshow(self.occ_map, cmap = 'binary')
        plt.show()
