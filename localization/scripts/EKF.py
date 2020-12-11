import numpy as np
import copy
from vehicleDynamicsModel import *

'''
TODO:
- adjust feature measurement covariance (Q_features)
- adjust model prediction covariance (R)
'''

class feature(object):
    def __init__(self, model, state):
        self.model = model
        self.state = state

    def set_numbers(self, num, state_idx, meas_idx):
        self.number = num # index from 0, feature number
        self.state_idx = state_idx # index from 0, index of feature (1st state var) in state
        self.meas_idx = meas_idx # index from 0, index of feature (1st state var) in measurement vec

class EKF_solver(object):
    def __init__(self):
        # state and sensors:
        self.num_robot_states = 6 # x,z,th,x',z',th'
        self.num_states = self.num_robot_states
        self.num_sensor_meas = 2+1 # x,z from GPS, th' from gyro
        self.num_meas = self.num_sensor_meas
        # features:
        self.states_per_feature = 2 # x,z
        self.meas_per_feature = 2 # x,z
        self.known_features = []
        self.num_features = 0
        self.feature_loc_bounds = 4 # (meters) bounds where recognize as same feature
        self.blacklisted_features = ['road line', 'crash barrier', 'road border'] #['road line']
        # EKF vectors, matrices:
        self.x = np.zeros(self.num_states)
        self.x[2] = np.pi # for starting facing the opposite direction
        self.x_hat = np.zeros(self.num_states)
        self.P = np.identity(self.num_states)*.1
        self.P_hat = np.identity(self.num_states)*.1
        # model, measurement covariances:
        self.R = np.identity(self.num_states)*.1 # gets updated in self.add_feature
        self.gps_cov = np.identity(2)*0.1
        self.gyro_cov = 0.1
        self.featureQ = 0.1

    def set_model(self, model):
        self.model = model

    def handle_feature(self, feature, meas_idx):
        for i in range(len(self.known_features)):
            known_feature = self.known_features[i]
            # check if we already have this feature (have one with same model w/in bounds):
            if feature.model == known_feature.model:
                if self.check_in_bounds(feature.state, known_feature.state):
                    self.known_features[i] = feature
                    feature.set_numbers(known_feature.number, known_feature.state_idx, meas_idx)
                    return
        if feature.model not in self.blacklisted_features:
            print(feature.model)
            print(feature.state)
            self.add_feature(feature, meas_idx) # if it's a new feature
        else:
            feature.model = None

    def add_feature(self, feature, meas_idx):
        prev_num_states = self.num_states
        # update feature, class variables:
        feature.set_numbers(self.num_features, self.num_states, meas_idx)
        self.num_states += self.states_per_feature # add states for new feature
        self.num_meas += self.meas_per_feature
        self.num_features += 1
        self.known_features.append(feature)
        # update state x_hat with position of new feature = (0,0):
        x_new = np.concatenate((self.x_hat, np.zeros(self.states_per_feature)))
        self.x_hat = x_new
        # update cov P_hat with inf variances for new feature:
        P_new = np.identity(self.num_states) * 9999999
        P_new[0:prev_num_states,0:prev_num_states] = self.P_hat
        self.P_hat = P_new
        # update covariance R for new feature:
        R_new = np.identity(self.num_states)
        R_new[0:prev_num_states,0:prev_num_states] = self.R
        self.R = R_new

    def check_in_bounds(self, new_loc, old_loc):
        result = True
        for i in range(len(new_loc)):
            if new_loc[i] >= old_loc[i] + self.feature_loc_bounds or new_loc[i] <= old_loc[i] - self.feature_loc_bounds:
                result = False
        return result

    def predict(self,u,dt):
        robot_state = self.x[0:self.num_robot_states]
        x_hat_robot,G_robot, _ = self.model.predictState(u,robot_state,dt) # what's called G here is F in vehicleDynamicsModel.py (del g/del x)
        self.x_hat = self.g(u, x_hat_robot)
        G = self.calc_G(u, G_robot)
        self.P_hat = self.R + self.pre_post_mult(G,self.P)
        # print(self.P_hat)

    def update(self,z):
        features = z[self.num_sensor_meas:]
        self.num_meas = self.num_sensor_meas + len(features)*self.meas_per_feature
        i=0
        while i < len(features):
            meas_idx = self.num_sensor_meas + i*self.meas_per_feature
            self.handle_feature(features[i], meas_idx)
            if features[i].model is None:
                self.num_meas -= self.meas_per_feature
                z = np.concatenate([z[0:self.num_sensor_meas+i],z[self.num_sensor_meas+i+self.meas_per_feature:]])
                features = np.concatenate([features[0:i],features[i+1:]])
                i -= 1
            i += 1
        self.num_meas = self.num_sensor_meas + len(features)*self.meas_per_feature
        # create new sensor vector with feature states in place of feature objects:
        z_new = z[0:self.num_sensor_meas]
        for i in range(len(features)):
            z_new = np.concatenate([z_new, features[i].state])
        # update matrices
        H = self.calc_H(self.x_hat, z_new, features)
        Q = self.calc_Q(z_new, len(features))
        K = self.calc_K(H, Q)
        # state update:
        self.x = self.x_hat + np.matmul(K,(z_new-self.h(features)))
        self.x[2] = self.wrap_angle(self.x[2]);
        I = np.identity(K.shape[0])
        self.P = np.matmul((I-np.matmul(K,H)), self.P_hat)
        self.update_feature_states(self.x)

    def g(self,u,x_hat_robot):
        x_hat = copy.copy(self.x)
        x_hat[0:self.num_robot_states] = x_hat_robot
        x_hat[2] = self.wrap_angle(x_hat[2])
        for i in range(self.num_robot_states, self.num_states, 2):
            for j in range(self.states_per_feature):
                x_hat[i+j] += x_hat[j]
        return x_hat

    def wrap_angle(self,a):
        return ( (a+np.pi)%(2*np.pi) ) - np.pi;

    def calc_G(self, u, G_robot):
        G = np.identity(self.num_states)
        G[0:self.num_robot_states,0:self.num_robot_states] = G_robot
        return G

    def h(self, features):
        z_pred = np.zeros(self.num_sensor_meas+len(features)*self.meas_per_feature)
        # sensor predictions:
        z_pred_robot = np.array([self.x_hat[0], self.x_hat[1], self.x_hat[5]])
        z_pred[0:self.num_sensor_meas] = z_pred_robot
        # THIS IS INCORRECT: forgot to acct for orientation in relative position
        # ^ Zoe: jk I think I did fix it and forgot to remove this (see cos below)
        # feature predictions:
        for feature in features:
            state_idx = feature.state_idx
            feature_state_pred = self.x_hat[state_idx : state_idx+self.states_per_feature]
            feature_z_pred = [(self.x_hat[i]-feature_state_pred[i])/np.cos(self.x_hat[2]) for i in range(self.states_per_feature)]
            # ^ Zoe: something wrong with this?
            z_idx = feature.meas_idx
            z_pred[z_idx : z_idx+self.meas_per_feature] = feature_z_pred # won't work for states_per_feature != meas_per_feature
        return z_pred

    def calc_H(self,x,z_new,features):
        # sensor portion:
        print(features)
        H = np.zeros((len(z_new),self.num_states))
        H[0,0] = 1
        H[1,1] = 1
        H[2,4] = 1
        # feature/feature and feature/state portions:
        for feature in features:
            print(feature.model)
            print(feature.state_idx)
            meas_idx = feature.meas_idx
            state_idx = feature.state_idx
            for j in range(self.states_per_feature):
                # identity (del feature)/(del feature) portion:
                H[meas_idx+j,state_idx+j] = -np.cos(x[2]) # -cos(th)
                # (del feature)/(del robot_state) portion:
                H[meas_idx+j,j] = 1
        return H

    def calc_Q(self, z, num_features):
        Q = np.zeros((len(z),len(z)))
        Q[0:2,0:2] = self.gps_cov
        Q[2,2] = self.gyro_cov
        Q_features = np.identity(num_features*self.meas_per_feature)*self.featureQ # modify
        Q[self.num_sensor_meas:,self.num_sensor_meas:] = Q_features
        return Q

    def pre_post_mult(self,A,B):
        # returns A*B*A'
        return np.matmul(A, np.matmul(B, np.transpose(A)))

    def calc_K(self, H, Q):
        # print("H:")
        # print(H)
        # print("Phat:")
        # print(self.P_hat);
        init_mat = self.pre_post_mult(H,self.P_hat)+Q
        #print(init_mat)
        inv = np.linalg.inv(init_mat)
        K = np.matmul(self.P_hat,np.matmul(np.transpose(H),inv))
        return K

    def update_feature_states(self, x_new):
        for feature in self.known_features:
            idx = feature.state_idx
            feature.state = x_new[idx:idx+self.states_per_feature]
        pass
