import numpy as np
from vehicleDynamicsModel.py import *

'''
TODO:
- fix h(), H() to account for vehicle orientation
- incorporate actual vehicle model in vehicle_model.py
'''

class feature(object):
    def __init__(self, model, state):
        self.model = model
        self.state = state

    def set_numbers(self, num, state_idx, meas_idx):
        self.number = num # index from 0, feature number
        self.state_idx = idx # index from 0, index of feature (1st state var) in state
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
        self.feature_loc_bounds = 2 # (meters) bounds where recognize as same feature
        # EKF vectors, matrices:
        self.x = np.zeros(shape=self.num_states)
        self.x_hat = np.zeros(shape=self.num_states)
        self.P = np.zeros(self.num_states)
        self.P_hat = np.zeros(self.num_states)
        # model, measurement covariances
        self.R = np.zeros(self.num_robot_states)
        self.gps_cov = np.zeros(2)
        self.gyro_cov = 0

    def handle_feature(self, feature):
        for i in range(len(self.known_features)):
            # check if we already have this feature (have one with same model w/in bounds):
            if feature.model == known_features[i].model:
                if self.check_in_bounds(feature.state, known_features[i].state):
                    feature.set_number(known_features[i].number, known_features[i].state_idx, known_features[i].meas_idx)
                return
        self.add_feature(feature) # if it's a new feature

    def add_feature(self, feature):
        prev_num_states = self.num_states
        # update feature, class variables:
        feature.set_numbers(self.num_features, self.num_states, self.num_meas)
        self.num_states += self.states_per_feature # add states for new feature
        self.num_meas += self.meas_per_feature
        self.num_features += 1
        self.known_features.append(feature.model)
        # update state x_hat with position of new feature = (0,0):
        x_new = np.concatenate(self.x_hat, np.zeros(shape=self.states_per_feature))
        self.x_hat = x_new
        # update cov P_hat with inf variances for new feature:
        P_new = np.identity(self.num_states) * np.inf
        P_new[0:prev_num_states][0:prev_num_states] = self.P_hat
        self.P_hat = P_new

    def check_in_bounds(self, new_loc, old_loc):
        result = True
        for i in range(len(new_loc)):
            if new_loc[i] >= old_loc[i] + self.feature_loc_bounds or new_loc[i] <= old_loc[i] - self.feature_loc_bounds:
                result = False
        return result

    def predict(self,u,dt):
        x_hat_robot,G_robot, _ = predictState(u,sp,dt) # what's called G here is F in vehicleDynamicsModel.py (del g/del x)
        self.x_hat = self.g(u, x_hat_robot)
        G = self.calc_G(u, G_robot)
        self.P_hat = self.R + self.pre_post_mult(G,self.P)

    def update(self,z):
        features = z[self.num_sensor_meas:-1]
        for i in range(len(features)):
            feature = z[num_features+i]
            self.handle_feature(feature)
        # create new sensor vector with feature states in place of feature objects:
        z_new = z[0:num_sensor_meas]
        for i in range(len(features)):
            np.concatenate([z_new, features[i].state])
        # update matrices
        H = self.calc_H(z_new, features)
        Q = self.calc_Q(z_new, len(features))
        K = self.calc_K(H, Q)
        # state update:
        self.x = self.x_hat + np.matmul(K,(z_new-h(features)))
        I = np.identity(K.matrix.shape[0])
        self.P = np.matmul((I-np.matmul(K,self.H)), self.P_hat)

    def g(self,u,x_hat_robot):
        x_hat = copy.copy(self.x)
        x_hat[0:self.num_robot_states] = x_hat_robot
        for i in range(self.num_robot_states, self.num_states):
            for j in range(self.states_per_feature):
                x_hat[i*self.states_per_feature+j] += x_hat[j]
        return x_hat

    def calc_G(self, u, G_robot):
        G = np.identity(self.num_states)
        G[0:num_robot_states][0:num_robot_states] = G_robot
        return G

    def h(self, features):
        z_pred = np.zeros(shape=self.num_sensor_meas+len(features))
        # sensor predictions:
        z_pred_robot = = np.array([self.x_hat[0], self.x_hat[1], self.x_hat[5]])
        z_pred[0:self.num_robot_states] = z_pred_robot
        # THIS IS INCORRECT: forgot to acct for orientation in relative position
        # feature predictions:
        for feature in features:
            state_idx = feature.state_idx
            feature_pred = self.x_hat[state_idx : state_idx+self.states_per_feature]
            z_idx = feature.meas_idx
            z_pred[z_idx : z_idx+self.meas_per_feature] = feature_pred # won't work for states_per_feature != meas_per_feature
        return z_pred

    def calc_H(self,x,z_new,features):
        # sensor portion:
        H = np.zeros(len(z_new),self.num_states)
        H[0][0],H[1][1],H[2][4] = 1
        # THIS IS INCORRECT: forgot to acct for orientation in relative position
        # feature/feature and feature/state portions:
        for feature in features:
            meas_idx = feature.meas_idx
            state_idx = feature.state_idx
            for j in range(self.states_per_feature):
                # identity (del feature)/(del feature) portion:
                H[meas_idx+j][state_idx+j] = -np.cos(x[2]) # -cos(th)
                # (del feature)/(del robot_state) portion:
                H[meas_idx][j] = 1
        return H

    def calc_Q(self, z, num_features):
        Q = np.zeros(len(z))
        Q[0:2][0:2] = self.gps_cov
        Q[2][2] = self.gyro_cov
        Q_features = np.identity(num_features) # modify
        Q[self.num_sensor_meas:-1][self.num_sensor_meas:-1] = Q_features
        pass

    def pre_post_mult(self,A,B):
        # returns A*B*A'
        return np.matmul(A, np.matmul(B, np.transpose(A)))

    def calc_K(self, H, Q):
        inv = np.linalg.inv(self.pre_post_mult(H,self.P_hat)+Q)
        K = np.matmul(P,np.matmul(np.transpose(H),inv))
        return K

    def update_feature_states(self, x_new):
        for feature in self.known_features:
            idx = feature.state_idx
            feature.state = x_new[idx:idx+self.states_per_feature]
        pass
