import numpy as np

class EKF_solver(object):
    def __init__:
        self.num_robot_states = 6 #x,y,th,x',v',th'
        self.num_states = num_robot_states
        self.x = np.zeros(shape=num_robot_states) 
        self.P = np.zeros(num_robot_states)
        self.R = np.zeros(num_robot_states)
    
    def predict(x,u):
        x_hat = g(x,u)
        G = calc_G(x)
        P_hat = self.R + pre_post_mult(G,P)
        return x_hat, P_hat
        
    def update(x_hat,z,K):
        x_new = x_hat + np.matmul(K,(z-h(x_hat)))
        I = np.identity(K.matrix.shape[0])
        P_new = np.matmul((I-np.matmul(K,H)), P)
    
    def g(x,u):
        return x
    
    def calc_G(x):
        G = np.zeros(num_states)
        return G
        
    def h(z):
        return z
    
    def calc_H(x):
        H = np.zeros(num_states)
        return H
        
    def pre_post_mult(A,B):
        return np.matmul(A, np.matmul(B, np.transpose(B)))
    
    def calc_K():
        inv = np.linalg.inv(pre_post_mult(H,P)+Q)
        K = np.matmul(P,np.matmul(np.transpose(H),inv))
        return K
        