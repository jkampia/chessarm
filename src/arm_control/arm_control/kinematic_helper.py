# Helper class for calculating the forward and inverse kinematics of a 5dof robot arm geometrically

import math as m
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from .chessarm_colors import COLOR_RGB, pickRandomColor

np.set_printoptions(suppress=True, precision=2)

class ARM_5DOF:


    def __init__(self, joint_params, init_angles):

        if len(joint_params) < 5:
            raise Exception("Please provide all (5) limb lengths")
        
        self.joint_params = joint_params
        self.home_angles = init_angles.copy()
        self.joint_angles = init_angles.copy()
        self.joint_coordinates = [[0.0, 0.0, 0.0], [], [], [], [], []] # default mm
        self.joint_coordinates_m = [[0.0, 0.0, 0.0], [], [], [], [], []] # non default m
        self.joint_colors = [COLOR_RGB["red"], COLOR_RGB["green"], COLOR_RGB["blue"], COLOR_RGB["orange"], COLOR_RGB["purple"], COLOR_RGB["cyan"]]

        self.dh_params = {}
        self.dh_params["alpha"] = [m.pi/2, 0.0, 0.0, m.pi/2, 0.0]
        self.dh_params["d"] = [joint_params[0], 0.0, 0.0, 0.0, joint_params[3] + joint_params[4]]
        self.dh_params["a"] = [0.0, joint_params[1], joint_params[2], 0.0, 0.0]

    
    def solveIK(self, target_pose):
        # unpack inputs for clarity
        x, y, z, theta4, theta5 = target_pose
        l1, l2, l3, l4, l5 = self.joint_params

        # base yaw
        theta1 = m.atan2(y, x)

        # effective wrist offset
        wrist_len = l4 + l5
        horz = wrist_len * m.cos(theta4)
        x4 = x - horz * m.cos(theta1)
        y4 = y - horz * m.sin(theta1)
        z4 = z - wrist_len * m.sin(theta4) - l1

        # distance from shoulder to wrist
        r = x4*x4 + y4*y4 + z4*z4

        # elbow
        cos_phi = (r - l2**2 - l3**2) / (2 * l2 * l3)
        theta3 = -m.acos(cos_phi)

        # shoulder
        theta2 = m.asin(z4 / m.sqrt(r)) + m.atan(l3 * m.sin(theta3) / (l2 + l3 * m.cos(theta3)))

        # apply offsets to theta4 as it is provided in world frame
        # NOT last link frame, and FK expects last link frame
        j4_offset = theta2 + theta3 
        theta4 -= j4_offset
        

        return [theta1, theta2, theta3, theta4, theta5]
    

    def printArrayPretty(self, array):
        print(' '.join(f"{val:6.2f}" for val in array))
        print("")
    
    
    def printMatrixPretty(self, matrix):
        for row in matrix:
            print(' '.join(f"{val:6.2f}" for val in row))
        print("")
    

    def tfMatrix(self, i):

        # extract variables from dict for verbosity
        theta = self.dh_params["theta"][i]
        alpha = self.dh_params["alpha"][i]
        a = self.dh_params["a"][i]
        d = self.dh_params["d"][i]

        # from standard layout of denavit-hartenburg matrix
        return np.array([[m.cos(theta), -m.sin(theta)*m.cos(alpha), m.sin(theta)*m.sin(alpha), a*m.cos(theta)], 
                         [m.sin(theta), m.cos(theta)*m.cos(alpha), -m.cos(theta)*m.sin(alpha), a*m.sin(theta)], 
                         [0.0, m.sin(alpha), m.cos(alpha), d], 
                         [0.0, 0.0, 0.0, 1.0]])


    def solveFK(self, joint_angles):

        # set 'theta' field of dh matrix
        self.dh_params["theta"] = joint_angles
        joint_coordinates = [[0.0, 0.0, 0.0], [], [], [], [], []] # default mm
        
        # generate tf matrices
        product = self.tfMatrix(0)
        joint_coordinates[1] = self.extractCoordinates(product)
        for i in range(1, 5):
            product = np.matmul(product, self.tfMatrix(i))
            joint_coordinates[i+1] = self.extractCoordinates(product) 
        
        #self.printMatrixPretty(self.joint_coordinates_m)
        return joint_coordinates


    def computeJacobian(self, joint_angles, delta=1e-6):
        J = np.zeros((3, 3))
        f0 = self.solveFK(joint_angles)[4]
        for i in range(3):
            t = joint_angles.copy()
            t[i] += delta
            fi = self.solveFK(t)[4]
            J[:, i] = (fi - f0) / delta
        return J
    

    def gradientDescentIK(self, joint_angles, p_des, alpha=0.01, max_iter=1000, tol=1e-4):
        
        pose = np.array(joint_angles[:3], dtype=float)

        for i in range(max_iter):
            p_cur = self.solveFK(pose)
            p_cur = np.array(p_cur[4], dtype=float) # extract end effector xyz position (FK returns all joint positions)
            error = p_cur - p_des
            loss = m.sqrt(np.sum(error**2)) # euclidean distance
            if loss < tol:
                break

            J = self.computeJacobian(pose)
            gradient = 2 * J.T @ error
            pose -= alpha * gradient
            print(pose)

        return pose
            

    def extractCoordinates(self, tf_mat):
        # extract x, y, z position of a particular joint given its T0i tf matrix      
        return [tf_mat[0][3], tf_mat[1][3], tf_mat[2][3]]
    

    def extractCoordinates_m(self, tf_mat):
        # extract x, y, z position of a particular joint given its T0i tf matrix      
        return [tf_mat[0][3] / 1000.0, tf_mat[1][3] / 1000.0, tf_mat[2][3] / 1000.0]

