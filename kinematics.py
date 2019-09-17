import numpy as np
#expm is a matrix exponential function
from scipy.linalg import expm

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""

DH_table = [[0, 0, 74.76, 0],
            [0, -90, 40.64, -90],
             [99.58, 0, 0, -90],
             [0, 90, 0, 90],
             [0, -90, 69.82+40.64, 0]]


def FK_dh(joint_angles, link):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the 
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
    T = np.identity(4)
    
    for i, DH in enumerate(DH_table):
        theta = np.radians(DH[1]) + joint_angles[i]
        phi = np.radians(DH[3])
        
        T1 = np.dot(translate(DH[0], "z"),rotation(theta, "z"))
        T2 = np.dot(T1, translate(DH[2], "x"))
        Ti = np.dot(T2, rotation(phi, "x"))
        
        # print(Ti)

        # print(Ti)
        
        T = np.dot(T, Ti)

    # print(T)

    return T

def FK_pox(joint_angles):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm
    using product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of the 
    desired link

    note: phi is the euler angle about y in the base frame

    """
    pass

def IK(pose):
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    pass


def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """
    pass

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """
    pass




def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass


def translate(dist, axis):
    I3 = np.identity(3)
    if axis == "x":
        D = np.array([[dist], [0], [0]])
    if axis == "y":
        D = np.array([[0], [dist], [0]])
    if axis == "z":
        D = np.array([[0], [0], [dist]])
    
    RD = np.concatenate((I3, D), axis=1)
    
    T = np.concatenate((RD,[[0,0,0,1]]), axis=0)
    return T

def rotation(theta, axis):
    D = np.zeros((3,1))
    
    # theta = theta * np.pi/180

    if axis == "z":
        R = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta),0], [0, 0, 1]])
    if axis == "x":
        R = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
    if axis == "y":
        R = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

    RD = np.concatenate((R,D), axis=1)

    T = np.concatenate((RD,[[0,0,0,1]]), axis=0)
    return T