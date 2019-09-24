import numpy as np
#expm is a matrix exponential function
from scipy.linalg import expm

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""


def FK_dh(joint_angles, DH_table):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the 
    desired link


    note: phi is the euler angle about the y-axis in the base frame

    """
    T = np.identity(4)
    
    for i, DH in enumerate(DH_table):
        theta = np.radians(DH[0]) + joint_angles[i]
        phi = np.radians(DH[3])
        
        Ti = T_frrom_DH(theta, DH[1], DH[2], phi)
     
        T = np.dot(T, Ti)
    # print(T)
    return T

def T_frrom_DH(theta, d, a, phi):
    T1 = np.dot(rotation(theta, "z"),translate(d, "z"))
    T2 = np.dot(T1, translate(a, "x"))
    Ti = np.dot(T2, rotation(phi, "x"))

    return Ti


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

def IK(pose, DH_table):
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    joint_angles = np.zeros((6,1))

    # 0. DH Parameters
    d1 = DH_table[0,1]
    d4 = DH_table[3,1]
    # d6 = DH_table[5,1]
    d6 = 110

    a2 = DH_table[1,2]

    # 1. Find O0w(wrist position in world frame)  
    O0t = pose[3, 0:3]
    R0t = pose[0:3, 0:3]

    O0w = O0t - R0t*np.array([0, 0, d6]).T
    
    # 2. Find theta1, theta2, theta3
    # theta1 will have 2 set
    # theta2, theta3 will have 2 set

    X0w = O0w[0]
    Y0w = O0w[1]
    Z0w = O0w[2]
    
    theta1 = np.arctan2(Y0w, X0w)  

    Zd = Z0w - d1
    r = Y0w/np.sin(theta1)

    theta3 = np.arccos((Zd**2+r**2-a2**2-d4**2) / 2*a2*d4)
    theta2 = -pi/2 - np.arctan2(Zd,r) + np.arctan2(d4*sin(theta3), a2 + d4*cos(theta3))

    # 3. Find R0w 
    joint_angles[0] = theta1
    joint_angles[1] = theta2
    joint_angles[2] = theta3

    T04 = np.identity(4)

    for i in range(3):
        DH = DH_table[i]

        theta = np.radians(DH[0]) + joint_angles[i]
        phi = np.radians(DH[3])
        
        Ti = T_frrom_DH(theta, DH[1], DH[2], phi)
        
        T04 = np.dot(T04, Ti)

    R0w = T04[0:3,0:3]

    # 4. Find theta4, theta5, theta6
    Rwt = np.dot(R0w^-1, R0t)
    
    Twt = np.identity(4)
    Twt[0:3,0:3] = Rwt

    [joint_angles[3], joint_angles[4], joint_angles[5]] = get_euler_angles_from_T(Twt)

    return joint_angles


def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """
    R = T[0:3,0:3]

    # by lecture 3 p20
    r33 = R[2,2]
    
    if(r33 == 1 or r33 == -1):
        theta = 0
        phi = np.arctan2(R[1,0], R[0,0])
        psi = 0
    else:
        # ***** Need to choose positive or negative
        theta = np.arctan2(np.sqrt(1-r33**2), r33)
        # theta = np.arctan2(-np.sqrt(1-r33^2), r33)

        psi = np.arctan2(R[2,1],-R[2,0])
        phi = np.arctan2(R[1,2],R[0,2])

    return np.array([phi, theta, psi])/np.pi *180
    # pass

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