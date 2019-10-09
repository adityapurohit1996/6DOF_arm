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
    joint_angles = np.zeros(6)

    # 0. DH Parameters
    d1 = DH_table[0,1]
    d4 = DH_table[3,1]
    d6 = DH_table[5,1]
    # d6 = 0

    a2 = DH_table[1,2]

    # 1. Find O0w(wrist position in world frame)  
    O0t = pose[0:3, 3]
    R0t = pose[0:3, 0:3]

    O0w = O0t - np.dot(R0t,np.array([0, 0, d6]).T)
    
    # 2. Find theta1, theta2, theta3
    X0w = O0w[0]
    Y0w = O0w[1]
    Z0w = O0w[2]
    
    # theta1 could be +180
    theta1 = np.arctan2(Y0w, X0w) 
    theta1 = clamp_if_close_to_angle(theta1)
    theta1 = clamp_if_close_to_angle(theta1, 180) 

    Zd = Z0w - d1
    if(theta1 == 0):
        r = X0w/np.cos(theta1)
    else:
        r = Y0w/np.sin(theta1)    # theta1 might = 90

    # theta2, theta3 could be 2 set
    cos_theta3 = (Zd**2+r**2-a2**2-d4**2) / (2*a2*d4)

    if(cos_theta3 <= 1):
        REACHABLE = True

        theta3 = np.arccos(cos_theta3)
        theta2 = np.arctan2(r, Zd) - np.arctan2(d4*np.sin(theta3), a2+d4*np.cos(theta3))

        theta2 = clamp_if_close_to_angle(theta2)
        theta3 = clamp_if_close_to_angle(theta3)

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
        Rwt = np.dot(np.linalg.inv(R0w), R0t)
        
        Twt = np.identity(4)
        Twt[0:3,0:3] = Rwt

        [joint_angles[3], joint_angles[4], joint_angles[5]] = get_euler_angles_from_T(Twt)
    else:
        REACHABLE = False
        joint_angles[:] = np.nan

    return joint_angles, REACHABLE

def clamp_if_close_to_angle(theta, angle=0):
    angle_resolution = 1e-3

    if(abs(theta - angle) <= angle_resolution):
        theta = 0

    return theta

def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    /home/student/Desktop/G07/armlab-f19/state_machine.py
    """
    R = T[0:3,0:3]

    # by lecture 3 p20
    r33 = R[2,2]

    if(abs(r33-1)<=0.01 or abs(r33-1)<=0.01):
        theta = 0
        phi = np.arctan2(R[1,0], R[0,0])/2
        psi = phi
    else:
        # ***** Need to choose positive or negative
        theta = np.arctan2(np.sqrt(1-r33**2), r33)
        
        phi = np.arctan2(R[1,2]/np.sin(theta), R[0,2]/np.sin(theta))
        psi = np.arctan2(R[2,1]/np.sin(theta), -R[2,0]/np.sin(theta))
        ### WHY I NEED A - HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
        
        phi = phi%(2*np.pi)
        psi = psi%(2*np.pi)

        if(phi > np.pi):
            phi = phi -2*np.pi

        if(psi > np.pi):
            psi = psi -2*np.pi

        if(phi > 100*np.pi/180):
            # print("phi > 100!!! phi= ", phi)
            phi = phi - np.pi
            theta = -theta

            # print("phi -np.pi = ", phi)
            # print()
            # psi = -psi 
        elif(phi < -100*np.pi/180):
            # print("phi < -100!!!")
            phi = phi + np.pi
            theta = -theta
            # psi = -psi 
        
        # print("psi:  ", psi)
        if(psi > 100*np.pi/180):
            # print("psi > 100!!! psi= ", psi)
            psi = psi - np.pi
            # print("psi -np.pi = ", psi)
            # print()

        elif(psi < -100*np.pi/180):
            # print("psi > 100!!! psi= ", psi)
            psi = psi + np.pi
            # print("psi -np.pi = ", psi)
            # print()

    phi = clamp_if_close_to_angle(phi)
    theta = clamp_if_close_to_angle(theta)
    psi = clamp_if_close_to_angle(psi)

    return np.array([phi, theta, psi])


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
    if axis == "z":
        R = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta),0], [0, 0, 1]])
    if axis == "x":
        R = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
    if axis == "y":
        R = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

    RD = np.concatenate((R,D), axis=1)

    T = np.concatenate((RD,[[0,0,0,1]]), axis=0)
    return T


"""main function for testing"""
import rexarm

def test():
    # DH = [theta, di, ai, alpha]
    DH_table = np.array([[0, 74.76+40.64, 0, -90],
                        [-90, 0, 99.58, 0],
                        [90, 0, 0, 90],
                        [0, 110.46, 0, -90],
                        [0, 0, 0, 90],
                        [0, 130.62, 0, 0]])

    DH_FK = np.array([[0, 74.76+40.64, 0, -90],
                        [-90, 0, 99.58, 0],
                        [90, 0, 0, 90],
                        [0, 110.46, 0, -90],
                        [0, 0, 0, 90],
                        [0, 150, 0, 0]])
    
    th1 = 90
    th2 = 90
    th3 = 0
    th4 = 90
    th5 = 0
    th6 = 0
    thetas = np.array([th1, th2, th3, th4, th5, th6])
    joint_angles = np.radians(thetas)
    
    Rexarm = rexarm.Rexarm([0],0 )

    pose = [-200, 200, 50, [np.pi/4 + np.pi/2, np.pi*3/4, 0]]
    grap_pose, prep_pose, isTOP = Rexarm.check_fesible_IK(pose, 20, False,"ARB")
    print("grap_pose", grap_pose)
    print("prep_pose", prep_pose)

    Rexarm.interpolating_in_WS(grap_pose[0:3,0:3], np.array([-200,200,20]), np.array([0,200,20]), 10)

    print("\n\nChange Grab POSE!!!\n\n")

    pose = [0, 200, 50, [np.pi/4, np.pi*3/4, 0]]
    grap_pose, prep_pose, isTOP = Rexarm.check_fesible_IK(pose, 20, False,"ARB")
    # print grap_pose
    Rexarm.interpolating_in_WS(grap_pose[0:3,0:3], np.array([0,200,20]), np.array([200,200,20]), 10)


    # T_FK = FK_dh(joint_angles, DH_FK)
    # T_FK = FK_dh(joint_angles, DH_table)
    
    # IK_angle, REACHABLE = IK(T_FK, DH_table)
 
    # # print("T_FK: ", T_FK)
    # print("REACHABLE: ", REACHABLE)
    # print("IK_angle: ", IK_angle)

    # if REACHABLE is True:
    #     T_IK = FK_dh(IK_angle, DH_table)


    #     print("IK: ", np.rad2deg(IK_angle))
    #     print("Difference of Joint angels: ", thetas - np.rad2deg(IK_angle[0:6]))
    #     # print("T from FK: ", T_FK)
    #     print("T from FK - IK: ", T_FK - T_IK)
    #     print("Distance of End-effecter: ", T_FK[0:3,3] - T_IK[0:3,3])
    #     print("Difference of Tool angles: ", (get_euler_angles_from_T(T_FK) - get_euler_angles_from_T(T_IK))*180/np.pi)
    # else:
    #     print("GG")

 
if __name__ == '__main__':
    test()