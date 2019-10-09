#rexarm.py
import numpy as np
from kinematics import *
import time

""" 
TODO:

Implement the missing functions
add anything you see fit

"""
""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592
#gripper close -0.9599 
#gripper open 0.5759
class Rexarm():
    def __init__(self, joints, gripper):
        """Recorded"""
        self.waypoints_recorded = []
        self.joints = joints
        self.gripper = gripper
        self.gripper_open_pos = -0.3
        self.gripper_closed_pos = self.gripper_open_pos + np.pi/2
        self.gripper_state = True
        self.estop = False
        self.Z0_offset = 10
        self.cube_height = 40 #mm
        self.prep_height = 60 #mm


        """TODO: Find the physical angle limits of the Rexarm. Remember to keep track of this if you include more motors"""
        safety_margim = 5
        self.angle_limits = np.array([
                            [-180+safety_margim, 179.99-safety_margim],
                            [-25-90+safety_margim, 208-90-safety_margim],
                            [-110+safety_margim, 100-safety_margim],
                            [-150+safety_margim, 150-safety_margim],
                            [-100+safety_margim, 100-safety_margim],
                            [-150+safety_margim, 150-safety_margim]], dtype=np.float)*D2R

        """ Commanded Values """
        self.num_joints = len(joints)
        self.position = [0.0] * self.num_joints     # degrees
        self.speed = [1.0] * self.num_joints        # 0 to 1
        self.max_torque = [1.0] * self.num_joints   # 0 to 1
        self.gripper_position = 0.0

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # degrees
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius
        self.move_fb = [0] *  self.num_joints

        # DH = [theta, di, ai, alpha]
        self.DH_table = np.array([[0, 117.52, 0, -90],
                        [-90, 0, 99.58, 0],
                        [90, 0, 0, 90],
                        [0, 110.46, 0, -90],
                        [0, 0, 0, 90],
                        [0, 130.66, 0, 0]])

    def initialize(self):
        for joint in self.joints:
            joint.enable_torque()
            joint.set_position(0.0)
            joint.set_torque_limit(0.5)
            joint.set_speed(0.25)
        if(self.gripper != 0):
            print("initialize!!")
            self.gripper.enable_torque()
            self.gripper.set_position(self.gripper_open_pos)
            self.gripper.set_torque_limit(1.0)
            self.gripper.set_speed(0.8)
            #self.close_gripper()

    def open_gripper(self):
        """ TODO """
        self.gripper_state = False
        self.gripper.set_position(self.gripper_open_pos)
        pass

    def close_gripper(self):
        """ TODO """
        self.gripper_state = True
        self.gripper.set_position(self.gripper_closed_pos)
        pass

    def toggle_gripper(self):
        """ TODO """
        if(self.gripper_state is True):
            self.gripper_state = False
            self.gripper.set_position(self.gripper_open_pos)
        else:
            self.gripper_state = True
            self.gripper.set_position(self.gripper_closed_pos)
        pass

    def set_positions(self, joint_angles, update_now = True):
        self.clamp(joint_angles)
        # print(joint_angles)
        for i,joint in enumerate(self.joints):
            #print(i)
            #print(joint)
            #print(joint_angles[i])
            self.position[i] = joint_angles[i]
            if(update_now):
                joint.set_position(joint_angles[i])

    def interpolating_in_WS(self, orientation, pi, pf, density):
        '''
        density: mm/point
        '''
        num = max(abs(pf-pi))/density
        
        step = (1.0/num)*(pf-pi)
        path = step[:,None]*np.arange(num) + pi[:,None]
        path = np.transpose(path)

        T = np.identity(4)
        T[0:3,0:3] = orientation

        for position in path:
            print(position)
            T[0:3,3] = position
            print(T)
            self.set_pose(T)
            self.pause(0.2) 

    def check_fesible_IK(self, pose_of_block, delta_Z, isGrab, desire_mode = "GRAB_FROM_TOP"):
        X, Y, Z, angles = pose_of_block
        theta = angles[0] # only use first z in zyz (right now)

        T_grab = np.identity(4)
        isTOP = False

        if(desire_mode == "GRAB_FROM_TOP"):
            T_grab[0:3,0:3] = np.array([[-np.cos(theta), -np.sin(theta), 0],
                                    [np.sin(theta), -np.cos(theta), 0],
                                    [0, 0, -1]])
            
            # checking if can grab from top
            T_grab[0,3] = X
            T_grab[1,3] = Y

            if(isGrab):
                T_grab[2,3] = Z - 40 # cube height
            else:
                T_grab[2,3] = Z
            _, REACHABLE_grab = IK(T_grab, self.DH_table)
            
            T_prep = np.copy(T_grab)
            T_prep[2,3] = Z+delta_Z
            _, REACHABLE_above = IK(T_prep, self.DH_table)

            print("From TOP:")
            print("grab: ", REACHABLE_grab, "  prep: ", REACHABLE_above)

            if REACHABLE_grab and REACHABLE_above:
                grap_pose = T_grab
                prep_pose = T_prep
                isTOP = True
            print("isTOP: ", isTOP)

            if(not isTOP):
                if(X >= 0):
                    if(Y >= 0):
                        theta = theta
                    else:
                        theta = theta - np.pi/2
                else:
                    if(Y >= 0):
                        theta = theta + np.pi/2
                    else:
                        theta = theta + np.pi

                T_grab = np.dot(rotation(theta, "z"), rotation(np.pi/2, "y"))
                T_grab[0,3] = X
                T_grab[1,3] = Y
                T_grab[2,3] = Z -20 # Cube is about 40mm
                _, REACHABLE_grab = IK(T_grab, self.DH_table)

                T_prep = np.copy(T_grab)
                T_prep[0,3] = X - delta_Z*np.cos(theta)
                T_prep[1,3] = Y - delta_Z*np.sin(theta)
                _, REACHABLE_side = IK(T_prep, self.DH_table)

                print("From Horizon:")
                print("grab: ", REACHABLE_grab, "  prep: ", REACHABLE_side)
                    
                if REACHABLE_grab and REACHABLE_side:
                    grap_pose = T_grab
                    prep_pose = T_prep
                else:
                    T_grab[:] = np.nan
                    T_prep[:] = np.nan
                    grap_pose = T_grab
                    prep_pose = T_prep

        elif(desire_mode == "ARB"):

            T_grab = np.dot(np.dot(rotation(angles[0], "z"), rotation(angles[1], "y")),rotation(angles[2],"z"))
            T_grab[0,3] = X
            T_grab[1,3] = Y
            T_grab[2,3] = Z

            _, REACHABLE_grab = IK(T_grab, self.DH_table)

            T_prep = np.copy(T_grab)
            T_prep[0,3] = X - delta_Z*np.cos(angles[0])*np.cos(angles[1])
            T_prep[1,3] = Y - delta_Z*np.sin(angles[0])*np.cos(angles[1])
            T_prep[2,3] = Z - delta_Z*np.sin(angles[1])
            _, REACHABLE_side = IK(T_prep, self.DH_table)

            if REACHABLE_grab and REACHABLE_side:
                grap_pose = T_grab
                prep_pose = T_prep
            else:
                T_grab[:] = np.nan
                T_prep[:] = np.nan
                grap_pose = T_grab
                prep_pose = T_prep

        return grap_pose, prep_pose, isTOP

    def grab_or_place_block(self, pose_of_block, offset, isGrab, desire_mode = "GRAB_FROM_TOP"):
        '''
        pose_of_block: X, Y, Z, theta(z-axis)
        '''
        T_grab, T_prep, isTOP = self.check_fesible_IK(pose_of_block,offset, isGrab)
        
        print("Grab", T_grab)
        print("Prep", T_prep)
        print("isTOP", isTOP)

        if(isTOP):
            self.set_pose(T_prep)
            self.pause(3)
            self.set_pose(T_grab)
            self.pause(3)
            self.toggle_gripper()
            self.pause(1)
            self.set_pose(T_prep)
            self.pause(3)
        elif(T_grab[0,0] != np.nan):
            # grapping from sideway, can still edit pose
            self.set_pose(T_prep)
            self.pause(3)
            self.set_pose(T_grab)
            self.pause(3)
            self.toggle_gripper()
            self.pause(1)
            self.set_pose(T_prep)
            self.pause(3)
        else:
            print("Sorry, I can't do this move.")

        print("DONE this pose!!")

    def set_gripper_position(self, gripper_position, update_now = True):
        self.gripper_position = gripper_position
        if(update_now):
            self.gripper.set_position(gripper_position)

    def set_pose(self, pose, update_now = True):
        joint_angles, isGOOD = IK(pose, self.DH_table)
        print("Joint angles from IK: ", joint_angles)
        # self.set_positions(joint_angles, update_now)
        if(isGOOD):
            self.set_positions(joint_angles[0:6], update_now)
        else:
            print("This doesn't achievable!!!")
            print(pose)
            print("=============")

    def set_speeds_normalized_global(self, speed, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speed
            if(update_now):
                joint.set_speed(speed)

    def set_speeds_normalized(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            if(update_now):
                joint.set_speed(speeds[i])

    def set_speeds(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            speed_msg = abs(speeds[i]/joint.max_speed)
            if (speed_msg < 3.0/1023.0):
                speed_msg = 3.0/1023.0
            if(update_now):
                joint.set_speed(speed_msg)
    
    def set_torque_limits(self, torques, update_now = True):
        for i,joint in enumerate(self.joints):
            self.max_torque[i] = torques[i]
            if(update_now):
                joint.set_torque_limit(torques[i])

    def send_commands(self):
        self.set_positions(self.position)
        self.set_speeds_normalized(self.speed)
        self.set_torque_limits(self.max_torque)
        self.gripper.set_position(self.gripper_position)

    def enable_torque(self):
        for joint in self.joints:
            joint.enable_torque()

    def disable_torque(self):
        for joint in self.joints:
            joint.disable_torque()

    def get_positions(self):
        for i,joint in enumerate(self.joints):
            self.joint_angles_fb[i] = joint.get_position()
        return self.joint_angles_fb

    def get_speeds(self):
        for i,joint in enumerate(self.joints):
            self.speed_fb[i] = joint.get_speed()
        return self.speed_fb

    def get_loads(self):
        for i,joint in enumerate(self.joints):
            self.load_fb[i] = joint.get_load()
        return self.load_fb

    def get_temps(self):
        for i,joint in enumerate(self.joints):
            self.temp_fb[i] = joint.get_temp()
        return self.temp_fb

    def get_moving_status(self):
        for i,joint in enumerate(self.joints):
            self.move_fb[i] = joint.is_moving()
        return self.move_fb

    def get_feedback(self):
        self.get_positions()
        self.get_speeds()
        self.get_loads()
        self.get_temps()
        self.get_moving_status()

    def pause(self, secs):
        time_start = time.time()
        while((time.time()-time_start) < secs):
            self.get_feedback()
            time.sleep(0.05)
            if(self.estop == True):
                break

    def clamp(self, joint_angles):
        for i, joint in enumerate(joint_angles):
            if joint > self.angle_limits[i][1]:
                joint_angles[i] = self.angle_limits[i][1]
            if joint < self.angle_limits[i][0]:
                joint_angles[i] = self.angle_limits[i][0]

        


    def get_wrist_pose(self):
        """TODO"""
        T = FK_dh(self.joint_angles_fb, self.DH_table)

        R = get_euler_angles_from_T(T)/np.pi*180
        D = np.dot(T,np.transpose([0, 0, 0, 1]))

        # print(D)

        return [D[0],D[1],D[2],R[0], R[1], R[2]]
