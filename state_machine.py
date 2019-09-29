import time
import cv2
import numpy as np
from kinematics import *

waypoints_recorded = []
"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.projection = np.identity(3)

        self.IK_target = self.rexarm.get_positions

    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "execute"):
                self.execute()
            if(self.next_state == "record"):
                self.record()
            if(self.next_state == "playback"):
                self.playback()
            if(self.next_state == "TP test"):
                self.TP_test()
            if(self.next_state == "IK_set_pose"):
                self.IK_set_pose()
            if(self.next_state == "IK_test"):
                self.IK_test()

                
        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()
        
        if(self.current_state == "execute"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "record"):
            if(self.next_state == "idle"):
                self.idle()  
        if(self.current_state == "playback"):
            if(self.next_state == "idle"):
                self.idle()   

        if(self.current_state == "TP test"):
            if(self.next_state == "idle"):
                self.idle()
        
        if(self.current_state == "IK_set_pose"):
            if(self.next_state == "idle"):
                self.idle()
        if(self.current_state == "IK_test"):
            if(self.next_state == "idle"):
                self.idle()
                
               

    """Functions run for each state"""
    def IK_set_pose(self):
        self.status_message = "State: IK_set_pose - setting pose for IK test"
        self.current_state = "IK_set_pose"

        self.IK_target = list(self.rexarm.get_positions())
        print("Setting target: ", self.IK_target)
        self.set_next_state("idle")

    def IK_test(self):
        self.status_message = "State: IK_test - Going to desinated pose"
        self.current_state = "IK_test"

        print("IK_target: ", self.IK_target)
        IK_pose = FK_dh(self.IK_target, self.rexarm.DH_table)
        print("IK_Pose: ", IK_pose)

        self.rexarm.set_pose(IK_pose)
        self.rexarm.pause(3)

        self.set_next_state("idle")
        self.rexarm.get_feedback()


    def TP_test(self):    
        self.status_message = "State: TP_test - testing trajectory planner"
        self.current_state = "TP_test"

        waypoints = np.array([[ 0.0, 0.0, 0.0, 0.0, 0.0],
                    [ 1.0, 0.8, 1.0, 0.5, 1.0],
                    [-1.0,-0.8,-1.0,-0.5, -1.0],
                    [-1.0, 0.8, 1.0, 0.5, 1.0],
                    [1.0, -0.8,-1.0,-0.5, -1.0],
                    [ 0.0, 0.0, 0.0, 0.0, 0.0]])
        
        max_speed = 1  # in radius/s

        # self.rexarm.set_speeds_normalized_global(max_speed/12.2595,update_now=True)

        for i in range(len(waypoints) - 1):
            self.tp.set_initial_wp(waypoints[i])
            self.tp.set_final_wp(waypoints[i+1])
            T = self.tp.calc_time_from_waypoints(max_speed)
           # print(T)
            plan = self.tp.generate_cubic_spline(T)

            # print(plan[0][-1])

            self.tp.execute_plan(plan, 10)
        
        self.rexarm.set_speeds_normalized_global()
        self.set_next_state("idle")

    def execute(self):
        self.status_message = "State: execute - executing predifined waypoints with trivial set_positions"
        self.current_state = "execute"
        
        print(self.rexarm.waypoints_recorded)
        #wtr = csv.writer(open ('C.csv', 'w'), delimiter=',', lineterset_next_stateminator='\n')
        #for x in self.rexarm.waypoints_recorded
        self.rexarm.set_speeds_normalized_global(0.2,update_now=True)
        self.status_message = "State: execute - task 1.2"
        self.current_state = "execute"
       
        waypoints = np.array([[ 0.0, 0.0, 0.0, 0.0, 0.0],
                    [ 1.0, 0.8, 1.0, 0.5, 1.0],
                    [-1.0,-0.8,-1.0,-0.5, -1.0],
                    [-1.0, 0.8, 1.0, 0.5, 1.0],
                    [1.0, -0.8,-1.0,-0.5, -1.0],
                    [ 0.0, 0.0, 0.0, 0.0, 0.0]])
        """            
        waypoints = self.rexarm.waypoints_recorded
        """
        for point in waypoints:
                    print(point)
                    self.rexarm.set_positions(point)
                    self.rexarm.pause(3)

        # self.rexarm.waypoints_recorded = []
        # self.rexarm.set_torque_limits([0/100.0]*self.rexarm.num_joints,update_now = True)
        self.set_next_state("idle")
        self.rexarm.get_feedback()

    def record(self):
        self.status_message = "State: record - recording position"
        self.current_state = "record"

        print("Made it")
        print(self.rexarm.get_positions())
        self.rexarm.waypoints_recorded.append(list(self.rexarm.get_positions()))
        #self.rexarm.waypoints_recorded = np.array(self.rexarm.waypoints_recorded)
        print(self.rexarm.waypoints_recorded)
        self.set_next_state("idle")
    
    def playback(self):
        self.status_message = "State: playback - playing recroded waypoints"
        self.current_state = "playback"
        self.rexarm.set_torque_limits([40/100.0]*self.rexarm.num_joints,update_now = True)
        self.execute()
        self.set_next_state("idle")


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        
    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.tp.go(max_speed=2.0)
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "corner of the tape"]
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
  
        Zc = 939
        b = np.transpose([-303.2,-303.2,303.2,-303.2,303.2,303.2,-303.2,303.2,179.38-303.2,174-303.2])
        World_points = np.array([[-303.2,-303.2,Zc],[-303.2,303.2,Zc],[303.2,303.2,Zc],[303.2,-303.2,Zc],[174.38-303.2,176-303.2,Zc]],dtype= np.float64)
        Camera_coord =np.array([])
        #intrinsic_matrix = self.kinect.loadCameraCalibration()

        #intrinsic matrix from camera_cal.py
        intrinsic_matrix = np.array([[ 526.17413472 ,   0   ,       325.10510152],
                            [   0      ,    526.04886194,  276.03701925],
                            [   0 ,           0     ,       1        ]])

        inv_intrinsic_matrix = np.linalg.inv(intrinsic_matrix)
        

        #get the affine transform from depth image to affine image
        self.kinect.depth2rgb_affine = self.kinect.getAffineTransform(self.kinect.depth_click_points,self.kinect.rgb_click_points)

        # Generate Camera coordinates from image coordinates
        for i,rgb in enumerate (self.kinect.rgb_click_points) :
            a = np.array([rgb[0],rgb[1],1])
            a = np.transpose(a)
            coord =Zc* np.dot(inv_intrinsic_matrix ,a) 
            coord =[coord]
            if i == 0:
                Camera_coord = coord
            else :
                Camera_coord = np.concatenate ((Camera_coord,coord),axis = 0)

        # Find the extrinsic matrix for relation between camera coordinates and world coordinates

        extrinsic_affine = self.kinect.getAffineTransform3(Camera_coord,np.squeeze(World_points))

        # Find the projection matrix between image and world coordinates
        self.projection =  np.dot(extrinsic_affine, inv_intrinsic_matrix)
       
        self.kinect.kinectCalibrated = True
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)