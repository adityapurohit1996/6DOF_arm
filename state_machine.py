import time
import cv2
import numpy as np
from kinematics import *
import scipy.io as sio

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
        self.Z0_offset = 13 #mm
        self.stack_step = 40 #mm
        self.z_offset = 60 #mm
        

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
            if(self.next_state == "Detect Blocks") :
                self.BlockDetection()
            if(self.next_state == "Grab_Place"):
                theta = np.array([[0, 0, 0],[0, 0, 0]])
                theta = np.deg2rad(theta)
                world_frame = self.clickCoordnates(2)
                z = self.Z0_offset #mm 
                self.Grab_Place(world_frame,theta,z)
            if(self.next_state == "BlockSlider"):
                self.BlockSlider(1)
            if(self.next_state == "Pick_N_Stack"):
                self.Pick_N_Stack(3)

                
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
        if(self.current_state == "Detect Blocks"):
            if(self.next_state == "idle"):
                self.idle()
        
        if(self.current_state == "Grab_Place"):
            if(self.next_state == "idle"):
                self.idle()
        
        if(self.current_state == "BlockSlider"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "Pick_N_Stack"):       
            if(self.next_state == "idle"):
                self.idle()
        
                
               

    """Functions run for each state"""
    

    def constructPose(self,world_frame,theta,z):
        world_frame[2] += z
        #homogeneous = np.transpose(np.ones((1,1)))
        #coordinates_global = np.concatenate((world_frame,homogeneous),axis = 1)
        coordinates_global = np.append(world_frame,1)
        orientation_gripper = np.zeros((4,4))
        pose = np.zeros((4,4))
        orientation_gripper = np.dot(np.dot(-1*rotation(theta[0],'z'),rotation(theta[1],'y')),rotation(theta[2],'z'))
        pose[0:4,0:4]=orientation_gripper
        pose[:,3] = np.transpose(coordinates_global)
        return pose
    
    def clickCoordnates(self,numPoints):
        world_frame = np.zeros((numPoints,3))
        cube_click_points = np.zeros((numPoints,2),int)
        #Get the World Coordinates of the pick up and drop off from mouse clicks============================
        i = 0
        for j in range(numPoints):
            self.status_message = "Click the current location of cube and location the cube should be placed"
            while (i <= j):
                #self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    cube_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False  
                x = self.kinect.last_click[0]
                y = self.kinect.last_click[1] 
                world_frame[j] = self.kinect.rgb2world(x,y)     
        i = 0
        #=====================================================================================================
        print("World Frame Clicked",world_frame)
        # world_frame[0:2,2] += z_offset
        homogeneous = np.transpose(np.ones((1,2)))
        coordinates_global = np.concatenate((world_frame,homogeneous),axis = 1)
        return world_frame  

    def BlockSlider(self,stack):
        self.status_message = "State: BlockSlider - Completes Event in Competition"
        self.current_state = "Grab_Place"
        self.rexarm.set_speeds_normalized_global(0.1,update_now=True)
        self.rexarm.open_gripper()
        
        #Motion Planning
        Slid_Positions = np.array([[-200, 200, 20], [0, 200, 20], 
                                   [200, 200, 20], [200, 0, 20],
                                   [200, -200, 20], [0, -200, 20],
                                   [-200, 200, 20], [-200, 0, 20],
                                   [-200,200,20]])
        
        Slid_Orientations = np.array([[np.pi*3/4, np.pi*3/4,0], [np.pi/4, np.pi*3/4,0],
                                      [np.pi/4, np.pi*3/4, 0], [-np.pi/4, np.pi*3/4, 0],
                                      [-np.pi/4, np.pi*3/4, 0], [-np.pi*3/4, np.pi*3/4, 0],
                                      [-np.pi*3/4, np.pi*3/4, 0], [np.pi*3/4, np.pi*3/4, 0] ])

        
        for i in range(len(Slid_Positions)-1):
            pose = Slid_Positions[i].append(Slid_Orientations[i])
            grap_pose, prep_pose, isTOP = self.rexarm.check_fesible_IK(pose, 20, False, "ARB")
            print("grap_pose",grap_pose[0,0])

            self.rexarm.set_pose(prep_pose)
            self.rexarm.pause(2)
            self.rexarm.set_pose(grap_pose)
            self.rexarm.pause(1)
            self.rexarm.toggle_gripper()
       
            self.rexarm.interpolating_in_WS(grap_pose[0:3,0:3], Slid_Positions[i], Slid_Positions[i+1], 10)
            self.rexarm.toggle_gripper()
            
            self.rexarm.joints[1].set_position(0)
            self.rexarm.joints[2].set_position(0)


        #Finished
        self.set_next_state("idle")
        self.rexarm.get_feedback()
    
    def Pick_N_Stack(self,numBlocks):
        self.status_message = "State: Pick_N_Stack - Stacks cubes in a specified location"
        self.current_state = "Pick_N_Stack"
        self.rexarm.set_speeds_normalized_global(0.1,update_now=True)
        self.rexarm.open_gripper()
        #Motion Planning
        world_frame = self.clickCoordnates(3)
        stack_location = np.array([-100,-100,self.Z0_offset])
        size = np.size(world_frame,0)
        '''
        new_plan = zeros((size*2,3))
        j=1
        for i in range(size)
            new_plan[i] = world_frame[i]
            new_plan[j] = stack_location
            j+=2
        world_frame = new_plan  
        '''
        theta = np.array([0,0,0])
        height_stack = self.Z0_offset
        for i in range(size):
            '''60
            print(height_stack)
            print(world_frame)
            print(world_frame[i])
            print(stack_location)
            print(np.stack((world_frame[i],stack_location)))
            '''
            self.Grab_Place(np.stack((world_frame[i],stack_location)), theta, height_stack)
            height_stack = height_stack+self.stack_step
        #Finished
        for joint in self.rexarm.joints:
            joint.set_position(0.0)
        self.set_next_state("idle")
        self.rexarm.get_feedback()
        

    def Grab_Place(self,world_frame, theta, z):
        self.status_message = "State: Grab_Place - Grabbing a Cube at one global coordinate and placing the cube in another"
        self.current_state = "Grab_Place"
        self.rexarm.set_speeds_normalized_global(0.1,update_now=True)
        self.rexarm.open_gripper()
        
        #Motion Planning
        size = np.size(world_frame,0)
        for i in range(size):
            
            pose_of_block = [world_frame[i][0],world_frame[i][1],world_frame[i][2], theta[i]]
            print("Point #", i)
            print(pose_of_block)
            
            if(i%2 == 1):
                isGrab = True
            else:
                isGrab = False

            self.rexarm.grab_or_place_block(pose_of_block, 40, isGrab)
            '''
            
            pose = self.constructPose(world_frame[i],np.array([0,0,0]),self.z_offset)
            print(pose)
            self.rexarm.set_pose(pose)
            self.rexarm.pause(4)
            if (i % 2) == 0 | i==0:
                pose[2][3] = self.Z0_offset
            else:
                pose[2][3] = z
            #print(pose)
            self.rexarm.set_pose(pose)
            self.rexarm.pause(3)
            if (i % 2) == 0 | i==0:
                self.rexarm.close_gripper()
            else: 
                print("Made it")
                self.rexarm.open_gripper()

            self.rexarm.pause(1)
            
        #Slowly move gripper above cube and then back to zero joint positions
        pose[2][3] = self.z_offset
        self.rexarm.set_speeds_normalized_global(0.05,update_now=True)
        # self.rexarm.set_pose(pose)
        self.rexarm.pause(2)
        self.rexarm.set_speeds_normalized_global(0.1,update_now=True)
        '''

        #Finished
        for joint in self.rexarm.joints:
            joint.set_position(0.0)
        self.set_next_state("idle")
        self.rexarm.get_feedback()
        

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

        waypoints_def = np.array([[ 0.0, 0.0, 0.0, 0.0, 0.0],
                    [ 1.0, 0.8, 1.0, 0.5, 1.0],
                    [-1.0,-0.8,-1.0,-0.5, -1.0],
                    [-1.0, 0.8, 1.0, 0.5, 1.0],
                    [1.0, -0.8,-1.0,-0.5, -1.0],
                    [ 0.0, 0.0, 0.0, 0.0, 0.0]])
        
        waypoints_hori = np.array([[ 0.0, 0.0, 0.0, 0.0, 0.0],
                    [ 1.0, 0.8, 0.0, 0.0, 0.0],
                    [ -1.0, 0.8, 0.0, 0.0, 0.0],
                    [ 0.0, 0.0, 0.0, 0.0, 0.0]])
        
        waypoints = waypoints_def

        # in radius/s
        max_speed = 1  # in radius/s
        # max_speed = 5

        # self.rexarm.set_speeds_normalized_global(max_speed/12.2595,update_now=True)

        for i in range(len(waypoints) - 1):
            self.tp.set_initial_wp(waypoints[i])
            self.tp.set_final_wp(waypoints[i+1])
            T = self.tp.calc_time_from_waypoints(max_speed)
           # print(T)
            
            if i == 1:
                plan = self.tp.generate_cubic_spline(T)
                Record_qd = plan[0]
                Record_vd = plan[1]
            else:
                plan = self.tp.generate_cubic_spline(T)
                Record_qd.append(plan[0])
                Record_vd.append(plan[1])

            if i == 1:
                Record_joints = self.tp.execute_plan(plan, 10)
            else:
                Record_joints.append(self.tp.execute_plan(plan, 10))

        Record_dict = {}
        Record_dict['qd'] = Record_qd
        Record_dict['vd'] = Record_vd
        Record_dict['joints'] = Record_joints
        sio.savemat('..\TP_Record.mat', Record_dict)
        
        self.rexarm.set_speeds_normalized_global(0.25)
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

        self.kinect.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])

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
        World_points = np.array([[-303.2,-303.2,Zc],[-303.2,303.2,Zc],[303.2,303.2,Zc],[303.2,-303.2,Zc],[179-303.2,174-303.2,Zc]],dtype= np.float64)
        Camera_coord =np.array([])
        #intrinsic_matrix = self.kinect.loadCameraCalibration()

        #intrinsic matrix from camera_cal.py
        intrinsic_matrix = np.array([[ 526.17413472 ,   0   ,       325.10510152],
                            [   0      ,    526.04886194,  276.03701925],
                            [   0 ,           0     ,       1        ]])

        inv_intrinsic_matrix = np.linalg.inv(intrinsic_matrix)
        

        #get the affine transform from depth image to affine image
        self.kinect.depth2rgb_affine = self.kinect.getAffineTransform(self.kinect.depth_click_points,self.kinect.rgb_click_points)
        np.savetxt("depth2rgb_affine.cfg", self.kinect.depth2rgb_affine)
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
        self.kinect.projection =  np.dot(extrinsic_affine, inv_intrinsic_matrix)
        np.savetxt("projection.cfg", self.kinect.projection)

        self.kinect.kinectCalibrated = True
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)

    def BlockDetection(self) :
        self.kinect.loadDepthFrame()
        print("Loaded depth frame")
        x,y = self.kinect.detectBlocksInDepthImage()
        print(x,y)
