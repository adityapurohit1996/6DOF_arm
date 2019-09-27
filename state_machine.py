import time
import cv2
import numpy as np

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
                
        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()
        
        if(self.current_state == "execute"):
            self.idle()

        if(self.current_state == "playback"):
            if(self.next_state == "idle"):
                self.idle()   
                
               

    """Functions run for each state"""
    def execute(self):
        print(self.rexarm.waypoints_recorded)
        #wtr = csv.writer(open ('C.csv', 'w'), delimiter=',', lineterminator='\n')
        #for x in self.rexarm.waypoints_recorded
        self.rexarm.set_speeds_normalized_global(0.1,update_now=True)
        self.status_message = "State: execute - task 1.2"
        self.current_state = "execute"
        """
        waypoints = np.array([[ 0.0, 0.0, 0.0, 0.0, 0.0],
                    [ 1.0, 0.8, 1.0, 0.5, 1.0],
                    [-1.0,-0.8,-1.0,-0.5, -1.0],
                    [-1.0, 0.8, 1.0, 0.5, 1.0],
                    [1.0, -0.8,-1.0,-0.5, -1.0],
                    [playback 0.0, 0.0, 0.0, 0.0, 0.0]])
        """
        waypoints = self.rexarm.waypoints_recorded
        for point in waypoints:
            print(point)
            self.rexarm.set_positions(point)
            self.rexarm.pause(3)
        self.rexarm.waypoints_recorded = []
        self.rexarm.set_torque_limits([0/100.0]*self.rexarm.num_joints,update_now = True)
        self.set_next_state("idle")

    def record(self):
        print("Made it")
        print(self.rexarm.get_positions())
        self.rexarm.waypoints_recorded.append(list(self.rexarm.get_positions()))
        #self.rexarm.waypoints_recorded = np.array(self.rexarm.waypoints_recorded)
        print(self.rexarm.waypoints_recorded)
        self.set_next_state("idle")
    
    def playback(self):
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
   
        #print(self.kinect.rgb_click_points)
        #print(self.kinect.depth_click_points)
        b = np.transpose([0,0,606.4,0,606.4,606.4,0,606.4,179.38,174])
        World_points = np.array([[0,0,914],[606.4,0,914],[606.4,606.4,914],[0,606.4,914],[179.38,174,914]],dtype= np.float64)
        Camera_coord =np.array([])
        #intrinsic_matrix = self.kinect.loadCameraCalibration()
        intrinsic_matrix = np.array([[ 526.17413472 ,   0   ,       325.10510152],
                            [   0      ,    526.04886194,  276.03701925],
                            [   0 ,           0     ,       1        ]])
        inv_intrinsic_matrix = np.linalg.inv(intrinsic_matrix)
        print(inv_intrinsic_matrix)
        self.kinect.depth2rgb_affine = self.kinect.getAffineTransform(self.kinect.depth_click_points,self.kinect.rgb_click_points)
        print(self.kinect.depth2rgb_affine)
        self.kinect.depth2rgb_affine = cv2.findHomography(self.kinect.depth_click_points,self.kinect.rgb_click_points)
        print(type(self.kinect.depth2rgb_affine))
        for i,rgb in enumerate (self.kinect.rgb_click_points) :
            a = np.array([rgb[0],rgb[1],1])
            a = np.transpose(a)
            Zc = 914
            #Zc = 1
            coord =Zc* np.dot(inv_intrinsic_matrix ,a) 
            #coord = a
            coord =[coord]
            #print(coord)
            if i == 0:
                Camera_coord = coord
            else :
                Camera_coord = np.concatenate ((Camera_coord,coord),axis = 0)
        print("camera_coord")
        print(Camera_coord)

        extrinsic_affine = self.kinect.getAffineTransform3(Camera_coord,np.squeeze(World_points))
        print(extrinsic_affine)
        self.projection = Zc * np.dot(extrinsic_affine, np.linalg.inv(intrinsic_matrix))
        """
        dist_Coeffs = np.array([ 0.30001958 ,-0.82118258 ,-0.00396884 ,-0.01034744 , 0.23477272])
        (success,rot_vec,trans_vec) = cv2.solvePnP (World_points,np.array(self.kinect.rgb_click_points,dtype=np.float32),intrinsic_matrix,dist_Coeffs)
        print(rot_vec)
        print(trans_vec)
        rot_mat = np.diag(np.squeeze(rot_vec))
        rot_mat = np.delete(rot_mat,2,1)
        extrinsic = np.append(rot_mat,trans_vec,1)
        print(extrinsic)
        I = np.identity(3)
        O = np.transpose([[0,0,0]])
        print(I.shape)
        print(O.shape)
        proj_mat = np.append(I,O,1)
        print
        fin_projection =np.dot(intrinsic_matrix,extrinsic)/Zc
        print(fin_projection)
        self.projection =np.linalg.inv(fin_projection)
        #self.projection = extrinsic_affine
       """
        print("projection")
        print(self.projection)
        self.kinect.kinectCalibrated = True
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)