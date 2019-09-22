import time
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
   
        print(self.kinect.rgb_click_points)
        print(self.kinect.depth_click_points)

        """TODO Perform camera calibration here"""
        A =np.array([])
        matrix_affine = np.array([])
        ma_vect =[]
        print(self.kinect.rgb_click_points[0])
        for i, rgb in enumerate(self.kinect.rgb_click_points) :
            a = np.array([[rgb[0],rgb[1],1,0,0,0],[0,0,0,rgb[0],rgb[1],1]])
            if(i==0) :
                A =a
            else :
                A = np.concatenate((A,a),axis = 0)
      #  print(A)
        b = np.transpose([0,0,0,606.4,606.4,606.4,606.4,0,179.38,174])
      #  print(np.shape(A))
       # print(np.shape(b))
        ma_vect = np.dot(np.dot(np.linalg.inv(np.dot(np.transpose(A),A)),np.transpose(A)),b)
        print(ma_vect)
        matrix_affine = [[ma_vect[0],ma_vect[1],ma_vect[2]],[ma_vect[3],ma_vect[4],ma_vect[5]],[0,0,1]]
        #print(matrix_affine)
        print("rgb")
        print(self.kinect.rgb_click_points[0])
        z = np.array([1])
        print(np.dot(matrix_affine,np.transpose(np.concatenate((self.kinect.rgb_click_points[0],[1]),axis = None))))
        print(np.dot(matrix_affine,np.transpose(np.concatenate((self.kinect.rgb_click_points[1],[1]),axis = None))))
        print(np.dot(matrix_affine,np.transpose(np.concatenate((self.kinect.rgb_click_points[2],[1]),axis = None))))
        print(np.dot(matrix_affine,np.transpose(np.concatenate((self.kinect.rgb_click_points[3],[1]),axis = None))))
        print(np.dot(matrix_affine,np.transpose(np.concatenate((self.kinect.rgb_click_points[4],[1]),axis = None))))
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)