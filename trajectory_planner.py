import numpy as np 
import time

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints 
        self.dt = 0.05 # command rate
    
    def set_initial_wp(self, waypoint):
        self.initial_wp = waypoint
        

    def set_final_wp(self, waypoint):
        self.final_wp = waypoint
        

    def go(self, max_speed = 0.2):
        # not sure?
        self.rexarm.set_speeds_normalized_global(max_speed,update_now=True)


    def stop(self):
        # not sure?
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        

    def calc_time_from_waypoints(self, max_speed = 0.4):
        t0 = 0
        tf = max(abs(self.final_wp-self.initial_wp))/(max_speed)
        T = np.array([t0,tf])
        return T
        

    def generate_cubic_spline(self, T):
        t0 = T[0]
        tf = T[1]
        # tf = 3
        t = np.linspace(0,tf,(tf-t0)/self.dt)
        
        plan_q = np.array([])
        plan_v = np.array([])

        M = np.array([[1, t0, t0**2, t0**3],
            [0, 1, 2*t0, 3*t0**2],
            [1, tf, tf**2, tf**3],
            [0, 1, 2*tf, 3*tf**2]])
        # print(self.initial_wp)
        # print(self.final_wp)
        for iter,i in enumerate(range(self.num_joints)):
            b = np.transpose(np.array([self.initial_wp[i],0,self.final_wp[i],0]))
            a = np.dot(np.linalg.inv(M),b)
            
            qi = np.transpose([a[0] + a[1]*np.power(t,1) + a[2]*np.power(t,2) + a[3]*np.power(t,3)])
            vi = np.transpose([a[1] + 2*a[2]*t + 3*a[3]*np.power(t,2)])
            if i == 0:
                plan_q = qi
                plan_v = vi
            else:
                plan_q = np.concatenate((plan_q, qi), axis = 1)
                plan_v = np.concatenate((plan_v, vi), axis = 1)
        
        plan = np.concatenate(([plan_q], [plan_v]), axis = 0)
        # print(plan_v[-2])
        # print(plan_v[-1])
        return plan   
        

    def execute_plan(self, plan, look_ahead=8):
        plan_q = plan[0]
        plan_v = plan[1]    # velocity in radius/s     row=time_step, col=join#

        # for i in range(len(plan_q)):
        #     if i > len(plan_q)-look_ahead-1:
        #         self.rexarm.set_positions(plan_q[-1])
        #     else:
        #         self.rexarm.set_positions(plan_q[i+look_ahead])
            
        #     self.rexarm.pause(self.dt)


        # self.rexarm.set_positions(plan_q[-1])

        for i in range(len(plan_v)):
            if i > len(plan_v)-look_ahead-1:
                # self.rexarm.set_speeds(plan_v[-1])
                self.rexarm.set_positions(plan_q[-1])
            else:
                # self.rexarm.set_speeds(plan_v[i+look_ahead])
                self.rexarm.set_positions(plan_q[i+look_ahead])
            
            self.rexarm.set_speeds(plan_v[i])

            self.rexarm.pause(self.dt)