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
        pass

    def set_final_wp(self, waypoint):
        self.final_wp = waypoint
        pass

    def go(self, max_speed = 0.2):
        self.rexarm.set_speeds_normalized_global(max_speed,update_now=True)
        pass

    def stop(self):
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        ti = 0
        tf = max((final_wp-initial_wp))./max_speed
        T = np.array(ti,tf)
        return T
        pass

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        ti = T(0)
        tf = T(1)
        t = linspace(0,tf,(tf-ti)/self.dt)
        q = []
        M = np.array([1, t0, t0^2, t0^3],
            [0, 1, 2*t0, 3*t0^2],
            [1, tf, tf^2, tf^3],
            [0, 1, 2*tf, 3*tf^2])
        for i in enumerate(rexarm.joints):
            b = np.array(np.transpose([initial_wp[i],0,final_wp[i],0]))
            a = np.dot(np.invert(M),b)
            q.append(np.transpose(a[0]+a[1]*numpy.power(t,2)+a[3]*numpy.power(t,3))
        return yinterp   
        pass

    def execute_plan(self, plan, look_ahead=8):
        self.rexarm.set_initial_wp(np.array([0, 0, 0, 0, 0]))
        self.rexarm.set_final_wp(np.array([0, pi()/2, 0, 0, 0]))
        self.rexarm.go(0.15)
        T = self.rexarm.calc_time_from_waypoints(self.rexarm.initial_wp,self.rexarm.final_wp,self.maxspeed)
        q = self.rexarm.generate_cubic_spline(self.rexarm.initial_wp,self.rexarm.final_wp,T)
        for i in enumerate(q):
            self.rexarm.set_positions(q[i])
            self.rexarm.pause(0.1)
        pass