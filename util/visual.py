import time
import numpy as np
import tp_test

import matplotlib.pyplot as plt

tp = tp_test.TrajectoryPlanner()

waypoints = np.array([[ 0.0, 0.0, 0.0, 0.0, 0.0],
                    [ 1.0, 0.8, 1.0, 0.5, 1.0],
                    [-1.0,-0.8,-1.0,-0.5, -1.0],
                    [-1.0, 0.8, 1.0, 0.5, 1.0],
                    [1.0, -0.8,-1.0,-0.5, -1.0],
                    [ 0.0, 0.0, 0.0, 0.0, 0.0]])
    
max_speed = 0.4  # in radius/s

Time = []
Q = []
V = []
t_last = 0

for i in range(len(waypoints) - 1):
    tp.set_initial_wp(waypoints[i])
    tp.set_final_wp(waypoints[i+1])
    T = tp.calc_time_from_waypoints(max_speed)

    [t, plan] = tp.generate_cubic_spline(T)

    plan_q = plan[0]
    plan_v = plan[1]

    plan_q = plan_q.transpose()
    plan_v = plan_v.transpose()

    if i == 0:
        Time = t
        Q = plan_q
        V = plan_v
    else:
        Time = np.concatenate((Time, t+t_last))
        Q = np.concatenate((Q,plan_q), axis=1)
        V = np.concatenate((V,plan_v), axis=1)

    t_last = t_last + t[-1]

inte_q = np.zeros((5,1))

for qi in Q.T:
    for 



ax1 = plt.subplot2grid(shape=(2,6), loc=(0,0), colspan=2)
ax2 = plt.subplot2grid((2,6), (0,2), colspan=2)
ax3 = plt.subplot2grid((2,6), (0,4), colspan=2)
ax4 = plt.subplot2grid((2,6), (1,1), colspan=2)
ax5 = plt.subplot2grid((2,6), (1,3), colspan=2)

axs = [ax1, ax2, ax3, ax4, ax5]

for i, [Qi, Vi] in enumerate(zip(Q, V)):

    plt.suptitle(['joint #', i])
    axs[i].plot(Time, Qi, 'r--', Time, Vi, 'b')
    
plt.show()

# print(plan[0][-1])



    # tp.execute_plan(plan, 10)

# self.set_next_state("idle")
