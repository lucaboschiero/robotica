#common stuff 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm

import os
from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.kin_dyn_utils import RNEA
from base_controllers.utils.kin_dyn_utils import getM
from base_controllers.utils.kin_dyn_utils import getg
from base_controllers.utils.kin_dyn_utils import getC


import L1_conf as conf

#instantiate graphic utils
os.system("killall rosmaster rviz")
ros_pub = RosPub("ur4")
robot = getRobotModel("ur4")


# Init variables
zero = np.array([0.0, 0.0, 0.0, 0.0])
time = 0.0

# Init loggers
q_log = np.empty((4))*nan
q_des_log = np.empty((4))*nan
qd_log = np.empty((4))*nan
qd_des_log = np.empty((4))*nan
qdd_log = np.empty((4))*nan
qdd_des_log = np.empty((4))*nan
tau_log = np.empty((4))*nan
f_log = np.empty((3,0))*nan
x_log = np.empty((3,0))*nan
time_log =  np.empty((0,0))*nan

# M_log = np.empty((4,4))*nan
# g_log = np.empty ((4,1))*nan
# C_log = np.empty((4,1))*nan 

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des = zero
qd_des = zero
qdd_des = zero        # joint reference acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

error = np.array([1, 1, 1, 1])


# Main loop to simulate dynamics
while any(i >= 0.01 for i in np.abs(error)):
       
    # initialize Pinocchio variables
    robot.computeAllTerms(q, qd)
    # vector of gravity acceleration
    g0 = np.array([0.0, 0.0, -9.81])
    ##############################
    # Exercise 3.1: implement RNEA
    ##############################

    # # compute RNEA with your function
    #tau = RNEA(g0,q,qd,qdd)
    # compute RNEA with Pinocchio
    #taup = pin.rnea(robot.model, robot.data, q, qd, qdd)

    # print(taup - tau)

    ######################################
    # Exercise 3.2: compute dynamic terms
    ######################################
    # gravity terms
    g = getg(q, robot)
    # Pinocchio
    gp = robot.gravity(q)

    # compute joint space inertia matrix with Pinocchio
    M = getM(q,robot)

    # joint space inertia with Pinocchio
    # using native function
    Mp = robot.mass(q, False)

    # compute joint space intertia matrix with built-in pinocchio rnea
    # Mp  = np.zeros((4,4))
    # for i in range(4):
    #     ei = np.array([0.0, 0.0, 0.0, 0.0])
    #     ei[i] = 1
    #     taup = pin.rnea(robot.model, robot.data, q, np.array([0,0,0,0]) ,ei)
    #     Mp[:4,i] = taup - g

    # Pinocchio bias terms
    hp = robot.nle(q, qd, False)
    c = getC(q,qd,robot)

    #############################################
    # Exercise 3.5: add a damping term
    #############################################
    # viscous friction to stop the motion
    damping = zero
    #damping =  - 20*qd

    #############################################
    # Exercise 3.3: compute joint accelerations
    #############################################

    # compute accelerations (torques are zero!)
    # Pinocchio
    #qdd = np.linalg.inv(Mp).dot(damping-hp)
    qdd = np.linalg.inv(M).dot(damping -c -g)

    #############################################
    # Exercise 3.4: Simulate the forward dynamics
    #############################################
    # Forward Euler Integration
    qd = qd + qdd * conf.dt
    q = q + conf.dt * qd  + 0.5 * pow(conf.dt,2) * qdd

    # Log Data into a vector
    time_log = np.append(time_log, time)
    q_log = np.vstack((q_log, q ))
    q_des_log= np.vstack((q_des_log, q_des))
    qd_log= np.vstack((qd_log, qd))
    qd_des_log= np.vstack((qd_des_log, qd_des))
    qdd_log= np.vstack((qdd_log, qd))
    qdd_des_log= np.vstack((qdd_des_log, qdd_des))

    # M_log = np.dstack((M_log, M))
    # C_log = np.dstack((C_log, C))
    # g_log = np.dtack((g_log, g))

    # update time
    time = time + conf.dt
                
    #publish joint variables
    ros_pub.publish(robot, q, qd)
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break
            
#raw_input("Robot came to a stop. Press Enter to continue")
ros_pub.deregister_node()

# plot joint variables
# plotJoint('position', 0, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
# plotJoint('velocity', 1, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
# plotJoint('acceleration', 2, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
# plotJoint('torque', 3, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
# raw_input("Press Enter to continue")





