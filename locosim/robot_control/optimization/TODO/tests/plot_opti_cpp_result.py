"""
Plot opti_c results
Author: Niraj Rathod
Date : 16/10/20

"""

import os
import sys
if sys.version_info[:2] == (2, 7):
    print("USING python 2.7")
else:
    sys.path = ['/usr/lib/python3.5/lib-dynload',  # package mmap
                '/usr/lib/python3.5/',  # numbers.py is here
                '/usr/local/lib/python3.5/dist-packages/',  # sudo pip3 install installs here numpy
                '/usr/lib/python3/dist-packages/',  # sudo pip3 install installs here yaml
                '/opt/ros/kinetic/lib/python2.7/dist-packages/',  # rospy genpy
                '/usr/lib/python2.7/dist-packages/',  # rospkg is here
                os.environ['HOME'] + '/' + os.environ['ROS_WORKSPACE_NAME'] + '/install/lib/python2.7/dist-packages',
                # where reference generator and ros ipedance controlelr messages are
                '','..']  # this is the current directory
import yaml
import scipy.io
import numpy as np
from tools.plottingFeatures import plotResult
from opti.optimization_utils import compute_cost
from tools.getConfig import getConfig

# plot struct
class plot_struct():
    pass

# get optimization params
optiConfig = getConfig()

# Load the optimization parameters
nx = optiConfig.nx
nu = optiConfig.nu
N = optiConfig.prediction_horizon
active_plots = optiConfig.active_plots

# Load desired states, force and parameter from opti_c
states_cpp = np.loadtxt("../data/states_cpp.txt")
forces_cpp = np.loadtxt("../data/forces_cpp.txt")
stance_cpp = np.loadtxt("../data/stance_cpp.txt").T
foot_pos_cpp = np.loadtxt("../data/foot_pos_cpp.txt")
miscellanea_cpp = np.loadtxt("../data/miscellanea_cpp.txt")
force_dot_cpp = np.loadtxt("../data/force_dot_cpp.txt")

# - unpack states
act_state_cpp = states_cpp[:,0:12].T
des_state_cpp = states_cpp[:,12:24].T
ref_state_cpp = states_cpp[:,24:].T

# - unpack forces
act_force_cpp = forces_cpp[:,0:12].T
des_force_cpp = forces_cpp[:,12:24].T
ref_force_cpp = forces_cpp[:,24:].T


# - unpack feet position
act_foot_pos_cpp = foot_pos_cpp[:,0:12].T
des_foot_pos_cpp = foot_pos_cpp[:,12:24].T
ref_foot_pos_cpp = foot_pos_cpp[:,24:].T

# - unpack stability margin and replanning flag
stab_margin_cpp = miscellanea_cpp[:,0].T
replanning_cpp = miscellanea_cpp[:,1].T

# - unpack force_dot
des_force_dot_cpp = force_dot_cpp[:,:].T

# struct for plot function
plot_struct = plot_struct()
plot_struct.replanning_flag_log = replanning_cpp
plot_struct.des_force_dot_log = des_force_dot_cpp
plot_struct.stance_flag_log =  stance_cpp

plot_struct.des_force_log = des_force_cpp
plot_struct.des_state_log = des_state_cpp
plot_struct.des_feetW_log = des_foot_pos_cpp

plot_struct.ref_force_log = ref_force_cpp
plot_struct.ref_state_log = ref_state_cpp
plot_struct.ref_feetW_log = ref_foot_pos_cpp

plot_struct.act_force_log = act_force_cpp
plot_struct.act_state_log = act_state_cpp
plot_struct.act_feetW_log = act_foot_pos_cpp

plot_struct.stability_margin_log = stab_margin_cpp

plot_struct.legend_desired = 'desired'
plot_struct.legend_actual = 'actual'
plot_struct.legend_ref = 'ref'

plot_struct.model_type = optiConfig.model_type

plotResult(plot_struct, active_plots)

# # compute cost from opti_c results
# args ={}
# args['nx'] = optiConfig.nx
# args['nu'] = optiConfig.nu
# args['N']= optiConfig.prediction_horizon
# args['Ts'] = optiConfig.Ts
# args['model_type'] = optiConfig.model_type
# args['include_mobility'] = optiConfig.include_mobility
#
# if optiConfig.model_type == 2:
#     # - weights for the cost
#     args['Q'] = np.diag(np.hstack([optiConfig.q_array, optiConfig.r_array])) / optiConfig.Ts
#     args['R'] = np.diag(optiConfig.deltaU_array) / optiConfig.Ts
#     args['Qe'] = np.diag(np.hstack([optiConfig.qn_array, optiConfig.r_array]))
# else:
#     # - weights for the cost
#     args['Q'] = np.diag(optiConfig.q_array) / optiConfig.Ts
#     args['R'] = np.diag(optiConfig.r_array) / optiConfig.Ts
#     args['Qe'] = np.diag(optiConfig.qn_array)
#
# args['M'] = np.diag(optiConfig.m_array) / optiConfig.Ts
# args['ref_hiptofoot_pos'] = optiConfig.ref_hiptofoot_pos
# args['verbose'] = optiConfig.verbose
#
# compute_cost(args, ref_state_cpp, ref_force_cpp, des_state_cpp, des_force_cpp, np.zeros((nu,N)), ref_foot_pos_cpp)