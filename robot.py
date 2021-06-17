from my_functions import *

import time
import numpy as np

import dynamic_graph_manager_cpp_bindings
from robot_properties_solo.solo8wrapper import Solo8Config 

# Create the dgm communication to the control process.
head = dynamic_graph_manager_cpp_bindings.DGMHead(Solo8Config.dgm_yaml_path)

K = 5
D = 0.1
dt = 0.001
next_time = time.time() + dt


###
leg_down = 0.1
leg_up = 0.25
extend = -0.015  # extension constant

Targets = (
    get_target(leg_down-extend, leg_down, leg_up, leg_up, leg_up, leg_up, leg_down+extend, leg_down),
    get_target(leg_up, leg_up, leg_down-extend, leg_down, leg_down+extend, leg_down, leg_up, leg_up),
    # get_target(leg_down-extend, leg_down, leg_up, leg_up, leg_up, leg_up, leg_down+extend, leg_down),  wrong direction
    # get_target(leg_up, leg_up, leg_down-extend, leg_down, leg_down+extend, leg_down, leg_up, leg_up),
    # get_target(leg_down, leg_up, leg_up, leg_down),
    # get_target(leg_up, leg_down, leg_down, leg_up),
)


L = None
i_Targets = 0
i_L = -1


# In your control loop:
while True:
    if time.time() >= next_time:
        next_time += dt

        # read
        head.read()
        slider_P = head.get_sensor('slider_positions')
        joint_P = head.get_sensor('joint_positions')
        joint_V = head.get_sensor('joint_velocities')

        if slider_P[0] > 0.5:
            head.set_control('ctrl_joint_torques', np.zeros(8))
            head.write()
            break

        if L is None or i_L >= len(L):
            L = np.linspace(joint_P, Targets[i_Targets], num=250)
            i_L = 0
            i_Targets = (i_Targets + 1) % len(Targets)


        target = L[i_L]
        i_L += 1

        tau = K*(target - joint_P) - D*joint_V

        # write
        head.set_control('ctrl_joint_torques', tau)
        head.write()

    time.sleep(0.0001)