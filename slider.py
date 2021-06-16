import time
import numpy as np

import dynamic_graph_manager_cpp_bindings
from robot_properties_solo.solo8wrapper import Solo8Config

# Create the dgm communication to the control process.
head = dynamic_graph_manager_cpp_bindings.DGMHead(Solo8Config.dgm_yaml_path)


D = 0.05
dt = 0.001
next_time = time.time() + dt

K = 5

angle_adjust = np.array([1,-2,1,-2,-1,2,-1,2]) * np.pi


# In your control loop:
while (True):
    if time.time() >= next_time:
        next_time += dt

        head.read()
        slider_P = head.get_sensor('slider_positions')
        joint_P = head.get_sensor('joint_positions')
        joint_V = head.get_sensor('joint_velocities')

        target = slider_P[0]/2 * angle_adjust


        tau = K*(target - joint_P) - D*joint_V

        head.set_control('ctrl_joint_torques', tau)
        head.write()

    time.sleep(0.0001)