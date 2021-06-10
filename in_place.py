import time
import numpy as np

import dynamic_graph_manager_cpp_bindings
from robot_properties_solo.solo8wrapper import Solo8Config 

# Create the dgm communication to the control process.
head = dynamic_graph_manager_cpp_bindings.DGMHead(Solo8Config.dgm_yaml_path)


D = 0.05
dt = 0.001
next_time = time.time() + dt

# new
# constant
K = 6

leg = 0


# In your control loop:
while (True):
    if time.time() >= next_time:
        next_time += dt

        ###
        # Get the latest measurements from the shared memory.
        head.read()


        # new
        slider_pos = head.get_sensor('slider_positions')[0] # slider 1
        joint_pos = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')

        # print(joint_pos,end='\r')
        print(joint_pos)

        # if slider_pos > 0.5:
        #     tau = 0*joint_pos
        # else:
        #     # tau = K*(slider_pos - joint_pos) - D * joint_velocities
        #     tau = 

        tau = K*(slider_pos - joint_pos) - D * joint_velocities
        for i in range(tau.shape[0]):
            pass

        # tau = 0*joint_pos
        tau = np.zeros_like(joint_pos)

        head.set_control('ctrl_joint_torques', tau)

        ###
        # Write the results into shared memory again.
        head.write()

    time.sleep(0.0001)