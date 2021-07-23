"""code to plot the GRFs with and without h."""
import matplotlib.pylab as plt

from mim_data_utils import DataReader

import numpy as np

import pinocchio as pin

from robot_properties_solo.solo12wrapper import Solo12Config

plt.rcParams['figure.dpi'] = 144  # default 72.0, make plot bigger

pin_robot = Solo12Config.buildRobotWrapper()
reader = DataReader('2021-07-20_11-10-52_reactive.mds')

reader_q = reader.data['q']
reader_dq = reader.data['dq']
reader_tau = reader.data['tau']

calculated_f = np.empty((reader_q.shape[0], 4, 3))

# calculate f
for ms in range(reader_q.shape[0]):
    q = reader_q[ms]
    dq = reader_dq[ms]
    pin_robot.computeJointJacobians(q)

    for i, endeff_name in enumerate(['FL_ANKLE', 'FR_ANKLE', 'HL_ANKLE', 'HR_ANKLE']):
        frame_id = pin_robot.model.getFrameId(endeff_name)
        pin_robot.framePlacement(q, index=frame_id)
        J = pin_robot.getFrameJacobian(frame_id=frame_id, rf_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_inv = np.linalg.pinv(J[:3, 6+3*i:9+3*i].T) * -1
        h = pin_robot.nle(q, dq)
        t = np.hstack((np.zeros(6), reader_tau[ms][6:]))
        calculated_f[ms, i] = J_inv @ ((t - h)[6+3*i:9+3*i])

# calculate f without h (nle)
without_h = np.empty((reader_q.shape[0], 4, 3))
for ms in range(reader_q.shape[0]):
    q = reader_q[ms]
    dq = reader_dq[ms]
    pin_robot.computeJointJacobians(q)

    for i, endeff_name in enumerate(['FL_ANKLE', 'FR_ANKLE', 'HL_ANKLE', 'HR_ANKLE']):
        frame_id = pin_robot.model.getFrameId(endeff_name)
        pin_robot.framePlacement(q, index=frame_id)
        J = pin_robot.getFrameJacobian(frame_id=frame_id, rf_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_inv = np.linalg.pinv(J[:3, 6+3*i:9+3*i].T) * -1
        t = np.hstack((np.zeros(6), reader_tau[ms][6:]))
        without_h[ms, i] = J_inv @ (t[6+3*i:9+3*i])

slice_h = slice(1000, 2000)
plt.plot(calculated_f[slice_h, 2, 2])
plt.plot(without_h[slice_h, 2, 2])
plt.legend(('F_z with h', 'F_z without h'))
plt.show()
