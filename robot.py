import time
import numpy as np
import pinocchio as pin
import dynamic_graph_manager_cpp_bindings
from robot_properties_solo.solo8wrapper import Solo8Config, Solo8Robot
from dynamic_graph_head import ThreadHead, Vicon, HoldPDController
# import imu_core.imu_core_cpp as IMU


def get_target(*args):
    angle_adjust = np.array([1, -2, 1, -2, -1, 2, -1, 2]) * np.pi
    if len(args) == 4:
        return np.array(list(args[i//2] for i in range(8))) * angle_adjust
    return np.array(args) * angle_adjust


class FootSliderController:
    def __init__(self, head, Kp, Kd, with_sliders=False):
        self.head = head
        self.Kp = Kp
        self.Kd = Kd

        self.slider_scale = np.pi
        self.with_sliders = with_sliders

        self.joint_positions = head.get_sensor('joint_positions')
        self.joint_velocities = head.get_sensor('joint_velocities')

        if with_sliders:
            self.slider_positions = head.get_sensor('slider_positions')


        # linspace code
        self.L = None
        self.i_L = 0
        self.go = 0
        self.num = 700

    def warmup(self, thread_head):
        self.zero_pos = self.joint_positions.copy()

        if self.with_sliders:
            self.slider_zero_pos = self.map_sliders(self.slider_positions)

    def go_zero(self):
        # TODO: Make this an interpolation.
        self.zero_pos = np.zeros_like(self.zero_pos)

        if self.with_sliders:
            self.slider_zero_pos = self.map_sliders(self.slider_positions)

    def map_sliders(self, sliders):
        sliders_out = np.zeros_like(self.joint_positions)
        if self.joint_positions.shape[0] == 8:
            slider_A = sliders[0]
            slider_B = sliders[1]
            for i in range(4):
                sliders_out[2 * i + 0] = slider_A
                sliders_out[2 * i + 1] = 2. * (1. - slider_B)

                if i >= 2:
                    sliders_out[2 * i: 2 * i + 2] *= -1

        return sliders_out

    def run(self, thread_head):
        def get_vicon(name1, name2=None):
            if name2 is None:
                name2 = name1
            pos, vel = thread_head.vicon.get_state(name1 + '/' + name2)
            return np.hstack([pos, vel])

        self.vicon_solo = get_vicon('solo8v2')
        self.vicon_leg_fr = get_vicon('solo8_fr', 'hopper_foot')
        self.vicon_leg_hl = get_vicon('solo8_hl', 'hopper_foot')
        self.vicon_leg_hr = get_vicon('solo8_hr', 'hopper_foot')

        # if self.with_sliders:
        #     self.des_position = self.slider_scale * (
        #         self.map_sliders(self.slider_positions) - self.slider_zero_pos
        #         ) + self.zero_pos 
        # else:
        #     self.des_position = self.zero_pos

        # linspace
        
        if self.go == 0:
            self.go = 1
            target = self.joint_positions.copy()
            target[5] += 0.5
            self.L = np.linspace(self.joint_positions, target, num=self.num)
        elif self.go == 1 and self.i_L >= self.L.shape[0]:
            self.go = 2
            self.i_L = 0
            target = self.joint_positions.copy()
            target[5] -= 0.5
            self.L = np.linspace(self.joint_positions, target, num=self.num)
        elif self.go == 2 and self.i_L >= self.L.shape[0]:
            self.L[self.L.shape[0]]  # switch to safety controller
        
        self.des_position = self.L[self.i_L]
        self.i_L += 1

        self.tau = self.Kp * (self.des_position - self.joint_positions) - self.Kd * self.joint_velocities

        self.tau[:4] = np.zeros(4)
        self.tau[6:] = np.zeros(2)

        thread_head.head.set_control('ctrl_joint_torques', self.tau)

class MyController:
    def __init__(self, head, Kp, Kd, with_sliders=False, num=100):
        self.head = head
        self.Kp = Kp
        self.Kd = Kd

        self.slider_scale = np.pi
        self.with_sliders = with_sliders

        self.joint_positions = head.get_sensor('joint_positions')
        self.joint_velocities = head.get_sensor('joint_velocities')

        if with_sliders:
            self.slider_positions = head.get_sensor('slider_positions')

        # variables for trotting
        leg_down = 0.1
        leg_up = 0.2
        extend = 0
        self.Targets = (
            # get_target(leg_down-extend, leg_down, leg_up, leg_up,
            #            leg_up, leg_up, leg_down+extend, leg_down),
            # get_target(leg_up, leg_up, leg_down-extend, leg_down,
                    #    leg_down+extend, leg_down, leg_up, leg_up),
            get_target(leg_down, leg_up, leg_up, leg_down),
            get_target(leg_up, leg_down, leg_down, leg_up),
        )
        self.i_Targets = 0
        self.L = None
        self.i_L = 0
        self.num = num

        # GRF z
        self.fz = np.zeros(4)


        ###
        self.robot = Solo8Config.buildRobotWrapper()

    def warmup(self, thread_head):
        self.zero_pos = self.joint_positions.copy()

        if self.with_sliders:
            self.slider_zero_pos = self.map_sliders(self.slider_positions)

    def go_zero(self):
        # TODO: Make this an interpolation.
        self.zero_pos = np.zeros_like(self.zero_pos)

        if self.with_sliders:
            self.slider_zero_pos = self.map_sliders(self.slider_positions)

    def map_sliders(self, sliders):
        sliders_out = np.zeros_like(self.joint_positions)
        if self.joint_positions.shape[0] == 8:
            slider_A = sliders[0]
            slider_B = sliders[1]
            for i in range(4):
                sliders_out[2 * i + 0] = slider_A
                sliders_out[2 * i + 1] = 2. * (1. - slider_B)

                if i >= 2:
                    sliders_out[2 * i + 0] *= -1
                    sliders_out[2 * i + 1] *= -1

        return sliders_out

    # def get_vicon(self, name1, name2=None):
    #     return np.hstack(thread_head.vicon.get_state(name1 + '/' + (name1 if name2 is None else name2)))
    
    def _vicon(self, thread_head):
        pass
        # self.vicon_solo = self.get_vicon('solo8v2')
        # self.vicon_leg_fr = self.get_vicon('solo8_fr', 'hopper_foot')
        # self.vicon_leg_hl = self.get_vicon('solo8_hl', 'hopper_foot')
        # self.vicon_leg_hr = self.get_vicon('solo8_hr', 'hopper_foot')
        

    def _move(self, thread_head):
        if self.with_sliders:
            self.des_position = self.slider_scale * (
                self.map_sliders(self.slider_positions) - self.slider_zero_pos
                ) + self.zero_pos
        else:
            self.des_position = self.zero_pos

        # if self.L is None or self.i_L >= len(self.L):
        #     self.L = np.linspace(self.joint_positions, self.Targets[self.i_Targets], num=self.num)
        #     self.i_L = 0
        #     self.i_Targets = (self.i_Targets + 1) % len(self.Targets)

        # self.des_position = self.L[self.i_L]
        # self.i_L += 1

        # self.tau = self.Kp * (self.des_position - self.joint_positions) - self.Kd * self.joint_velocities

    def _GRF(self, thread_head):
        head = thread_head.head
        PR = Solo8Config.buildRobotWrapper()
        # vicon = thread_head.vicon.get_state('solo8v2/solo8v2')
        vicon = (np.zeros(7), np.zeros(6))
        q = np.hstack((vicon[0], head.get_sensor('joint_positions')))
        v = np.hstack((vicon[1], head.get_sensor('joint_velocities')))
        endeff_names = ['FL_ANKLE', 'FR_ANKLE', 'HL_ANKLE', 'HR_ANKLE']

        PR.computeJointJacobians(q)

        f = np.zeros((4,3))
        for i in range(4):
            frame_id = PR.model.getFrameId(endeff_names[i])
            PR.framePlacement(q, index=frame_id)
            J = PR.getFrameJacobian(frame_id=frame_id, rf_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

            J_inv = np.linalg.pinv(J[:3][:,6+2*i:8+2*i].T) * -1
            h = PR.nle(q, v)
            _a = (np.hstack((np.zeros(6), thread_head.active_controllers[0].tau)) - h)[6+2*i:8+2*i]
            f[i] = J_inv @ _a 
        self.fz = f[:, 2]
        return f

    def run(self, thread_head):
        self._move(thread_head)  # calculate tau
        self._vicon(thread_head)  # vicon objects
        self._GRF(thread_head)  # calculate GRF

        # self.tau = np.zeros(8)
        thread_head.head.set_control('ctrl_joint_torques', self.tau)

if __name__ == "__main__":
    ###
    # Create the dgm communication and instantiate the controllers.
    head = dynamic_graph_manager_cpp_bindings.DGMHead(Solo8Config.dgm_yaml_path)

    # Create the controllers.
    hold_pd_controller = HoldPDController(head, 3., 0.05, with_sliders=True)

    my_controller = MyController(head, 3., 0.05, with_sliders=True, num=250)

    foot_slider_controller = FootSliderController(head, 3., 0.05, with_sliders=True)

    thread_head = ThreadHead(
        0.001,
        hold_pd_controller,
        head,
        [
            # ('vicon', Vicon('172.24.117.119:801', [
            #     'solo8v2/solo8v2',
            #     'solo8_fr/hopper_foot',
            #     'solo8_hl/hopper_foot',
            #     'solo8_hr/hopper_foot'
            # ]))
        ]
    )

    # Start the parallel processing.
    thread_head.start()

    time.sleep(0.1)

    # thread_head.switch_controllers(my_controller)
    thread_head.switch_controllers(foot_slider_controller)
    
    thread_head.start_logging()
    time.sleep(3)
    thread_head.stop_logging()
