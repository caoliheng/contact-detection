import numpy as np


# constants
angle_adjust = np.array([1,-2,1,-2,-1,2,-1,2]) * np.pi


# functions
def get_target(*args):  # FL, FR, HL, HR
    if len(args) == 1:
        return get_target(*args[0])
    elif len(args) == 4:
        return np.array([args[0], args[0], args[1], args[1], args[2], args[2], args[3], args[3]])* angle_adjust
    elif len(args) == 8:
        return np.array([args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7]])* angle_adjust
    
    return np.zeros(8)


# ## 
# def reset(start=None):
#     global head
#     head.reset_state(lying_down_start if start is None else start, Solo8Config.v0)

#     for _ in range(500):  # lay robot down
#         tau = np.zeros((8))
#         head.set_control('ctrl_joint_torques', tau)
#         head.write()

        
# # single frame, have to loop over this function
# def bounce_in_place_hardcoded_single_frame(up):
#     joint_pos = head.get_sensor('joint_positions')
#     joint_velocities = head.get_sensor('joint_velocities')
    
#     a = np.pi*np.array([1.0/6, -1.0/3]*2)
#     target_up = np.concatenate((a, -1*a))
    
#     a = np.pi*np.array([0.5, -1]*2)
#     target_down = np.concatenate((a, -1*a))
    
#     if up:
#         if np.sum(np.abs(joint_pos - target_up)) < 0.5:
#             up = False
    
#         tau = K*(target_up - joint_pos) - D*joint_velocities
            
#     else:
#         if np.sum(np.mod(np.abs(joint_pos - target_down), 2*np.pi)) < 0.5:
#             up = True
#         tau = -D/4*joint_velocities  # let down slowly 
#         # this was because dampening was too high, simulation would start "flying"
                
#     return tau, up


# # multiple frames: goes up and down, call as many times as you want it to bounce
# def bounce_in_place_hardcoded():
#     a = np.pi*np.array([1.0/6, -1.0/3]*2)  # same as angle adjust, angle adjust was written later
#     target_up = np.concatenate((a, -1*a))
    
#     a = np.pi*np.array([0.5, -1]*2)
#     target_down = np.concatenate((a, -1*a))
    
#     while True:
#         head.read()
#         joint_pos = head.get_sensor('joint_positions')
#         joint_velocities = head.get_sensor('joint_velocities')
        
#         if np.sum(np.mod(np.abs(joint_pos - target_up), 2*np.pi)) < 0.5:
#             break
    
#         tau = K*(target_up - joint_pos) - D*joint_velocities
        
#         head.set_control('ctrl_joint_torques', tau)
#         head.write()
            
#     while True:
#         head.read()
#         joint_pos = head.get_sensor('joint_positions')
#         joint_velocities = head.get_sensor('joint_velocities')
        
#         if np.sum(np.mod(np.abs(joint_pos - target_down), 2*np.pi)) < 0.5:
#             break
            
#         tau = -D/4*joint_velocities  # let down slowly
    
#         head.set_control('ctrl_joint_torques', tau)
#         head.write()

        
# def move_with_slider():
#     while 1:  
#         head.read()
#         joint_pos = head.get_sensor('joint_positions')
#         joint_velocities = head.get_sensor('joint_velocities')
#         slider_pos = head.get_sensor('slider_positions')
#         A = slider_pos[0]
                    

#         target = angle_adjust*A/2  # div by 2 (/2) lets you use the whole slider; no longer restricted to lower half
            
#         tau = K*(target - joint_pos) - D*joint_velocities
        
#         head.set_control('ctrl_joint_torques', tau)
#         head.write()


# def bounce_in_place():
#     reset()
    
#     target_up = angle_adjust * 1/6
    
#     head.read()    
#     joint_positions = head.get_sensor('joint_positions')
    
#     L = np.linspace(joint_positions, target_up, num=2000)
    
#     for target in L:
#         head.read()
        
#         joint_positions = head.get_sensor('joint_positions')
#         joint_velocities = head.get_sensor('joint_velocities')
#         slider_positions = head.get_sensor('slider_positions')
        
#         tau = K*(target - joint_positions) - D*joint_velocities
        
#         head.set_control('ctrl_joint_torques', tau)
#         head.write()
            
#     target_down = angle_adjust * 0.4
    
#     head.read()
#     joint_positions = head.get_sensor('joint_positions')
#     print(joint_positions- target_up)
#     L = np.linspace(joint_positions, target_down, num=2000)
    
#     for target in L:
#         head.read()
        
#         joint_positions = head.get_sensor('joint_positions')
#         joint_velocities = head.get_sensor('joint_velocities')
#         slider_positions = head.get_sensor('slider_positions')
        
#         tau = K/10*(target - joint_positions) - D*joint_velocities
        
#         head.set_control('ctrl_joint_torques', tau)
#         head.write()

        
# def up_then_slider():  # up, then use slider 
#     reset()

#     head.read()
#     joint_positions = head.get_sensor('joint_positions')
#     target_up = angle_adjust * head.get_sensor('slider_positions')[0]/2
#     L = np.linspace(joint_positions, target_up, num=2000)

#     for target in L:
#         head.read()
#         joint_positions = head.get_sensor('joint_positions')
#         joint_velocities = head.get_sensor('joint_velocities')

#         tau = K*(target - joint_positions) - D*joint_velocities

#         head.set_control('ctrl_joint_torques', tau)
#         head.write()

#     while 1:  
#         head.read()
#         joint_pos = head.get_sensor('joint_positions')
#         joint_velocities = head.get_sensor('joint_velocities')
#         slider_pos = head.get_sensor('slider_positions')

#         A = slider_pos[0]/2
#         B_t = slider_pos[1] * angle_adjust/2


#         target = angle_adjust*A
#     #     target[3:7] = B_t[3:7]
#     #     target[7:] = B_t[7:]
#     #     target[2:4] = B_t[2:4]
#         target[2:6] = B_t[2:6]

#     #     print(target, end='\r')

#         tau = K*(target - joint_pos) - D*joint_velocities

#         head.set_control('ctrl_joint_torques', tau)
#         head.write()
        
        
# def up_then_down():  # up, then down; linspace/slider-esque 
#     reset()  # to laying down

#     target_up = angle_adjust * 0.01

#     head.read()
#     joint_positions = head.get_sensor('joint_positions')

#     L = np.linspace(joint_positions, target_up, num=2000)

#     for target in L:
#         head.read()
#         joint_positions = head.get_sensor('joint_positions')
#         joint_velocities = head.get_sensor('joint_velocities')

#         tau = K*(target - joint_positions) - D*joint_velocities

#         head.set_control('ctrl_joint_torques', tau)
#         head.write()

#     target_down = angle_adjust * 0.49

#     head.read()
#     joint_positions = head.get_sensor('joint_positions')

#     L = np.linspace(joint_positions, target_down, num=2000)

#     for target in L:
#         head.read()
#         joint_positions = head.get_sensor('joint_positions')
#         joint_velocities = head.get_sensor('joint_velocities')

#         tau = K*(target - joint_positions) - D*joint_velocities

#         head.set_control('ctrl_joint_torques', tau)
#         head.write()


# # def get_target(FL, FR, HL, HR, a=None, b=None, c=None, d=None):  # FL, FR, HL, HR
# #     if a is None:
# #         return np.array([FL, FL, FR, FR, HL, HL, HR, HR]) * angle_adjust
# #     else:
# #         return np.array([FL, FR, HL, HR, a, b, c, d]) * angle_adjust


# def get_target(*args):  # FL, FR, HL, HR
#     if len(args) == 1:
#         return get_target(*args[0])
#     elif len(args) == 4:
#         return np.array([args[0], args[0], args[1], args[1], args[2], args[2], args[3], args[3]])* angle_adjust
#     elif len(args) == 8:
#         return np.array([args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7]])* angle_adjust
    
#     return np.zeros(8)


# def go_to(T, num=2000):  # go to (with) linspace (for smoothness)
#     head.read()
#     joint_positions = head.get_sensor('joint_positions')
# #     T = get_target()
#     L = np.linspace(joint_positions, T, num=num)

#     for target in L:
#         head.read()
#         joint_positons = head.get_sensor('joint_positions')
#         joint_velocities = head.get_sensor('joint_velocities')

#         tau = K*(target - joint_positions) - D*joint_velocities
        
#         head.set_control('ctrl_joint_torques', tau)
#         head.write()

        
# def walk():  # lean forward first
#     reset()

#     stand = 0.15  # leg angle when standing
#     lean = 0.25  # leg angle when leaning
#     _F = 0.1  # how much to lean forward by


#     #  walk forward
#     targets = (
#         (stand,) *4,  # stand up
#         (stand+_F, stand, stand+_F, stand, stand-_F, stand, stand-_F, stand),  # move body forward
#         (stand+_F+0.1, stand+0.1, stand+_F, stand, stand-_F+0.1, stand+0.1, stand-_F, stand),  # lean to left side

#         (stand+_F+0.1, stand+0.1, stand+_F, stand, stand-_F+0.1, stand+0.1, stand-_F+0.1, stand+0.1),  # lift HR
#         (stand+_F+0.1, stand+0.1, stand+_F, stand, stand-_F+0.1, stand+0.1, stand+0.1, stand),  # move HR forward
#         (stand+_F+0.1, stand+0.1, stand+_F, stand, stand-_F+0.1, stand+0.1, stand, stand),  # put HR down

#         (stand+_F+0.1, stand+0.1, stand+_F-0.1, stand-0.1, stand-_F+0.1, stand+0.1, stand, stand),  # lift FR
#         (stand+_F+0.1, stand+0.1, stand+_F-0.1, stand-0.1, stand-_F+0.1, stand+0.1, stand, stand),  # move HR forward


#     )

#     for T in targets:
#         go_to(get_target(T))

        
# def walk2():  # move legs first
#     reset()

#     stand = 0.15  # leg angle when standing
#     lean = 0.25  # leg angle when leaning

#     targets_to_go_to = [  # FL, FR, HL, HR
#         (stand,) * 4,  # stand up
#         (lean, stand, lean, stand),  # lean to left side
#         (lean, lean, stand, stand+0.1, lean, lean, stand, stand),  # lift FR up
#         (lean, lean, stand - 0.15, stand-0.05, lean, lean, stand, stand),  # move FR forward
#     ]

#     for T in targets_to_go_to:
#         go_to(get_target(T))


#     head.read()
#     # orig_j_p = get_target(0.2, 0.3, 0.2, 0.1)
#     orig_j_p = np.array(head.get_sensor('joint_positions'), copy=True)

#     while 0:
#         head.read()
#         joint_p = head.get_sensor('joint_positions')
#         joint_v = head.get_sensor('joint_velocities')
#         slider_p = head.get_sensor('slider_positions')

#         A = (slider_p[0]-0.5) * np.pi
#         B = slider_p[1] * np.pi

#         target = orig_j_p

#         target[6:8] = np.array([-A, 2*B])

#         tau = K*(target - joint_p) - D*joint_v
#         head.set_control('ctrl_joint_torques', tau)
#         head.write()
        
        
# def trot_inplace():
#     reset()

#     stand = 0.1  # leg angle when standing
#     lean = 0.2  # leg angle when leaning

#     go_to(get_target((stand,) * 4))

#     #  walk forward
#     targets = (
#         (stand, lean, lean, stand),
#         (lean, stand, stand, lean),
#     )

#     i = 0
#     while 1:
#         go_to(get_target(targets[i]), num=650)
#         i = (i+1) % len(targets)

#     # for T in targets:
#     #     go_to(get_target(T),num=1000)