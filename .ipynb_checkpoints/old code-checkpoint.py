def reset(start=None):
    head.reset_state(lying_down_start if start is None else start, Solo8Config.v0)

    for _ in range(500):  # lay robot down
        tau = np.zeros((8))
        head.set_control('ctrl_joint_torques', tau)
        head.write()

        
def bounce_in_place_hardcoded_single_frame(up):  # single frame, have to loop over this function
    joint_pos = head.get_sensor('joint_positions')
    joint_velocities = head.get_sensor('joint_velocities')
    
    a = np.pi*np.array([1.0/6, -1.0/3]*2)
    target_up = np.concatenate((a, -1*a))
    
    a = np.pi*np.array([0.5, -1]*2)
    target_down = np.concatenate((a, -1*a))
    
    if up:
        if np.sum(np.abs(joint_pos - target_up)) < 0.5:
            up = False
    
        tau = K*(target_up - joint_pos) - D*joint_velocities
            
    else:
        if np.sum(np.mod(np.abs(joint_pos - target_down), 2*np.pi)) < 0.5:
            up = True
        tau = -D/4*joint_velocities  # let down slowly 
        # this was because dampening was too high, simulation would start "flying"
                
    return tau, up


def bounce_in_place_hardcoded():  # multiple frames: goes up and down, call as many times as you want it to bounce
    a = np.pi*np.array([1.0/6, -1.0/3]*2)  # same as angle adjust, angle adjust was written later
    target_up = np.concatenate((a, -1*a))
    
    a = np.pi*np.array([0.5, -1]*2)
    target_down = np.concatenate((a, -1*a))
    
    while True:
        head.read()
        joint_pos = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')
        
        if np.sum(np.mod(np.abs(joint_pos - target_up), 2*np.pi)) < 0.5:
            break
    
        tau = K*(target_up - joint_pos) - D*joint_velocities
        
        head.set_control('ctrl_joint_torques', tau)
        head.write()
            
    while True:
        head.read()
        joint_pos = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')
        
        if np.sum(np.mod(np.abs(joint_pos - target_down), 2*np.pi)) < 0.5:
            break
            
        tau = -D/4*joint_velocities  # let down slowly
    
        head.set_control('ctrl_joint_torques', tau)
        head.write()
        
        
def stay_still():  # goes up and stays there until slider a > 0.5
    a = np.pi*np.array([1.0/6, -1.0/3]*2)
    target_up = np.concatenate((a, -1*a))
    
    a = np.pi*np.array([0.5, -1]*2)
    target_down = np.concatenate((a, -1*a))
    
    while True:  # stand up
        head.read()
        joint_pos = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')
        
        if np.sum(np.mod(np.abs(joint_pos - target_up), 2*np.pi)) < 0.5:
            break
        
        tau = K*(target_up - joint_pos) - D*joint_velocities
        
        head.set_control('ctrl_joint_torques', tau)
        head.write()
    
    
    while True:  # stay still
        head.read()
        slider_a_pos = head.get_sensor('slider_positions')[0]
        
        if slider_a_pos > 0.5: # don't move until slider > 0.5
            break
            
    
    while True:  # get down
        head.read()
        joint_pos = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')
        
        if np.sum(np.mod(np.abs(joint_pos - target_down), 2*np.pi)) < 0.5:
            break
            
        tau = -D/4*joint_velocities  # let down slowly
    
        head.set_control('ctrl_joint_torques', tau)
        head.write()

        
def move_with_slider():  
    while 1:  
        head.read()
        joint_pos = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')
        slider_pos = head.get_sensor('slider_positions')
        A = slider_pos[0]
                    

        target = angle_adjust*A/2  # div by 2 (/2) lets you use the show slider; no longer restricted to lower half
            
        tau = K*(target - joint_pos) - D*joint_velocities
        
        head.set_control('ctrl_joint_torques', tau)
        head.write()


def move_leg_forward():
    a = np.pi*np.array([1/6, -1/3]*2)
    target_up = np.concatenate((a, -1*a))
    
    a = np.pi*np.array([0.5, -1]*2)

    target_down = np.concatenate((a, -1*a))
    while True:  # up
        head.read()
        joint_pos = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')
        
        if np.sum(np.abs(joint_pos - target_up)) < 0.5:
            break
        
        tau = K*(target_up - joint_pos) - D*joint_velocities
        
        head.set_control('ctrl_joint_torques', tau)
        head.write()
    
    while True:  # down
        head.read()
        joint_pos = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')
        slider_a_pos = head.get_sensor('slider_positions')[0]
        
        if slider_a_pos > 0.5: # manual exit
            break
        
        if np.sum(np.mod(np.abs(joint_pos - target_down), 2*np.pi)) < 0.5:
            break
        
        tau = -D/4*joint_velocities  # let down slowly
    
        head.set_control('ctrl_joint_torques', tau)
        head.write()


def bounce_in_place():
    target_up = angle_adjust * 1/6
    
    head.read()    
    joint_positions = head.get_sensor('joint_positions')
    
    L = np.linspace(joint_positions, target_up, num=2000)
    
    for target in L:
        head.read()
        
        joint_positions = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')
        slider_positions = head.get_sensor('slider_positions')
        
        tau = K*(target - joint_positions) - D*joint_velocities
        
        head.set_control('ctrl_joint_torques', tau)
        head.write()
            
    target_down = angle_adjust * 0.4
    
    head.read()
    joint_positions = head.get_sensor('joint_positions')
    print(joint_positions- target_up)
    L = np.linspace(joint_positions, target_down, num=2000)
    
    for target in L:
        head.read()
        
        joint_positions = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')
        slider_positions = head.get_sensor('slider_positions')
        
        tau = K/10*(target - joint_positions) - D*joint_velocities
        
        head.set_control('ctrl_joint_torques', tau)
        head.write()

        
def up_then_slider():  # up, then use slider
    reset()

    head.read()
    joint_positions = head.get_sensor('joint_positions')
    target_up = angle_adjust * head.get_sensor('slider_positions')[0]/2
    L = np.linspace(joint_positions, target_up, num=2000)

    for target in L:
        head.read()
        joint_positions = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')

        tau = K*(target - joint_positions) - D*joint_velocities

        head.set_control('ctrl_joint_torques', tau)
        head.write()

    while 1:  
        head.read()
        joint_pos = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')
        slider_pos = head.get_sensor('slider_positions')

        A = slider_pos[0]/2
        B_t = slider_pos[1] * angle_adjust/2


        target = angle_adjust*A
    #     target[3:7] = B_t[3:7]
    #     target[7:] = B_t[7:]
    #     target[2:4] = B_t[2:4]
        target[2:6] = B_t[2:6]

    #     print(target, end='\r')

        tau = K*(target - joint_pos) - D*joint_velocities

        head.set_control('ctrl_joint_torques', tau)
        head.write()
        
        
def up_then_down():  # up, then down; linspace/slider-esque
    reset()  # to laying down

    target_up = angle_adjust * 0.01

    head.read()
    joint_positions = head.get_sensor('joint_positions')

    L = np.linspace(joint_positions, target_up, num=2000)

    for target in L:
        head.read()
        joint_positions = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')

        tau = K*(target - joint_positions) - D*joint_velocities

        head.set_control('ctrl_joint_torques', tau)
        head.write()

    target_down = angle_adjust * 0.49

    head.read()
    joint_positions = head.get_sensor('joint_positions')

    L = np.linspace(joint_positions, target_down, num=2000)

    for target in L:
        head.read()
        joint_positions = head.get_sensor('joint_positions')
        joint_velocities = head.get_sensor('joint_velocities')

        tau = K*(target - joint_positions) - D*joint_velocities

        head.set_control('ctrl_joint_torques', tau)
        head.write()