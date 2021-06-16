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
