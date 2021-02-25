from xh430 import *
import time
import matplotlib.pyplot as plt
import numpy as np

# create motor object
my_dxl_L = XH430(1)
my_dxl_R = XH430(2)  # Right actuator (base close to us)

# connecting
XH430.open_port()
XH430.set_baudrate()

# setting the position limit
my_dxl_L.set_position_limit()
my_dxl_R.set_position_limit()
# activating torque
my_dxl_L.enable_torque()
my_dxl_R.enable_torque()
# check operating mode
my_dxl_L.get_operating_mode()
my_dxl_R.get_operating_mode()

# Linear Trajectory to follow
lin_traj = [*range(1820, 2275, 1)]  # from 70deg to 110deg


def user_input():
    ans = input("Continue ? y/n : ")
    if ans == "n":
        return False
    else:
        return True


def deg2pos(angle):
    return int(4095.0/360*angle + 4095.0/4)


def motionGradient(traj, dt):
    return len(traj)/dt

def iniPosition(motor_object, traj):
    motor_object.set_position(traj[0])
    time.sleep(2)

def singleAct(motor_object, traj, dt, forward):
    if forward:
        forward_coeff = 1
        last_index = -1
    else:
        forward_coeff = -1
        last_index = 0
    start_pos = motor_object.get_position()
    gradient = int(motionGradient(traj, dt))  # gradient of the motion
    step = 1
    while abs(step) <= dt:
        input_pos = gradient*step*forward_coeff + start_pos
        input_pos = motor_object.set_position(input_pos)
        while 1:
            present_pos = motor_object.get_position()
            if not abs(input_pos - present_pos) > motor_object.DXL_MOVING_STATUS_THRESHOLD:
                step += 1
                break
    motor_object.set_position(traj[last_index])
    time.sleep(.75) # time for the actuator to reach the final position
    print("the final position is :")
    final_pos = motor_object.get_position()
    return final_pos

iniPosition(my_dxl_R, lin_traj)
dt = int(input("How long do you want the motion to last :"))
singleAct(my_dxl_R, lin_traj, dt, True)
singleAct(my_dxl_R, lin_traj, dt, False)


# deconnecting
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
XH430.close_port()