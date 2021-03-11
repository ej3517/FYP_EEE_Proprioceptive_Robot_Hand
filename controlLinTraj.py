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
    """change position of one actuator with a given speed"""
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


def parallelDualAct(motor_L, motor_R, traj, dt, forward):
    """parallel motion of the actuator with a given speed"""
    if forward:
        forward_coeff = 1
        last_index = -1
    else:
        forward_coeff = -1
        last_index = 0
    start_pos_L = motor_L.get_position()
    start_pos_R = motor_R.get_position()
    gradient = int(motionGradient(traj, dt))  # gradient of the motion
    step = 1
    while abs(step) <= dt:
        input_pos_L = gradient*step*forward_coeff + start_pos_L
        input_pos_R = gradient*step*forward_coeff + start_pos_R
        input_pos_L = motor_L.set_position(input_pos_L)
        input_pos_R = motor_R.set_position(input_pos_R)
        while 1:
            present_pos_L = motor_L.get_position()
            present_pos_R = motor_R.get_position()

            if ((not abs(input_pos_L - present_pos_L) > motor_L.DXL_MOVING_STATUS_THRESHOLD) and
                (not abs(input_pos_R - present_pos_R) > motor_R.DXL_MOVING_STATUS_THRESHOLD)):
                step += 1
                break
    motor_L.set_position(traj[last_index])
    motor_R.set_position(traj[last_index])
    time.sleep(.75) # time for the actuator to reach the final position
    print("the final position is :")
    final_pos_L = motor_L.get_position()
    final_pos_R = motor_R.get_position()
    return [final_pos_L, final_pos_R]


iniPosition(my_dxl_L, lin_traj)
iniPosition(my_dxl_R, lin_traj)
dt = int(input("How long do you want the motion to last :")) # after testing 30 ?
#singleAct(my_dxl_L, lin_traj, dt, True)
#singleAct(my_dxl_L, lin_traj, dt, False)
parallelDualAct(my_dxl_L, my_dxl_R, lin_traj, dt, True)
parallelDualAct(my_dxl_L, my_dxl_R, lin_traj, dt, False)

print(my_dxl_L.get_position())

# deconnecting
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
XH430.close_port()