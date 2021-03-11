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


def deg2pos(angle):
    return int(4095.0/360*angle + 4095.0/4)


# Linear Trajectory to follow
lin_traj = [*range(deg2pos(40), deg2pos(140), 1)]  # from 70deg to 110deg


def user_input():
    ans = input("Continue ? y/n : ")
    if ans == "n":
        return False
    else:
        return True


def motionGradient(traj, dt):
    return len(traj)/dt


def iniPosition(motor_L, motor_R, traj):
    motor_L.set_position(traj[0])
    motor_R.set_position(traj[-1])
    time.sleep(2)

def set_current_control_mode(motor):
    motor.set_operating_mode(motor.CURRENT_CONTROL_MODE)
    if motor.id == 1:
        motor.set_current(20)   # low torque
    else:
        motor.set_current(-50)  # low torque


def singleAct(motor_object, traj, dt, forward):
    """change position of one actuator with a given speed"""
    motor_object.set_operating_mode(motor_object.POSITION_CONTROL_MODE)
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

def graspManip(motor_L, motor_R, traj, dt, forward):
    """parallel motion of the actuator with a given speed"""
    motor_L.set_operating_mode(motor_L.POSITION_CONTROL_MODE)
    motor_R.set_operating_mode(motor_R.POSITION_CONTROL_MODE)
    if forward:
        forward_coeff_L = 1
        forward_coeff_R = -1
        last_index_L = -1
        last_index_R = 0
    else:
        forward_coeff_L = -1
        forward_coeff_R = 1
        last_index_L = 0
        last_index_R = -1
    start_pos_L = motor_L.get_position()  # start position
    start_pos_R = motor_R.get_position()  # start position
    gradient = int(motionGradient(traj, dt))  # gradient of the motion
    step = 1
    while abs(step) <= dt:
        new_present_pos_L = motor_L.get_position()
        new_present_pos_R = motor_R.get_position()
        input_pos_L = gradient*step*forward_coeff_L + start_pos_L
        input_pos_R = gradient*step*forward_coeff_R + start_pos_R
        input_pos_L = motor_L.set_position(input_pos_L)
        input_pos_R = motor_R.set_position(input_pos_R)
        while 1:
            present_pos_L = motor_L.get_position()
            present_pos_R = motor_R.get_position()
            if present_pos_L == new_present_pos_L or present_pos_R == new_present_pos_R:
                break
            elif ((not abs(input_pos_L - present_pos_L) > motor_L.DXL_MOVING_STATUS_THRESHOLD) and
                  (not abs(input_pos_R - present_pos_R) > motor_R.DXL_MOVING_STATUS_THRESHOLD)):
                step += 1
                break
            new_present_pos_L = motor_L.get_position()
            new_present_pos_R = motor_R.get_position()
        if present_pos_L == new_present_pos_L or present_pos_R == new_present_pos_R:
            break
    if (step > dt):
        motor_L.set_position(traj[last_index_L])
        motor_R.set_position(traj[last_index_R])
        time.sleep(.75) # time for the actuator to reach the final position
    else:
        motor_L.set_position(new_present_pos_L)
        motor_R.set_position(new_present_pos_R)
        time.sleep(.75) # time for the actuator to reach the final position
    print("the final position is :")
    final_pos_L = motor_L.get_position()
    final_pos_R = motor_R.get_position()
    return [*range(final_pos_R, traj[-1], 1)]


def controlRolling(motor_L, motor_R, traj):
    iniPosition(motor_L, motor_R, traj)
    dt = int(input("How long do you want the motion to last :")) # after testing 30 ?
    ### grasp the object
    new_traj = graspManip(motor_L, motor_R, traj, dt, True)
    time.sleep(.5)
    ### set the left finger to torque mode
    set_current_control_mode(motor_L)
    time.sleep(.75)
    ### start the motion of the right finger
    final_pos_R = singleAct(motor_R, new_traj, dt, True)
    new_traj = [*range(traj[0], final_pos_R, 1)]
    ### motion done so left finger goes to position mode
    motor_L.set_operating_mode(motor_L.POSITION_CONTROL_MODE)
    time.sleep(.75)
    ### set the right finger to torque mode
    set_current_control_mode(motor_R)
    time.sleep(.75)
    ### start the motion of the left finger
    singleAct(motor_L, new_traj, dt, False)
    ### motion done so left finger goes to position mode
    motor_R.set_operating_mode(motor_L.POSITION_CONTROL_MODE)
    time.sleep(.75)
    new_traj_L = [*range(motor_L.get_position(), 2047, 1)]
    new_traj_R = [*range(motor_R.get_position(), 2047, 1)]
    singleAct(motor_R, new_traj_R, dt, True)
    singleAct(motor_L, new_traj_L, dt, True)


controlRolling(my_dxl_L, my_dxl_R, lin_traj)

# deconnecting
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
XH430.close_port()