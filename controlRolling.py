from xh430 import *
import time
import matplotlib.pyplot as plt
import numpy as np
import json

# create motor object
my_dxl_L = XH430(1)
my_dxl_R = XH430(2)  # Right actuator (base close to us)

# connecting
XH430.open_port()
XH430.set_baudrate()

# make sure the torque is disabled to WRITE on the EEPROM registers
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
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
        motor.set_current(-30)  # low torque


def singleAct(motor_object, torque_motor, traj, dt, forward):
    """change position of one actuator with a given speed """
    data_pos = []
    data_pos_torque = []
    motor_object.set_operating_mode(motor_object.POSITION_CONTROL_MODE)
    if forward:
        forward_coeff = 1
        last_index = -1
    else:
        forward_coeff = -1
        last_index = 0
    start_pos = motor_object.get_position()
    data_pos += [start_pos]
    data_pos_torque += [torque_motor.get_position()]
    gradient = int(motionGradient(traj, dt))  # gradient of the motion
    step = 1
    while abs(step) <= dt:
        input_pos = gradient*step*forward_coeff + start_pos
        input_pos = motor_object.set_position(input_pos)
        while 1:
            present_pos = motor_object.get_position()
            data_pos += [present_pos]
            data_pos_torque += [torque_motor.get_position()]
            if not abs(input_pos - present_pos) > motor_object.DXL_MOVING_STATUS_THRESHOLD:
                step += 1
                break
    motor_object.set_position(traj[last_index])
    time.sleep(.75)  # time for the actuator to reach the final position
    data_pos += [motor_object.get_position()]
    data_pos_torque += [torque_motor.get_position()]
    return [data_pos, data_pos_torque]


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
    time.sleep(.5)


######### control of the entire motion #########


def controlRolling(motor_L, motor_R, traj):
    # PUT THE FINGERS AT THEIR INITIAL POSITION
    data_pos_L = []
    data_pos_R = []
    clockwise = True
    anticlockwise = False
    iniPosition(motor_L, motor_R, traj)
    # SET UP THE SPEED OF THE MOTION (30 SEEMS TO BE GOOD) / PUT THE OBJECT
    dt = 70  # int(input("How long do you want the motion to last :"))
    # INITIALIZE THE GRASP
    graspManip(motor_L, motor_R, traj, dt, clockwise)
    ######## CLOCKWISE MOTION
    # LF TORQUE MODE
    pos_L = motor_L.get_position()
    set_current_control_mode(motor_L)
    # RF NEW TRAJECTORY
    time.sleep(.75)
    # INITIALIZE LIST OF POSITION
    pos_R = motor_R.get_position()
    data_pos_R += [pos_R]
    data_pos_L += [pos_L]
    # RF TRAJECTORY
    new_traj = [*range(pos_R, traj[-1], 1)]
    # RF POSITION MODE AND START CLOCKWISE MOTION
    list_pos_RL = singleAct(motor_R, motor_L, new_traj, dt, clockwise)
    data_pos_R += list_pos_RL[0]
    data_pos_L += list_pos_RL[1]
    ######## ANTICLOCKWISE MOTION
    # LF POSITION CONTROL MODE
    motor_L.set_operating_mode(motor_L.POSITION_CONTROL_MODE)
    # RF CURRENT CONTROL MODE
    pos_R = motor_R.get_position()
    data_pos_R += [pos_R]
    set_current_control_mode(motor_R)
    time.sleep(.75)
    # LF NEW TRAJECTORY
    pos_L = motor_L.get_position()
    data_pos_L += [pos_L]
    new_traj = [*range(traj[0], pos_L, 1)]
    time.sleep(.75)
    # LF ANTICLOCKWISE MOTION
    list_pos_LR = singleAct(motor_L, motor_R, new_traj, dt, anticlockwise)
    data_pos_L += list_pos_LR[0]
    data_pos_R += list_pos_LR[1]
    ######## CLOCKWISE MOTION
    # RF POSITION CONTROL MODE
    motor_R.set_operating_mode(motor_R.POSITION_CONTROL_MODE)
    # RF CURRENT CONTROL MODE
    pos_L = motor_L.get_position()
    data_pos_L += [pos_L]
    set_current_control_mode(motor_L)
    time.sleep(.75)
    # RF NEW TRAJECTORY
    pos_R = motor_R.get_position()
    data_pos_R += [pos_R]
    new_traj = [*range(pos_R, traj[-1], 1)]
    time.sleep(.75)
    # RF CLOCKWISE MOTION
    list_pos_RL = singleAct(motor_R, motor_L, new_traj, dt, clockwise)
    data_pos_R += list_pos_RL[0]
    data_pos_L += list_pos_RL[1]
    # LF POSITION CONTROL MODE
    motor_L.set_operating_mode(motor_L.POSITION_CONTROL_MODE)
    time.sleep(.75)
    new_traj_L = [*range(2047, motor_L.get_position(), 1)]
    new_traj_R = [*range(2047, motor_R.get_position(), 1)]
    singleAct(motor_L, motor_R, new_traj_L, dt, anticlockwise)
    singleAct(motor_R, motor_L, new_traj_R, dt, anticlockwise)
    return [data_pos_L, data_pos_R]


#[data_pos_L, data_pos_R] = controlRolling(my_dxl_L, my_dxl_R, lin_traj)

# instantiate an empty dict
"""
filename = "trial obj(square) pos(up) dim(3x3)"
data_pos = {
    filename: {
        "LF": data_pos_L,
        "RF": data_pos_R
    }
}
"""

test = {
    "test": {
        "LF": 0,
        "RF": 1
    }
}

with open("data_pos.json", 'r+') as f:
    # indent=2 is not needed but makes the file human-readable
    data_pos_prev = json.load(f)
    data_pos_prev.update(test)
    f.seek(0)
    json.dump(test, f, indent=2)
"""
plt.figure(1)
plt.plot(data_pos_L,'b--',label=r'Position LF')
plt.plot(data_pos_R,'r--',label=r'Position RF')
plt.xlabel('time(s)')
plt.ylabel('deg')
plt.legend(loc='best')
plt.savefig(filename)"""

# deconnecting
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
XH430.close_port()