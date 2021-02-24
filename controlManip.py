from xh430 import *
import time
import matplotlib.pyplot as plt

# create motor object
my_dxl_L = XH430(1) # left actuator when placed on the frame
my_dxl_R = XH430(2) # right actuator when placed on the frame

# connecting
XH430.open_port()
XH430.set_baudrate()

# make sure the torque is disabled to WRITE on the EEPROM registers
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
# setting the position limit
my_dxl_L.set_position_limit()
my_dxl_R.set_position_limit()


def user_input():
    ans = input("Continue ? y/n : ")
    if ans == "n":
        return False
    else:
        return True


def deg2pos(angle):
    return int(4095.0/360*angle + 4095.0/4)


def check_param_change(motor_object):
    pres_current = motor_object.get_current()
    while 1:
        if abs(motor_object.get_current() - pres_current) > 5:
            print("this the present current ", motor_object.get_current())
            break

############ POSITION CONTROL MODE ###############

def control_position_speed_1F(motor_object, angle):
    """position a finger at an angle with a certain speed"""
    motor_object.set_operating_mode(motor_object.POSITION_CONTROL_MODE)
    final_pos = deg2pos(angle)
    # initial finger position
    pos = motor_object.get_position()
    # speed set up
    motion_range = final_pos - pos
    speed = 5 # based on experiment this multiplicative coefficient seems to set the optimal motion speed
    step = int(speed*motion_range/abs(motion_range))
    while abs(pos - final_pos) > motor_object.DXL_MOVING_STATUS_THRESHOLD:
        pos = motor_object.set_position(pos + step)
        # read present position
        while 1:
            # left actuator
            present_pos = motor_object.get_position()
            if not (abs(pos - present_pos) > motor_object.DXL_MOVING_STATUS_THRESHOLD):
                break

def control_position_speed_2F(motor_object_L, motor_object_R, angle_L, angle_R):
    """move both fingers at the same time"""
    final_pos_R = deg2pos(float(80))
    final_pos_L = deg2pos(float(100))
    step = 10
    mem_set_pos_R = 0  # store the previous set position for right finger
    mem_set_pos_L = 0  # store the previous set position for left finger
    while (abs(present_pos_R - final_pos_R) > motor_object_R.DXL_MOVING_STATUS_THRESHOLD
           and abs(present_pos_L - final_pos_L) > motor_object_L.DXL_MOVING_STATUS_THRESHOLD):
        set_pos_R = motor_object_R.set_position(present_pos_R - step)
        set_pos_L = motor_object_L.set_position(present_pos_L + step)
        # does the set position has changed since the last time
        if (set_pos_R == mem_set_pos_R or set_pos_L == mem_set_pos_L):
            break
        mem_set_pos_R = present_pos_R - step
        mem_set_pos_L = present_pos_L - step
        while 1:
            present_pos_R = motor_object_R.get_position()
            present_pos_L = motor_object_L.get_position()
            if not (abs(set_pos_R - present_pos_R) > motor_object_R.DXL_MOVING_STATUS_THRESHOLD):
                break
            elif not (abs(set_pos_L - present_pos_L) > motor_object_L.DXL_MOVING_STATUS_THRESHOLD):
                break


def test_finger(motor_object):
    bool_test = True
    while bool_test:
        input_angle = float(input("input_angle : "))
        control_position_speed_1F(motor_object, input_angle)
        bool_test = user_input()


def initialize_grasp(motor_object_L, motor_object_R):
    """control the initialization of the grasp with an object"""
    motor_object_L.set_operating_mode(motor_object_L.POSITION_CONTROL_MODE)
    motor_object_R.set_operating_mode(motor_object_R.POSITION_CONTROL_MODE)
    input_pos_L = motor_object_L.set_position(deg2pos(float(70)))
    input_pos_R = motor_object_R.set_position(deg2pos(float(110)))
    while 1:
        present_pos_L = motor_object_L.get_position()
        present_pos_R = motor_object_R.get_position()
        if not (abs(input_pos_L - present_pos_L) > motor_object_L.DXL_MOVING_STATUS_THRESHOLD
                and abs(input_pos_R - present_pos_R) > motor_object_R.DXL_MOVING_STATUS_THRESHOLD):
            break
    if user_input():
        final_pos_R = deg2pos(float(80))
        final_pos_L = deg2pos(float(100))
        step = 10
        mem_set_pos_R = 0  # store the previous set position for right finger
        mem_set_pos_L = 0  # store the previous set position for left finger
        while (abs(present_pos_R - final_pos_R) > motor_object_R.DXL_MOVING_STATUS_THRESHOLD
                and abs(present_pos_L - final_pos_L) > motor_object_L.DXL_MOVING_STATUS_THRESHOLD):
            set_pos_R = motor_object_R.set_position(present_pos_R - step)
            set_pos_L = motor_object_L.set_position(present_pos_L + step)
            # does the set position has changed since the last time
            if (set_pos_R == mem_set_pos_R or set_pos_L == mem_set_pos_L):
                break
            mem_set_pos_R = present_pos_R - step
            mem_set_pos_L = present_pos_L - step
            while 1:
                present_pos_R = motor_object_R.get_position()
                present_pos_L = motor_object_L.get_position()
                if not (abs(set_pos_R - present_pos_R) > motor_object_R.DXL_MOVING_STATUS_THRESHOLD):
                    break
                elif not (abs(set_pos_L - present_pos_L) > motor_object_L.DXL_MOVING_STATUS_THRESHOLD):
                    break


def control_rolling(motor_object_l, motor_object_r):
    # initialize the grasp
    initialize_grasp(motor_object_l, motor_object_r)
    # start the rolling
    control_motion(motor_object_l, motor_object_r)


def control_motion(motor_object_t, motor_object_p):
    bool_test = True
    while bool_test:
        # torque/current control mode actuator set with desired angle input
        motor_object_t.set_operating_mode(motor_object_t.CURRENT_CONTROL_MODE)
        if motor_object_t.id == 1:
            motor_object_t.set_current(10)   # low torque
        else:
            motor_object_t.set_current(-10)  # low torque
        # position control mode actuator set with desired angle input
        motor_object_p.set_operating_mode(motor_object_p.POSITION_CONTROL_MODE)
        input_angle = float(input("input_angle : "))
        control_position_speed_1F(motor_object_p, input_angle)
        motor_object_t.disable_torque()
        print("finished")
        bool_test = user_input()


control_rolling(my_dxl_L, my_dxl_R)

#test_finger(my_dxl_R)

# deconnecting
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
XH430.close_port()