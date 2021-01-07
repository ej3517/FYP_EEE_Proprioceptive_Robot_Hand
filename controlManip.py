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
# set operating mode to current-based position control mode
my_dxl_L.set_operating_mode(my_dxl_L.POSITION_CONTROL_MODE)
my_dxl_R.set_operating_mode(my_dxl_R.CURRENT_CONTROL_MODE)
my_dxl_L.get_operating_mode()
my_dxl_R.get_operating_mode()
# activating torque
my_dxl_L.enable_torque()
my_dxl_R.enable_torque()


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


def control_alg(motor_object_t, motor_object_p):
    bool_test = True
    while bool_test:
        # Initialize the grasp
        # Operating mode
        motor_object_t.set_operating_mode(motor_object_t.CURRENT_CONTROL_MODE)
        motor_object_t.enable_torque()
        if motor_object_t.id == 1:
            motor_object_t.set_current(70)   # low torque
        else:
            motor_object_t.set_current(-10)  # low torque
        # position control mode actuator set with desired angle input
        # Operating mode
        motor_object_p.set_operating_mode(motor_object_p.POSITION_CONTROL_MODE)
        motor_object_p.enable_torque()
        input_angle = float(input("input_angle : "))
        input_pos = deg2pos(input_angle)
        input_pos_p = motor_object_p.set_position(input_pos)
        while 1:
            # check if the motion is finished
            present_pos_p = motor_object_p.get_position()
            if not (abs(input_pos_p - present_pos_p) > motor_object_p.DXL_MOVING_STATUS_THRESHOLD):
                # stop the actuator in current control mode
                motor_object_t.disable_torque()
                motor_object_t.enable_torque()
                break
        # reinitialize angle to 90
        motor_object_t.set_operating_mode(motor_object_t.POSITION_CONTROL_MODE)
        motor_object_t.enable_torque()
        motor_object_t.set_position(2048)
        motor_object_p.set_position(2048)

        # asking if user want to continue
        bool_test = user_input()

control_alg(my_dxl_L, my_dxl_R)

# deconnecting
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
XH430.close_port()