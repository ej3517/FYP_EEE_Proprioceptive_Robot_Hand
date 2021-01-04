from xh430 import *
import time
import matplotlib.pyplot as plt

# create motor object
my_dxl_L = XH430(1)
my_dxl_R = XH430(2)

# connecting
XH430.open_port()
XH430.set_baudrate()

# activating torque
my_dxl_L.enable_torque()
my_dxl_R.enable_torque()


def user_input():
    ans = input("Continue ? y/n : ")
    if ans == "n":
        return False
    else:
        return True


def get_time(start_time):
    current_time = time.time() - start_time
    return current_time


def deg2pos(angle):
    return int(4095.0/360*angle + 4095.0/4)


def test_pos(motor_object_L, motor_object_R):
    bool_test = True
    while bool_test:
        """ FOR PLOTTING
        motor_pos_ref = []
        motor_pos = []
        exp_time = [] """
        # left actuator
        motor_object_L.get_position()
        # right actuator
        motor_object_R.get_position()
        # desired angle input
        input_angle = float(input("input_angle : "))
        input_pos = deg2pos(input_angle)
        input_pos_L = motor_object_L.set_position(input_pos)
        input_pos_R = motor_object_R.set_position(input_pos)
        # start of the motion
        start_time = time.time()
        # read present position
        while 1:
            # left actuator
            present_pos_L = motor_object_L.get_position()
            # right actuator
            present_pos_R = motor_object_R.get_position()
            """ FOR PLOTTING
            present_time = get_time(start_time)
            # evolution
            motor_pos_ref += [input_pos]
            motor_pos += [present_pos]
            exp_time += [present_time]"""
            if not (abs(input_pos_L - present_pos_L) > motor_object_L.DXL_MOVING_STATUS_THRESHOLD
                    and abs(input_pos_R - present_pos_R) > motor_object_R.DXL_MOVING_STATUS_THRESHOLD):
                break
        # asking user for new input
        bool_test = user_input()
        # plotting
        """ FOR PLOTTING
        plt.figure()
        plt.plot(exp_time, motor_pos_ref, linewidth=3, color='r')
        plt.plot(exp_time, motor_pos, markersize=3)
        plt.title('Evolution of the Actuator position')
        plt.show() """


test_pos(my_dxl_L, my_dxl_R)

# deconnecting
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
XH430.close_port()
