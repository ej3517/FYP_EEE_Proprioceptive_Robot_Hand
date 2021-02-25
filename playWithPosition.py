from xh430 import *
import time
import matplotlib.pyplot as plt

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
        start_time = time.perf_counter()
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

def single_act(motor_object):
    bool_test = True
    while bool_test:
        #FOR PLOTTING
        motor_pos_ref = []
        motor_pos = []
        motor_vel = []
        exp_time = []
        motor_object.get_position()
        # desired angle input
        input_angle = float(input("input_angle : "))
        input_pos = deg2pos(input_angle)
        input_pos = motor_object.set_position(input_pos)
        # start of the motion
        start_time = time.perf_counter()
        # read present position
        while 1:
            present_pos = motor_object.get_position()
            present_vel = motor_object.get_velocity()
            present_time = time.perf_counter() - start_time
            motor_pos_ref += [input_pos]
            motor_pos += [present_pos]
            motor_vel += [[present_vel]]
            exp_time += [present_time]
            if not abs(input_pos - present_pos) > motor_object.DXL_MOVING_STATUS_THRESHOLD:
                break
        ### plotting ###
        plt.figure()
        plt.plot(exp_time, motor_pos)
        plt.title("Position trajectory")
        plt.show()
        plt.figure()
        plt.plot(exp_time, motor_vel)
        plt.title("Velocity trajectory")
        plt.show()
        # asking user for new input
        bool_test = user_input()


def test_torque(motor_object):
    # initialize the position
    motor_object.set_operating_mode(motor_object.POSITION_CONTROL_MODE)
    motor_object.enable_torque()
    # desired angle input
    input_angle = float(input("input_angle : "))
    input_pos = deg2pos(input_angle)
    input_pos = motor_object.set_position(input_pos)
    while 1:
        present_pos = motor_object.get_position()
        if not (abs(input_pos - present_pos) > motor_object.DXL_MOVING_STATUS_THRESHOLD):
            break
    # set to current control mode
    motor_object.set_operating_mode(motor_object.CURRENT_CONTROL_MODE)
    motor_object.enable_torque()
    # counter
    u = 0
    # desired angle input
    input_current = int(input("input current : "))
    # read present current
    motor_object.set_current(input_current)
    while u < 30:
        motor_object.get_current()
        print("this is the input current", input_current)
        present_position = motor_object.get_position()
        if present_position > 3071 or present_position < 1024:
            motor_object.disable_torque()
            print(u)
            break
        else:
            u += 1
    motor_object.disable_torque()
    motor_object.set_operating_mode(motor_object.POSITION_CONTROL_MODE)


single_act(my_dxl_R)

# deconnecting
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
XH430.close_port()
