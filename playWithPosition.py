from xh430 import *

# create motor object
my_dxl = XH430(1)

# connecting
XH430.open_port()
XH430.set_baudrate()

# activating torque
my_dxl.enable_torque()

def user_input():
    ans = input("Continue ? y/n : ")
    if ans == "n":
        return False
    else:
        return True


def test_pos(motor_object):
    bool_test = True
    index = 0
    while bool_test:
        motor_object.get_position()
        # desired angle input
        input_pos = int(input("input_pos : "))
        motor_object.set_position(input_pos)
        bool_test = user_input()

test_pos(my_dxl)

# deconnecting
my_dxl.disable_torque()
XH430.close_port()
