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

# make sure the motors are deactivated
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


def set_ang(motor_L, motor_R, ang):
    motor_L.set_operating_mode(motor_L.POSITION_CONTROL_MODE)
    motor_R.set_operating_mode(motor_R.POSITION_CONTROL_MODE)
    motor_L.set_position(ang)
    motor_R.set_position(ang)


set_ang(my_dxl_L, my_dxl_R, deg2pos(90.0))

# deconnecting
my_dxl_L.disable_torque()
my_dxl_R.disable_torque()
XH430.close_port()