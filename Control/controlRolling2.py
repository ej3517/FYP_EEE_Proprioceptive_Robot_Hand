from xh430 import *
import time
import matplotlib.pyplot as plt
import numpy as np
import json
import sys
import os


testfile =  open("test_file.txt", "a")

################ MOTORS #################

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

# when you need to stop the motors


def stop_motors():
    my_dxl_L.disable_torque()
    my_dxl_R.disable_torque()
    XH430.close_port()


# CLOCK SET UP
def get_time(start_time):
    current_time = time.time() - start_time
    return current_time


# CONVERTER ANGLE TO POSITION
def deg2pos(angle):
    return int(4095.0/360*angle + 4095.0/4)


# Linear Trajectory to follow
lin_traj = [deg2pos(60), deg2pos(120)]  # from 70deg to 110deg

def motionGradient(traj, dt):
    return abs(traj[0]-traj[-1])/dt

def set_current_control_mode(motor):
    motor.set_operating_mode(motor.CURRENT_CONTROL_MODE)
    if motor.id == 1:
        motor.set_current(40)   # low torque
    else:
        motor.set_current(-60)  # low torque

def reboot():
    my_dxl_L.set_operating_mode(my_dxl_L.POSITION_CONTROL_MODE)
    my_dxl_R.set_operating_mode(my_dxl_R.POSITION_CONTROL_MODE)
    ang = deg2pos(90.0)
    my_dxl_L.set_position(ang)
    my_dxl_R.set_position(ang)

# Condition to continue program
def user_input():
    while True:
        ans = input("Continue ? y/n/reboot : ")
        if ans == "n":
            reboot()
            stop_motors()
            sys.exit("EXIT PROGRAM")
        elif ans == "y":
            break
        elif ans == "reboot":
            reboot()
        else:
            print("This is an invalid answer. Try again ... (y/n) ?")


def singleAct(motor_object, torque_motor, traj, dt, forward, start_time):
    """change position of one actuator with a given speed """
    # INITIALIZATION
    data_pos = [[], []]
    data_pos_torque = [[], []]
    # MOTORS
    if motor_object.get_operating_mode() == motor_object.CURRENT_CONTROL_MODE:
        motor_object.set_operating_mode(motor_object.POSITION_CONTROL_MODE)
    if forward:
        forward_coeff = 1
        first_index = 0
        last_index = -1
    else:
        forward_coeff = -1
        first_index = -1
        last_index = 0
    start_pos = traj[first_index]
    data_pos[0] += [start_pos]
    data_pos[1] += [get_time(start_time)]
    data_pos_torque[0] += [torque_motor.get_position()]
    data_pos_torque[1] += [get_time(start_time)]

    gradient = motionGradient(traj, dt)   # gradient of the motion
    step = 0
    while abs(step) <= dt:
        input_pos = int(gradient*step*forward_coeff + start_pos)
        testfile.write(str(input_pos)+"\n")
        input_pos = motor_object.set_position(input_pos)
        while 1:
            present_pos = motor_object.get_position()
            data_pos[0] += [present_pos]
            data_pos[1] += [get_time(start_time)]
            data_pos_torque[0] += [torque_motor.get_position()]
            data_pos_torque[1] += [get_time(start_time)]
            if not abs(input_pos - present_pos) > motor_object.DXL_MOVING_STATUS_THRESHOLD:
                step += 1
                break
    motor_object.set_position(traj[last_index])
    time.sleep(.75)  # time for the actuator to reach the final position
    print(len(data_pos[0]))
    # do you want to stop the manipulation
    user_input()
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
    user_input()


######### CONTROL FULL MANIPULATION #########

def controlRolling(motor_L, motor_R, traj):
    #### PUT THE FINGERS AT THEIR INITIAL POSITION ####
    # INITIALIZATION DATA
    data_pos_L = [[], []]
    data_pos_R = [[], []]
    dict_pos = {}

    clockwise = True
    anticlockwise = False

    # INITIALIZE THE GRASP
    user_input()
    dt = 50
    graspManip(motor_L, motor_R, traj, dt, clockwise)

    ######## INITIAL MANIPUALTION --> CLOCKWISE MOTION #######
    # SET UP THE SPEED
    dt = 200
    # LF TORQUE MODE
    pos_L = motor_L.get_position()
    set_current_control_mode(motor_L)

    # RF NEW TRAJECTORY
    time.sleep(.75)

    # START THE CLOCK
    start_manip = time.time()

    # RF TRAJECTORY
    pos_R = motor_R.get_position()
    new_traj = [pos_R, traj[-1]]

    # RF POSITION MODE AND START CLOCKWISE MOTION
    list_pos_RL = singleAct(motor_R, motor_L, new_traj, dt, clockwise, start_manip)

    # UPDATE LIST
    data_pos_R[0] += list_pos_RL[0][0]
    data_pos_R[1] += list_pos_RL[0][1]
    data_pos_L[0] += list_pos_RL[1][0]
    data_pos_L[1] += list_pos_RL[1][1]

    ######## FIRST MANIPUALTION --> ANTICLOCKWISE MOTION ########
    # SET UP THE SPEED
    dt = 200
    # LF POSITION CONTROL MODE
    motor_L.set_operating_mode(motor_L.POSITION_CONTROL_MODE)
    # RF CURRENT CONTROL MODE
    set_current_control_mode(motor_R)
    time.sleep(.75)

    # LF NEW TRAJECTORY
    pos_L = motor_L.get_position()
    new_traj = [traj[0], pos_L]
    time.sleep(.75)

    # LF ANTICLOCKWISE MOTION
    list_pos_LR = singleAct(motor_L, motor_R, new_traj, dt, anticlockwise, start_manip)

    data_pos_L[0] += list_pos_LR[0][0]
    data_pos_L[1] += list_pos_LR[0][1]
    data_pos_R[0] += list_pos_LR[1][0]
    data_pos_R[1] += list_pos_LR[1][1]

    dict_pos["LF_motion1"] = [list_pos_LR[0][0], list_pos_LR[0][1]]
    dict_pos["RF_motion1"] = [list_pos_LR[1][0], list_pos_LR[1][1]]

    ######## SECOND MANIPUALTION --> CLOCKWISE MOTION ########
    # RF POSITION CONTROL MODE
    motor_R.set_operating_mode(motor_R.POSITION_CONTROL_MODE)

    # RF CURRENT CONTROL MODE
    set_current_control_mode(motor_L)
    time.sleep(.75)

    # RF NEW TRAJECTORY
    pos_R = motor_R.get_position()
    new_traj = [pos_R, traj[-1]]
    time.sleep(.75)

    # RF CLOCKWISE MOTION
    list_pos_RL = singleAct(motor_R, motor_L, new_traj, dt, clockwise, start_manip)

    data_pos_R[0] += list_pos_RL[0][0]
    data_pos_R[1] += list_pos_RL[0][1]
    data_pos_L[0] += list_pos_RL[1][0]
    data_pos_L[1] += list_pos_RL[1][1]

    dict_pos["LF_motion2"] = [list_pos_RL[1][0], list_pos_RL[1][1]]
    dict_pos["RF_motion2"] = [list_pos_RL[0][0], list_pos_RL[0][1]]

    # LF POSITION CONTROL MODE
    motor_L.set_operating_mode(motor_L.POSITION_CONTROL_MODE)

    ############ GO BACK TO INITIAL POSITION #############
    # SET UP THE SPEED
    dt = 200
    #
    time.sleep(.75)
    new_traj_L = [2047, motor_L.get_position()]
    new_traj_R = [2047, motor_R.get_position()]
    singleAct(motor_L, motor_R, new_traj_L, dt, anticlockwise, start_manip)
    singleAct(motor_R, motor_L, new_traj_R, dt, anticlockwise, start_manip)
    return [data_pos_L, data_pos_R, dict_pos]


def plot_pos_evol(thetaL, thetaR, file):
    """ plotting """
    MEDIUM_SIZE = 20
    BIGGER_SIZE = 30

    plt.rc('font', size=BIGGER_SIZE)         # controls default text sizes
    plt.rc('axes', titlesize=BIGGER_SIZE)    # fontsize of the axes title
    plt.rc('axes', labelsize=BIGGER_SIZE)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=MEDIUM_SIZE)   # fontsize of the tick labels
    plt.rc('ytick', labelsize=MEDIUM_SIZE)   # fontsize of the tick labels
    plt.rc('legend', fontsize=MEDIUM_SIZE)   # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

    fig, ax = plt.subplots()
    plt.style.use("ggplot")
    plt.rcParams["figure.figsize"] = (12, 8)
    time.sleep(1)
    ax.plot(thetaL[1], thetaL[0], label=r'$\theta_L$')
    ax.plot(thetaR[1], thetaR[0], label=r'$\theta_R$')
    ax.legend(loc='upper right')
    plt.title("square rolling "+file)
    plt.ylabel('position(deg)')
    plt.xlabel('time(s)')
    #plt.savefig(file)

####### CALL THE MAIN CONTROL FUNCTION #######

# White lists
sqrt_hex_dim = ["20", "25", "30"]
circle_dim = ["20","25"]
rectangle_dim = ["20_25"]
poses_drf = ["60", "65", "70"]
finger_gaps = ["34", "42", "50"]
shape_list = ["rectangle", "circle", "square", "hexagon"]
trials = ["1", "2"]

# How to store data, new file ?
possibilities = ["new", "push", "n"]

# set up filename and database name
def set_filename():
    filename = ""
    # The shape
    while True:
        print("The shape ?")
        shape = input(shape_list)
        if (shape in  shape_list):
            break
        else:
            print("Oops!  That was no valid shape name.  Try again...")
    # dimension
    if (shape == "rectangle"):
        dim_list = rectangle_dim
    elif (shape == "circle"):
        dim_list = circle_dim
    else:
        dim_list = sqrt_hex_dim
    while True:
        print(("The dimension ?"))
        dim = input (dim_list)
        if (dim in dim_list):
            break
        else:
            print("Oops!  That was no valid dimension.  Try again...")
    # pose
    while True:
        print(("The pose ?"))
        pose = input(poses_drf)
        if (pose in poses_drf):
            break
        else:
            print("Oops!  That was no valid pose.  Try again...")
    # gap
    while True:
        print(("The fingers gap ?"))
        gap = input (finger_gaps)
        if (gap in finger_gaps):
            break
        else:
            print("Oops!  That was no valid fingers gap.  Try again...")
    # trial
    while True:
        print(("The trial ?"))
        trial = input(trials)
        if (trial in trials):
            break
        else:
            print("Oops!  That was no valid trial number.  Try again...")
    # generate the filename
    filename += "dim_"+dim+"_pose_"+pose+"_gap_"+gap+"_"+trial
    return [filename, shape]

def set_databasename(filename, shape, data_pos_trial):
    # STORE THE DATA  INTO A JSON FILE
    data_pos = {
        filename: data_pos_trial
    }

    # In what file do you want to store the data
    databasename = "data_pos_trial_"+shape
    while True:
        ans = input("Do you want to add a number at the end of the database name (y/n) ?")
        if ans == "y":
            while True:
                try:
                    x = int(input("Please enter a number: "))
                    databasename += "_"+str(x)
                    break
                except ValueError:
                    print("Oops!  That was no valid number.  Try again...")
            break
        elif ans == "n":
            break
        else:
            print("Invalid answer. Try again ...")
    databasename += ".json"

    # trial
    while True:
        print("Do you want to create new file, push to existing  database or do nothing ?")
        poss = input(possibilities)
        if poss == "new":
            with open(databasename, 'w') as f:
                indent = 2  # is not needed but makes the file human-readable
                json.dump(data_pos, f, indent=2)
            break
        elif poss == "push":
            with open(databasename, 'r+') as f:
                indent = 2  # is not needed but makes the file human-readable
                data_pos_final = json.load(f)
                data_pos_final.update(data_pos)
                f.seek(0)
                json.dump(data_pos_final, f, indent=2)
            break
        elif poss == "n":
            break
        else:
            print("Oops!  That was not part of the possibilities.  Try again...")


def manipulate(dxl_traj):
    """ perform the manipulation and store the date given the user input"""
    [data_pos_L, data_pos_R, data_pos_trial] = controlRolling(my_dxl_L, my_dxl_R, dxl_traj)

    print(len(data_pos_L[0]))

    while True:
        ans = input ("Do you want to store the data (y/n)?")
        if ans == "y":
            [filename, shape] = set_filename()
            set_databasename(filename, shape, data_pos_trial)
            ####### PLOT THE RESULTS #######
            plot_pos_evol(data_pos_L, data_pos_R, filename)
            plot_pos_evol(data_pos_L, data_pos_R, filename)
            break
        elif ans == "n":
            break
        else:
            print("invalid answer (y/n). Try again")


# execute the function
manipulate(lin_traj)

################ DECONNECTING #################

stop_motors()

testfile.close()

