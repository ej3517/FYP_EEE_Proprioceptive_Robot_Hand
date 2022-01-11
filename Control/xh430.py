# import modules
import os
from dynamixel_sdk import *
from xh430_ControlTable import *


class XH430:
    """class for dynamixel XH430 motors"""
    # Protocol version
    PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel
    # Default setting
    BAUDRATE = 57600  # Dynamixel default baudrate : 57600
    DEVICENAME = '/dev/cu.usbserial-FT4TFUMI'  # Check which port is being used on your controller
    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    DXL_MINIMUM_POSITION_VALUE = 1324 #1535  # Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE = 2771 #2560  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    DXL_MOVING_STATUS_THRESHOLD = 10  # Dynamixel moving status threshold # the initial value was 20

    # Different operating mode
    CURRENT_CONTROL_MODE = 0
    POSITION_CONTROL_MODE = 3
    CURRENT_BASED_POSITION_CONTROL_MODE = 5

    @classmethod
    def open_port(cls):
        # Open port
        if cls.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()

    @classmethod
    def set_baudrate(cls):
        # Set port baudrate
        if cls.portHandler.setBaudRate(cls.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

    @classmethod
    def close_port(cls):
        # Close port
        cls.portHandler.closePort()
        print("successfully closed port")

    def __init__(self, motor_id):
        """Initialize motor ID"""
        self.id = motor_id

    def set_register1(self, reg_num, reg_value):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = XH430.packetHandler.write1ByteTxRx(XH430.portHandler, self.id, reg_num, reg_value)
        XH430.check_error(dxl_comm_result, dxl_error)

    def set_register2(self, reg_num, reg_value):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = XH430.packetHandler.write2ByteTxRx(XH430.portHandler, self.id, reg_num, reg_value)
        XH430.check_error(dxl_comm_result, dxl_error)

    def set_register4(self, reg_num, reg_value):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = XH430.packetHandler.write4ByteTxRx(XH430.portHandler, self.id, reg_num, reg_value)
        XH430.check_error(dxl_comm_result, dxl_error)

    def get_register1(self, reg_num):
        reg_data, dxl_comm_result, dxl_error = XH430.packetHandler.read1ByteTxRx(XH430.portHandler, self.id, reg_num)
        XH430.check_error(dxl_comm_result, dxl_error)
        return reg_data

    def get_register2(self, reg_num):
        reg_data, dxl_comm_result, dxl_error = XH430.packetHandler.read2ByteTxRx(XH430.portHandler, self.id, reg_num)
        XH430.check_error(dxl_comm_result, dxl_error)
        return reg_data

    def get_register4(self, reg_num_low):
        reg_data, dxl_comm_result, dxl_error = XH430.packetHandler.read4ByteTxRx(XH430.portHandler, self.id, reg_num_low)
        XH430.check_error(dxl_comm_result, dxl_error)
        return reg_data

    def enable_torque(self):
        """enable torque for motor"""
        self.set_register1(ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        print("the torque has been enabled : ", self.get_register1(ADDR_PRO_TORQUE_ENABLE))

    def disable_torque(self):
        """disable torque for motor"""
        self.set_register1(ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        print("the torque has been disabled : ", self.get_register1(ADDR_PRO_TORQUE_ENABLE))

    def set_operating_mode(self, operating_mode):
        self.disable_torque()
        self.set_register1(ADDR_PRO_OPER_MODE, operating_mode)
        self.enable_torque()
        print("[ID:%03d] Operating Mode set to %03d" % (self.id, operating_mode))

    def get_operating_mode(self):
        dxl_operating_mode = self.get_register1(ADDR_PRO_OPER_MODE)
        print("[ID:%03d] Operating Mode: %03d" % (self.id, dxl_operating_mode))

    def set_position_limit(self):
        self.set_register4(ADDR_PRO_MAX_POS_LIMIT, XH430.DXL_MAXIMUM_POSITION_VALUE)
        self.set_register4(ADDR_PRO_MIN_POS_LIMIT, XH430.DXL_MINIMUM_POSITION_VALUE)
        print("[ID:%03d] MaxPos:%03d" % (self.id, self.get_register4(ADDR_PRO_MAX_POS_LIMIT)))
        print("[ID:%03d] MaxPos:%03d" % (self.id, self.get_register4(ADDR_PRO_MIN_POS_LIMIT)))

    def set_position(self, dxl_goal_position):
        """write goal position and check if the goal position is not out of bound"""
        if dxl_goal_position > XH430.DXL_MAXIMUM_POSITION_VALUE:
            self.set_register4(ADDR_PRO_GOAL_POSITION, XH430.DXL_MAXIMUM_POSITION_VALUE)
            print("Goal position out of bound so Position of dxl ID: %d set to %d " % (self.id, XH430.DXL_MAXIMUM_POSITION_VALUE))
            return XH430.DXL_MAXIMUM_POSITION_VALUE
        elif dxl_goal_position < XH430.DXL_MINIMUM_POSITION_VALUE:
            self.set_register4(ADDR_PRO_GOAL_POSITION, XH430.DXL_MINIMUM_POSITION_VALUE)
            print("Goal position out of bound so Position of dxl ID: %d set to %d " % (self.id, XH430.DXL_MINIMUM_POSITION_VALUE))
            return XH430.DXL_MINIMUM_POSITION_VALUE
        else:
            self.set_register4(ADDR_PRO_GOAL_POSITION, dxl_goal_position)
            print("Position of dxl ID: %d set to %d " % (self.id, dxl_goal_position))
            return dxl_goal_position

    def get_position(self):
        """Read present position"""
        dxl_present_position = self.get_register4(ADDR_PRO_PRESENT_POSITION)
        print("[ID:%03d] PresPos:%03d" % (self.id, dxl_present_position))
        return dxl_present_position

    def get_velocity(self):
        """Read present velocity"""
        dxl_present_velocity = self.get_register4(ADDR_PRO_PRESENT_VELOCITY)
        print("[ID:%03d] PresVel:%03d" % (self.id, dxl_present_velocity))
        return dxl_present_velocity

    def get_current(self):
        """Read present current"""
        dxl_present_current = self.get_register2(ADDR_PRO_PRESENT_CURRENT)
        dxl_current_limit = self.get_register2(ADDR_PRO_CURRENT_LIMIT)
        # We want the signed version of the present current
        if dxl_present_current >= 0x8000:
            dxl_present_current -= 0x10000
        print("[ID:%03d] PresCurrent:%03d" % (self.id, dxl_present_current))
        #print("[ID:%03d] CurrentLim:%03d" % (self.id, dxl_current_limit))
        return dxl_present_current

    def set_current(self, dxl_goal_current):
        """write goal current and check if the goal current is not out of bound"""
        self.set_register2(ADDR_PRO_GOAL_CURRENT, dxl_goal_current)
        print("Current of dxl ID: %d set to %d " % (self.id, dxl_goal_current))

    @staticmethod
    def check_error(comm_result, dxl_err):
        if comm_result != COMM_SUCCESS:
            print("%s" % XH430.packetHandler.getTxRxResult(comm_result))
        elif dxl_err != 0:
            print("%s" % XH430.packetHandler.getRxPacketError(dxl_err))