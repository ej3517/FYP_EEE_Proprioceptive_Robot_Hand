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

    DXL_MINIMUM_POSITION_VALUE = 10  # Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE = 4000  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

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

    def set_register4(self, reg_num, reg_value):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = XH430.packetHandler.write4ByteTxRx(XH430.portHandler, self.id, reg_num, reg_value)
        XH430.check_error(dxl_comm_result, dxl_error)

    def get_register1(self, reg_num):
        reg_data, dxl_comm_result, dxl_error = XH430.packetHandler.read1ByteTxRx(XH430.portHandler, self.id, reg_num)
        XH430.check_error(dxl_comm_result, dxl_error)
        return reg_data

    def get_register4(self, reg_num_low):
        reg_data, dxl_comm_result, dxl_error = XH430.packetHandler.read4ByteTxRx(XH430.portHandler, self.id, reg_num_low)
        XH430.check_error(dxl_comm_result, dxl_error)
        return reg_data

    def enable_torque(self):
        """enable torque for motor"""
        self.set_register1(ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        print(self.get_register1(ADDR_PRO_TORQUE_ENABLE))

    def disable_torque(self):
        """disable torque for motor"""
        self.set_register1(ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        print(self.get_register1(ADDR_PRO_TORQUE_ENABLE))

    def set_position(self, dxl_goal_position):
        """write goal position"""
        self.set_register4(ADDR_PRO_GOAL_POSITION, dxl_goal_position)
        print("Position of dxl ID: %d set to %d " % (self.id, dxl_goal_position))

    def get_position(self):
        """Read present position"""
        dxl_present_position = self.get_register4(ADDR_PRO_PRESENT_POSITION)
        print("[ID:%03d] PresPos:%03d" % (self.id, dxl_present_position))
        return dxl_present_position

    @staticmethod
    def check_error(comm_result, dxl_err):
        if comm_result != COMM_SUCCESS:
            print("%s" % XH430.packetHandler.getTxRxResult(comm_result))
        elif dxl_err != 0:
            print("%s" % XH430.packetHandler.getRxPacketError(dxl_err))