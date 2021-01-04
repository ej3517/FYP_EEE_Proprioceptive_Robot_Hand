# Control table ADDress for XH430-W350-R

# EEPROM REGISTER ADDRESSES for XH430
# Permanently stored in memory once changed
ADDR_PRO_OPERATING_MODE     = 11

# RAM REGISTER ADDRESSES
# resets after shut down
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# USER DEFINED
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0