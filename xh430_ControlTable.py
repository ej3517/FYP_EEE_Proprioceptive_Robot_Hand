# Control table ADDress for XH430-W350-R

# EEPROM REGISTER ADDRESSES for XH430
# Permanently stored in memory once changed # the torque must be disabled to write
ADDR_PRO_OPER_MODE     = 11
ADDR_PRO_CURRENT_LIMIT = 38
ADDR_PRO_MAX_POS_LIMIT = 48
ADDR_PRO_MIN_POS_LIMIT = 52

# RAM REGISTER ADDRESSES
# resets after shut down, Control table address is different in Dynamixel model
ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_GOAL_CURRENT       = 102
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_CURRENT    = 126
ADDR_PRO_PRESENT_POSITION   = 132

# USER DEFINED
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
