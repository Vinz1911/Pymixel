from pymixel import Dynamixel
import time

# the dynamixel id's
DXL_ID_ONE = 1
DXL_ID_TWO = 2
# control mode
VELOCITY_MODE = 1

# create instance of dynamixel
dynamixel = Dynamixel()
# open the serial connection
dynamixel.open()
# set the operating mode of the dynamixel
dynamixel.set_operating_mode(DXL_ID_ONE, VELOCITY_MODE)
dynamixel.set_operating_mode(DXL_ID_TWO, VELOCITY_MODE)
# enable torque
dynamixel.set_torque(DXL_ID_ONE, True)
dynamixel.set_torque(DXL_ID_TWO, True)
# enable led's
dynamixel.set_led(DXL_ID_ONE, True)
dynamixel.set_led(DXL_ID_TWO, True)
# let move the dynamixel
dynamixel.set_group_goal_velocity(DXL_ID_ONE, DXL_ID_TWO, 100, 100)
# wait a second
time.sleep(1.0)
# stop moving
dynamixel.set_group_goal_velocity(DXL_ID_ONE, DXL_ID_TWO, 0, 0)
# disable torque
dynamixel.set_torque(DXL_ID_ONE, False)
dynamixel.set_torque(DXL_ID_TWO, False)
# disable led's
dynamixel.set_led(DXL_ID_ONE, False)
dynamixel.set_led(DXL_ID_TWO, False)
