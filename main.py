from XL430 import *

DXL_ID_ONE = 1
DXL_ID_TWO = 2

dynamixel = XL430()
dynamixel.start()

dynamixel.set_torque(DXL_ID_ONE, True)
dynamixel.set_torque(DXL_ID_TWO, True)

dynamixel.set_led(DXL_ID_ONE, True)
dynamixel.set_led(DXL_ID_TWO, True)

print("Temperature DXL_1:", dynamixel.get_present_temperature(DXL_ID_ONE))
print("Temperature DXL_2:", dynamixel.get_present_temperature(DXL_ID_TWO))

print("Position DXL_1:", dynamixel.get_present_position(DXL_ID_ONE))
print("Position DXL_2:", dynamixel.get_present_position(DXL_ID_TWO))

dynamixel.set_sync_goal_velocity(DXL_ID_ONE, DXL_ID_TWO, 20, 20)

time.sleep(2)

dynamixel.set_led(DXL_ID_ONE, False)
dynamixel.set_led(DXL_ID_TWO, False)

dynamixel.set_sync_goal_velocity(DXL_ID_ONE, DXL_ID_TWO, 0, 0)

print("Disabled Motors")

dynamixel.set_torque(DXL_ID_ONE, False)
dynamixel.set_torque(DXL_ID_TWO, False)

print("Torque Disabled")

dynamixel.stop()
