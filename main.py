#!/usr/bin/env python3
"""
Example for the XL430 Wrapper usage.

This prints some basic information's about the connected Dynamixel's.
After that it will check if the torque is enabled, if the torque is enabled,
the torque will be disabled. Then it will be checked if the correct operating mode is set
(GOAL_VELOCITY). If not, the drive mode will be changed. Then the Torque will be enabled, also
the built in LED's. Then the Dynamixel's rotate with Velocity 50 for 3 seconds, than the dynamixel's
will be stopped and the LED's disabled. Finally Torque will be disabled and the connection closed.
"""

from XL430 import *

__author__ = "Vinzenz Weist"
__copyright__ = "Copyright 2020, Vinzenz Weist"
__license__ = "GPLv3"
__version__ = "1.0.0"

DXL1_ID = 1
DXL2_ID = 2

dynamixel = XL430()
dynamixel.start()
print("Port Opened")

print("Firmware Version:", dynamixel.get_firmware_version(DXL1_ID))
print("Firmware Version:", dynamixel.get_firmware_version(DXL2_ID))
print("Model Number:", dynamixel.get_model_number(DXL1_ID))
print("Model Number:", dynamixel.get_model_number(DXL2_ID))
print("Temperature DXL_1:", dynamixel.get_present_temperature(DXL1_ID))
print("Temperature DXL_2:", dynamixel.get_present_temperature(DXL2_ID))
print("Position DXL_1:", dynamixel.get_present_position(DXL1_ID))
print("Position DXL_2:", dynamixel.get_present_position(DXL2_ID))
print("Voltage DXL_1:", dynamixel.get_present_input_voltage(DXL1_ID))
print("Voltage DXL_2:", dynamixel.get_present_input_voltage(DXL2_ID))

if not dynamixel.get_torque(DXL1_ID) or not dynamixel.get_torque(DXL2_ID):
    print("Torque is disabled")
    if dynamixel.get_operating_mode(DXL1_ID) != XL_CONTROL_MODE_VELOCITY or dynamixel.get_operating_mode(DXL2_ID) != XL_CONTROL_MODE_VELOCITY:
        print("Drive Mode changed to: VELOCITY")
        dynamixel.set_operating_mode(DXL1_ID, XL_CONTROL_MODE_VELOCITY)
        dynamixel.set_operating_mode(DXL2_ID, XL_CONTROL_MODE_VELOCITY)
else:
    print("Torque was enabled, now disabled")
    dynamixel.set_torque(DXL1_ID, XL_TORQUE_DISABLED)
    dynamixel.set_torque(DXL2_ID, XL_TORQUE_DISABLED)
    if dynamixel.get_operating_mode(DXL1_ID) != XL_CONTROL_MODE_VELOCITY or dynamixel.get_operating_mode(DXL2_ID) != XL_CONTROL_MODE_VELOCITY:
        print("Drive Mode changed to: VELOCITY")
        dynamixel.set_operating_mode(DXL1_ID, XL_CONTROL_MODE_VELOCITY)
        dynamixel.set_operating_mode(DXL2_ID, XL_CONTROL_MODE_VELOCITY)

dynamixel.set_torque(DXL1_ID, XL_TORQUE_ENABLED)
dynamixel.set_torque(DXL2_ID, XL_TORQUE_ENABLED)
print("Torque Enabled")

dynamixel.set_led(DXL1_ID, XL_LED_ENABLED)
dynamixel.set_led(DXL2_ID, XL_LED_ENABLED)
print("LED's Enabled")

print("Drive Servos!")
dynamixel.set_goal_velocity_group(DXL1_ID, DXL2_ID, 50, -50)

time.sleep(3)

dynamixel.set_goal_velocity_group(DXL1_ID, DXL2_ID, 0, 0)
print("Stop Servos!")

dynamixel.set_led(DXL1_ID, XL_LED_DISABLED)
dynamixel.set_led(DXL2_ID, XL_LED_DISABLED)
print("LED's Disabled")

time.sleep(0.5)

dynamixel.set_torque(DXL1_ID, XL_TORQUE_DISABLED)
dynamixel.set_torque(DXL2_ID, XL_TORQUE_DISABLED)
print("Torque Disabled")

dynamixel.stop()
print("Port Closed")
