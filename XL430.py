#!/usr/bin/env python3
"""XL430 is a handsome wrapper around the DynamixelSDK.

This makes it much easier to deal with Dynamixel's XL430.
It contains easy to use functions for connecting and position control,
also for get some relevant control data from it
"""

from dynamixel_sdk import *
from XL430_ADDRESS_TABLE import *

__author__ = "Vinzenz Weist"
__copyright__ = "Copyright 2020, Vinzenz Weist"
__license__ = "GPLv3"
__version__ = "1.0.0"


class XL430:
    """Handsome class to control Dynamixel XL430 Servos"""
    PROTOCOL_VERSION = 2.0  # Dynamixel Protocol Version
    BAUDRATE = 1_000_000  # Default Baud Rate
    DEVICENAME = "/dev/ttyACM0"  # Device Connection Path

    # init class
    def __init__(self):
        self.__port_handler = PortHandler(self.DEVICENAME)
        self.__packet_handler = PacketHandler(self.PROTOCOL_VERSION)
        self.__group_bulk_write = GroupBulkWrite(self.__port_handler, self.__packet_handler)

    # open ttl port and set baudrate for communication
    # with the dynamixel servo motors
    def start(self):
        if not self.__port_handler.openPort():
            print("failed: port not opened")
        if not self.__port_handler.setBaudRate(self.BAUDRATE):
            print("failed: baudrate not set")

    # terminate the connection
    # close the communication channel
    def stop(self):
        self.__port_handler.closePort()

    # reboot dynamixel
    def reboot(self, dxl_id):
        dxl_result, dxl_error = self.__packet_handler.reboot(self.__port_handler, dxl_id)
        self.__validate_write(dxl_result, dxl_error)

    # get torque status
    def get_torque(self, dxl_id):
        dxl_torque = self.__read_register1(dxl_id, ADDR_XL_TORQUE)
        return dxl_torque

    # get led status
    def get_led(self, dxl_id):
        dxl_led = self.__read_register1(dxl_id, ADDR_XL_LED)
        return dxl_led

    # get drive mode
    def get_drive_mode(self, dxl_id):
        dxl_drive_mode = self.__read_register1(dxl_id, ADDR_XL_DRIVE_MODE)
        return dxl_drive_mode

    def get_operating_mode(self, dxl_id):
        dxl_operating_mode = self.__read_register1(dxl_id, ADDR_XL_OPERATING_MODE)
        return dxl_operating_mode

    # get model number
    def get_model_number(self, dxl_id):
        dxl_model_number = self.__read_register2(dxl_id, ADDR_XL_MODEL_NUMBER)
        return dxl_model_number

    # get firmware version
    def get_firmware_version(self, dxl_id):
        dxl_firmware_version = self.__read_register1(dxl_id, ADDR_XL_FIRMWARE_VERSION)
        return dxl_firmware_version

    # get protocol type
    def get_protocol_type(self, dxl_id):
        dxl_protocol_type = self.__read_register1(dxl_id, ADDR_XL_PROTOCOL_TYPE)
        return dxl_protocol_type

    # get current servo temperature
    def get_present_temperature(self, dxl_id):
        dxl_temperature = self.__read_register1(dxl_id, ADDR_XL_PRESENT_TEMPERATURE)
        return dxl_temperature

    # get current pwm
    def get_present_pwm(self, dxl_id):
        dxl_pwm = self.__read_register4(dxl_id, ADDR_XL_PRESENT_PWM)
        return dxl_pwm

    # get present load
    def get_present_load(self, dxl_id):
        dxl_load = self.__read_register2(dxl_id, ADDR_XL_PRESENT_LOAD)
        return dxl_load

    # get present input voltage
    def get_present_input_voltage(self, dxl_id):
        dxl_input_voltage = self.__read_register2(dxl_id, ADDR_XL_PRESENT_INPUT_VOLTAGE)
        return dxl_input_voltage

    # get current servo velocity
    def get_present_velocity(self, dxl_id):
        dxl_velocity = self.__read_register4(dxl_id, ADDR_XL_PRESENT_VELOCITY)
        return dxl_velocity

    # get current servo position
    def get_present_position(self, dxl_id):
        dxl_position = self.__read_register4(dxl_id, ADDR_XL_PRESENT_POSITION)
        return dxl_position

    # get real time tick
    def get_realtime_tick(self, dxl_id):
        dxl_realtime_tick = self.__read_register2(dxl_id, ADDR_XL_REALTIME_TICK)
        return dxl_realtime_tick

    # set torque on servo
    def set_torque(self, dxl_id, torque_state):
        self.__write_register1(dxl_id, ADDR_XL_TORQUE, torque_state)

    # set led on servo
    def set_led(self, dxl_id, led_state):
        self.__write_register1(dxl_id, ADDR_XL_LED, led_state)

    # set servo operating mode
    def set_operating_mode(self, dxl_id, control_mode):
        self.__write_register1(dxl_id, ADDR_XL_OPERATING_MODE, control_mode)

    # set drive mode
    def set_drive_mode(self, dxl_id, drive_mode):
        self.__write_register1(dxl_id, ADDR_XL_DRIVE_MODE, drive_mode)

    # set goal velocity (drive servo)
    def set_goal_velocity(self, dxl_id, velocity):
        self.__write_register4(dxl_id, ADDR_XL_GOAL_VELOCITY, velocity)

    # set goal position (servo position)
    def set_goal_position(self, dxl_id, position):
        self.__write_register4(dxl_id, ADDR_XL_GOAL_POSITION, position)

    # set goal velocity to two dynamixel's (drive servos)
    def set_goal_velocity_group(self, dxl1_id, dxl2_id, dxl1_velocity, dxl2_velocity):
        dxl1_velocity_value = self.__sync_value(dxl1_velocity)
        dxl2_velocity_value = self.__sync_value(dxl2_velocity)
        self.__write_group_register4(dxl1_id, dxl2_id, ADDR_XL_GOAL_VELOCITY, XL_LEN_GOAL_VELOCITY, dxl1_velocity_value, dxl2_velocity_value)

    # set goal position to two dynamixel's (servo positions)
    def set_goal_position_group(self, dxl1_id, dxl2_id, dxl1_position, dxl2_position):
        dxl1_position_value = self.__sync_value(dxl1_position)
        dxl2_position_value = self.__sync_value(dxl2_position)
        self.__write_group_register4(dxl1_id, dxl2_id, ADDR_XL_GOAL_POSITION, XL_LEN_GOAL_POSITION, dxl1_position_value, dxl2_position_value)

    # MARK: PRIVATE REGISTER FUNCTIONS
    def __write_register1(self, dxl_id, address, value):
        dxl_result, dxl_error = self.__packet_handler.write1ByteTxRx(self.__port_handler, dxl_id, address, value)
        self.__validate_write(dxl_result, dxl_error)

    def __write_register2(self, dxl_id, address, value):
        dxl_result, dxl_error = self.__packet_handler.write2ByteTxRx(self.__port_handler, dxl_id, address, value)
        self.__validate_write(dxl_result, dxl_error)

    def __write_register4(self, dxl_id, address, value):
        dxl_result, dxl_error = self.__packet_handler.write4ByteTxRx(self.__port_handler, dxl_id, address, value)
        self.__validate_write(dxl_result, dxl_error)

    def __write_group_register4(self, dxl1_id, dxl2_id, address, addr_len, dxl1_value, dxl2_value):
        dxl1_param = self.__group_bulk_write.addParam(dxl1_id, address, addr_len, dxl1_value)
        dxl2_param = self.__group_bulk_write.addParam(dxl2_id, address, addr_len, dxl2_value)
        self.__validate_param(dxl1_id, dxl2_id, dxl1_param, dxl2_param)

        dxl_result = self.__group_bulk_write.txPacket()
        self.__validate_write(dxl_result, 0)
        self.__group_bulk_write.clearParam()

    def __read_register1(self, dxl_id, address):
        dxl_data, dxl_result, dxl_error = self.__packet_handler.read1ByteTxRx(self.__port_handler, dxl_id, address)
        self.__validate_write(dxl_result, dxl_error)
        return dxl_data

    def __read_register2(self, dxl_id, address):
        dxl_data, dxl_result, dxl_error = self.__packet_handler.read2ByteTxRx(self.__port_handler, dxl_id, address)
        self.__validate_write(dxl_result, dxl_error)
        return dxl_data

    def __read_register4(self, dxl_id, address):
        dxl_data, dxl_result, dxl_error = self.__packet_handler.read4ByteTxRx(self.__port_handler, dxl_id, address)
        self.__validate_write(dxl_result, dxl_error)
        return dxl_data

    def __sync_value(self, value):
        data = [DXL_LOBYTE(DXL_LOWORD(value)),
                DXL_HIBYTE(DXL_LOWORD(value)),
                DXL_LOBYTE(DXL_HIWORD(value)),
                DXL_HIBYTE(DXL_HIWORD(value))]
        return data

    def __validate_param(self, dxl1_id, dxl2_id, dxl1_param, dxl2_param):
        if not dxl1_param:
            print("failed: param not added for id:", dxl1_id)
        if not dxl2_param:
            print("failed: param not added for id:", dxl2_id)

    def __validate_write(self, dxl_result, dxl_error):
        if dxl_result != COMM_SUCCESS:
            print("failed: data not written:", self.__packet_handler.getTxRxResult(dxl_result))
        elif dxl_error != 0:
            print("failed: data not written:", self.__packet_handler.getRxPacketError(dxl_error))
