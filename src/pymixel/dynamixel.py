#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Dynamixel
#
# Copyright (C) 2020 Vinzenz Weist Vinz1911@gmail.com
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from dynamixel_sdk import *
from .address_table import AddressTable


class Dynamixel:
    def __init__(self, path: str = "/dev/ttyACM0", baud: int = 1_000_000, version: int = 2.0):
        """
        create instance of 'Dynamixel' class
        :param path: the path of the dynamixel controller, default: '/dev/ttyACM0'
        :param baud: the baud rate of the dynamixel in bit/s, default: 1_000_000
        :param version: the protocol version, default: 2.0
        """
        self.__protocol_version = version
        self.__baud_rate = baud
        self.__port_handler = PortHandler(path)
        self.__packet_handler = PacketHandler(self.__protocol_version)
        self.__group_bulk_write = GroupBulkWrite(self.__port_handler, self.__packet_handler)

    def open(self):
        """
        open the port to the given path of the controller
        """
        try:
            self.__port_handler.openPort()
            self.__port_handler.setBaudRate(self.__baud_rate)
        except Exception as error:
            print(error)

    def close(self):
        """
        close the port of the connection to the controller
        """
        self.__port_handler.closePort()

    def reboot(self, dxl_id: int):
        """
        reboot a dynamixel
        :param dxl_id: the id of the dynamixel
        """
        dxl_result, dxl_error = self.__packet_handler.reboot(self.__port_handler, dxl_id)
        self.__validate(dxl_result, dxl_error)

    def factory_reset(self, dxl_id: int):
        """
        reset a dynamixel to their default values
        :param dxl_id: the id of the dynamixel
        """
        dxl_result, dxl_error = self.__packet_handler.factoryReset(self.__port_handler, dxl_id)
        self.__validate(dxl_result, dxl_error)

    def get_torque(self, dxl_id: int) -> int:
        """
        get the current torque state of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the torque state
        """
        dxl_torque = self.__get_small_register(dxl_id, AddressTable.ADDR_TORQUE.value)
        return dxl_torque

    def get_led(self, dxl_id: int) -> int:
        """
        get the current led state of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the led state
        """
        dxl_led = self.__get_small_register(dxl_id, AddressTable.ADDR_LED.value)
        return dxl_led

    def get_id(self, dxl_id: int) -> int:
        """
        get the the current id of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the id
        """
        dxl_identifier = self.__get_small_register(dxl_id, AddressTable.ADDR_ID.value)
        return dxl_identifier

    def get_shadow_id(self, dxl_id: int) -> int:
        """
        get the current shadow id of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the shadow id
        """
        dxl_shadow_id = self.__get_small_register(dxl_id, AddressTable.ADDR_SHADOW_ID.value)
        return dxl_shadow_id

    def get_drive_mode(self, dxl_id: int) -> int:
        """
        get the current drive mode of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the drive mode
        """
        dxl_drive_mode = self.__get_small_register(dxl_id, AddressTable.ADDR_DRIVE_MODE.value)
        return dxl_drive_mode

    def get_operating_mode(self, dxl_id: int) -> int:
        """
        get the current operating mode of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the operating mode
        """
        dxl_operating_mode = self.__get_small_register(dxl_id, AddressTable.ADDR_OPERATING_MODE.value)
        return dxl_operating_mode

    def get_model_number(self, dxl_id: int) -> int:
        """
        get the model number of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the model number of the dynamixel
        """
        dxl_model_number = self.__get_medium_register(dxl_id, AddressTable.ADDR_MODEL_NUMBER.value)
        return dxl_model_number

    def get_firmware_version(self, dxl_id: int) -> int:
        """
        get the firmware version of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the firmware version
        """
        dxl_firmware_version = self.__get_small_register(dxl_id, AddressTable.ADDR_FIRMWARE_VERSION.value)
        return dxl_firmware_version

    def get_protocol_type(self, dxl_id: int) -> int:
        """
        get the protocol type of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the protocol type
        """
        dxl_protocol_type = self.__get_small_register(dxl_id, AddressTable.ADDR_PROTOCOL_TYPE.value)
        return dxl_protocol_type

    def get_velocity_kp_gain(self, dxl_id: int) -> int:
        """
        get the velocity p gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the velocity p gain
        """
        dxl_gain = self.__get_medium_register(dxl_id, AddressTable.ADDR_VELOCITY_P_GAIN.value)
        return dxl_gain

    def get_velocity_ki_gain(self, dxl_id: int) -> int:
        """
        get the velocity i gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the velocity i gain
        """
        dxl_gain = self.__get_medium_register(dxl_id, AddressTable.ADDR_VELOCITY_I_GAIN.value)
        return dxl_gain

    def get_position_kp_gain(self, dxl_id: int) -> int:
        """
        get the position p gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the position p gain
        """
        dxl_gain = self.__get_medium_register(dxl_id, AddressTable.ADDR_POSITION_P_GAIN.value)
        return dxl_gain

    def get_position_ki_gain(self, dxl_id: int) -> int:
        """
        get the position i gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the position i gain
        """
        dxl_gain = self.__get_medium_register(dxl_id, AddressTable.ADDR_POSITION_I_GAIN.value)
        return dxl_gain

    def get_position_kd_gain(self, dxl_id: int) -> int:
        """
        get the position d gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the position d gain
        """
        dxl_gain = self.__get_medium_register(dxl_id, AddressTable.ADDR_POSITION_D_GAIN.value)
        return dxl_gain

    def get_feedforward_first_gain(self, dxl_id: int) -> int:
        """
        get the feedforward first gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the feedforward first gain
        """
        dxl_gain = self.__get_medium_register(dxl_id, AddressTable.ADDR_FEEDFORWARD_FIRST_GAIN.value)
        return dxl_gain

    def get_feedforward_second_gain(self, dxl_id: int) -> int:
        """
        get the feedforward second gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the feedforward second gain
        """
        dxl_gain = self.__get_medium_register(dxl_id, AddressTable.ADDR_FEEDFORWARD_SECOND_GAIN.value)
        return dxl_gain

    def get_present_temperature(self, dxl_id: int) -> int:
        """
        get the present temperature of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present temperature
        """
        dxl_temperature = self.__get_small_register(dxl_id, AddressTable.ADDR_PRESENT_TEMPERATURE.value)
        return dxl_temperature

    def get_present_pwm(self, dxl_id: int) -> int:
        """
        get the present pwm of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present pwm
        """
        dxl_pwm = self.__get_large_register(dxl_id, AddressTable.ADDR_PRESENT_PWM.value)
        return dxl_pwm

    def get_present_load(self, dxl_id: int) -> int:
        """
        get the present load of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present load
        """
        dxl_load = self.__get_medium_register(dxl_id, AddressTable.ADDR_PRESENT_LOAD.value)
        return dxl_load

    def get_present_input_voltage(self, dxl_id: int) -> int:
        """
        get the present input voltage of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present input voltage
        """
        dxl_input_voltage = self.__get_medium_register(dxl_id, AddressTable.ADDR_PRESENT_INPUT_VOLTAGE.value)
        return dxl_input_voltage

    def get_present_velocity(self, dxl_id: int) -> int:
        """
        get the present velocity of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present velocity
        """
        dxl_velocity = self.__get_large_register(dxl_id, AddressTable.ADDR_PRESENT_VELOCITY.value)
        return dxl_velocity

    def get_present_position(self, dxl_id: int) -> int:
        """
        get the present position of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present position
        """
        dxl_position = self.__get_large_register(dxl_id, AddressTable.ADDR_PRESENT_POSITION.value)
        return dxl_position

    def get_realtime_tick(self, dxl_id: int) -> int:
        """
        get the realtime tick of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the realtime tick
        """
        dxl_realtime_tick = self.__get_medium_register(dxl_id, AddressTable.ADDR_REALTIME_TICK.value)
        return dxl_realtime_tick

    def set_torque(self, dxl_id: int, torque_state: int):
        """
        set the torque of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param torque_state: the torque as int
        """
        self.__set_small_register(dxl_id, AddressTable.ADDR_TORQUE.value, torque_state)

    def set_led(self, dxl_id: int, led_state: bool):
        """
        set the led of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param led_state: led state as bool
        """
        self.__set_small_register(dxl_id, AddressTable.ADDR_LED.value, led_state)

    def set_id(self, dxl_id: int, identifier: int):
        """
        set the dynamixel id of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param identifier: the id as int from 0-255
        """
        self.__set_small_register(dxl_id, AddressTable.ADDR_ID.value, identifier)

    def set_shadow_id(self, dxl_id: int, shadow_id: int):
        """
        set the shadow id of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param shadow_id: the shadow id as int from 0-255
        """
        self.__set_small_register(dxl_id, AddressTable.ADDR_SHADOW_ID.value, shadow_id)

    def set_operating_mode(self, dxl_id: int, control_mode: int):
        """
        set the operating mode of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param control_mode: the operating mode as int
        """
        self.__set_small_register(dxl_id, AddressTable.ADDR_OPERATING_MODE.value, control_mode)

    def set_drive_mode(self, dxl_id: int, drive_mode: int):
        """
        set the drive mode of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param drive_mode: the drive mode as int
        """
        self.__set_small_register(dxl_id, AddressTable.ADDR_DRIVE_MODE.value, drive_mode)

    def set_goal_velocity(self, dxl_id: int, velocity: int):
        """
        set the goal velocity of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param velocity: the velocity as int, depends on model
        """
        self.__set_large_register(dxl_id, AddressTable.ADDR_GOAL_VELOCITY.value, velocity)

    def set_goal_position(self, dxl_id: int, position: int):
        """
        set the goal position of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param position: the goal position as int from 0-4096, depends on model
        """
        self.__set_large_register(dxl_id, AddressTable.ADDR_GOAL_POSITION.value, position)

    def set_velocity_kp_gain(self, dxl_id: int, gain: int):
        """
        set the velocity p gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self.__set_medium_register(dxl_id, AddressTable.ADDR_VELOCITY_P_GAIN.value, gain)

    def set_velocity_ki_gain(self, dxl_id: int, gain: int):
        """
        set the velocity i gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self.__set_medium_register(dxl_id, AddressTable.ADDR_VELOCITY_I_GAIN.value, gain)

    def set_position_kp_gain(self, dxl_id: int, gain: int):
        """
        set the position p gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self.__set_medium_register(dxl_id, AddressTable.ADDR_POSITION_P_GAIN.value, gain)

    def set_position_ki_gain(self, dxl_id: int, gain: int):
        """
        set the position i gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self.__set_medium_register(dxl_id, AddressTable.ADDR_POSITION_I_GAIN.value, gain)

    def set_position_kd_gain(self, dxl_id: int, gain: int):
        """
        set the position d gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self.__set_medium_register(dxl_id, AddressTable.ADDR_POSITION_D_GAIN.value, gain)

    def set_feedforward_first_gain(self, dxl_id: int, gain: int):
        """
        set the feedforward first gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self.__set_medium_register(dxl_id, AddressTable.ADDR_FEEDFORWARD_FIRST_GAIN.value, gain)

    def set_feedforward_second_gain(self, dxl_id: int, gain: int):
        """
        set the feedforward first gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self.__set_medium_register(dxl_id, AddressTable.ADDR_FEEDFORWARD_SECOND_GAIN.value, gain)

    def set_goal_velocity_group(self, dxl1_id: int, dxl2_id: int, dxl1_velocity: int, dxl2_velocity: int):
        """
        set the goal velocity of a dynamixel group
        :param dxl1_id: the id of the first dynamixel
        :param dxl2_id: the id of the second dynamixel
        :param dxl1_velocity: velocity as int of the first dynamixel
        :param dxl2_velocity: velocity as int of the second dynamixel
        """
        dxl1_velocity_value = self.__sync_value(dxl1_velocity)
        dxl2_velocity_value = self.__sync_value(dxl2_velocity)
        self.__set_group_register(dxl1_id, dxl2_id, AddressTable.ADDR_GOAL_VELOCITY.value,
                                  AddressTable.ADDR_LEN_GOAL_VELOCITY.value,
                                  dxl1_velocity_value, dxl2_velocity_value)

    def set_goal_position_group(self, dxl1_id: int, dxl2_id: int, dxl1_position: int, dxl2_position: int):
        """
        set the goal position of a dynamixel group
        :param dxl1_id: the id of the first dynamixel
        :param dxl2_id: the id of the second dynamixel
        :param dxl1_position: position as int of the first dynamixel
        :param dxl2_position: position as int of the first dynamixel
        """
        dxl1_position_value = self.__sync_value(dxl1_position)
        dxl2_position_value = self.__sync_value(dxl2_position)
        self.__set_group_register(dxl1_id, dxl2_id, AddressTable.ADDR_GOAL_POSITION.value,
                                  AddressTable.ADDR_LEN_GOAL_POSITION.value,
                                  dxl1_position_value, dxl2_position_value)

    # MARK: PRIVATE REGISTER FUNCTIONS
    def __set_small_register(self, dxl_id: int, address: int, value: int):
        """
        set a small register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :param value: the value to set
        """
        dxl_result, dxl_error = self.__packet_handler.write1ByteTxRx(self.__port_handler, dxl_id, address, value)
        self.__validate(dxl_result, dxl_error)

    def __set_medium_register(self, dxl_id: int, address: int, value: int):
        """
        set a medium register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :param value: the value to set
        """
        dxl_result, dxl_error = self.__packet_handler.write2ByteTxRx(self.__port_handler, dxl_id, address, value)
        self.__validate(dxl_result, dxl_error)

    def __set_large_register(self, dxl_id: int, address: int, value: int):
        """
        set a large register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :param value: the value to set
        """
        dxl_result, dxl_error = self.__packet_handler.write4ByteTxRx(self.__port_handler, dxl_id, address, value)
        self.__validate(dxl_result, dxl_error)

    def __set_group_register(self, dxl1_id: int, dxl2_id: int, address: int, addr_len: int, dxl1_value: list, dxl2_value: list):
        """
        set a large register of a group of dynamixel
        :param dxl1_id: the id of the first dynamixel
        :param dxl2_id: the id of the second dynamixel
        :param address: dynamixel register address
        :param addr_len: the len of the address register
        :param dxl1_value: the list of the first dynamixel to set
        :param dxl2_value: the list of the second dynamixel to set
        """
        dxl1_param = self.__group_bulk_write.addParam(dxl1_id, address, addr_len, dxl1_value)
        dxl2_param = self.__group_bulk_write.addParam(dxl2_id, address, addr_len, dxl2_value)
        if not dxl1_param or not dxl2_param:
            return
        dxl_result = self.__group_bulk_write.txPacket()
        self.__validate(dxl_result, 0)
        self.__group_bulk_write.clearParam()

    def __get_small_register(self, dxl_id: int, address: int) -> int:
        """
        get a small register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :return: the register entry
        """
        dxl_data, dxl_result, dxl_error = self.__packet_handler.read1ByteTxRx(self.__port_handler, dxl_id, address)
        self.__validate(dxl_result, dxl_error)
        return dxl_data

    def __get_medium_register(self, dxl_id: int, address: int) -> int:
        """
        get a medium register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :return: the register entry
        """
        dxl_data, dxl_result, dxl_error = self.__packet_handler.read2ByteTxRx(self.__port_handler, dxl_id, address)
        self.__validate(dxl_result, dxl_error)
        return dxl_data

    def __get_large_register(self, dxl_id: int, address: int) -> int:
        """
        get a large register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :return: the register entry
        """
        dxl_data, dxl_result, dxl_error = self.__packet_handler.read4ByteTxRx(self.__port_handler, dxl_id, address)
        self.__validate(dxl_result, dxl_error)
        return dxl_data

    def __sync_value(self, value: int) -> list:
        """
        sync value for group register set
        :param value: the id of the dynamixel
        :return: the synced data set
        """
        data = [DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)),
                DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value))]
        return data

    def __validate(self, dxl_result: int, dxl_error: int):
        """
        validate register set result's
        :param dxl_result: result value
        :param dxl_error: error value
        """
        if dxl_result != COMM_SUCCESS:
            print(self.__packet_handler.getTxRxResult(dxl_result))
        elif dxl_error != 0:
            print(self.__packet_handler.getRxPacketError(dxl_error))