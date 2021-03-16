#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# AddressTable
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

from enum import Enum


class AddressTable(Enum):
    # 1. EEPROM based codes
    ADDR_MODEL_NUMBER = 0
    ADDR_MODEL_INFORMATION = 2
    ADDR_FIRMWARE_VERSION = 6
    ADDR_ID = 7
    ADDR_BAUD_RATE = 8
    ADDR_RETURN_DELAY_TIME = 9
    ADDR_DRIVE_MODE = 10
    ADDR_OPERATING_MODE = 11
    ADDR_SHADOW_ID = 12
    ADDR_PROTOCOL_TYPE = 13
    ADDR_HOMING_OFFSET = 20
    ADDR_MOVING_THRESHOLD = 24
    ADDR_TEMPERATURE_LIMIT = 31
    ADDR_MAX_VOLTAGE_LIMIT = 32
    ADDR_MIN_VOLTAGE_LIMIT = 34
    ADDR_PWM_LIMIT = 36
    ADDR_VELOCITY_LIMIT = 44
    ADDR_MAX_POSITION_LIMIT = 48
    ADDR_MIN_POSITION_LIMIT = 52
    ADDR_SHUTDOWN = 63

    # 1. Ram based codes
    ADDR_TORQUE_ENABLE = 64
    ADDR_LED = 65
    ADDR_STATUS_RETURN_LEVEL = 68
    ADDR_REGISTERED_INSTRUCTION = 69
    ADDR_HARDWARE_ERROR_STATUS = 70
    ADDR_VELOCITY_I_GAIN = 76
    ADDR_VELOCITY_P_GAIN = 78
    ADDR_POSITION_D_GAIN = 80
    ADDR_POSITION_I_GAIN = 82
    ADDR_POSITION_P_GAIN = 84
    ADDR_FEEDFORWARD_2_GAIN = 88
    ADDR_FEEDFORWARD_1_GAIN = 90
    ADDR_BUS_WATCHDOG = 98
    ADDR_GOAL_PWM = 100
    ADDR_GOAL_VELOCITY = 104
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_GOAL_POSITION = 116
    ADDR_REALTIME_TICK = 120
    ADDR_MOVING = 122
    ADDR_MOVING_STATUS = 123
    ADDR_PRESENT_PWM = 124
    ADDR_PRESENT_LOAD = 126
    ADDR_PRESENT_VELOCITY = 128
    ADDR_PRESENT_POSITION = 132
    ADDR_VELOCITY_TRAJECTORY = 136
    ADDR_POSITION_TRAJECTORY = 140
    ADDR_PRESENT_INPUT_VOLTAGE = 144
    ADDR_PRESENT_TEMPERATURE = 146

    # extended control code
    ADDR_TORQUE_DISABLED = 0
    ADDR_TORQUE_ENABLED = 1
    ADDR_LED_DISABLED = 0
    ADDR_LED_ENABLED = 1
    ADDR_VELOCITY_CONTROL_MODE = 1
    ADDR_POSITION_CONTROL_MODE = 3
    ADDR_EXTENDED_POSITION_CONTROL_MODE = 4
    ADDR_PWM_CONTROL_MODE = 16
    ADDR_LEN_GOAL_VELOCITY = 4
    ADDR_LEN_GOAL_POSITION = 4
    ADDR_LEN_PRESENT_POSITION = 4

