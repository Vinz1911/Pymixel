from dynamixel_sdk import *
from XL430_ADDRESS_TABLE import *


class XL430:
    """Handsome class to control Dynamixel XL430 Servos"""
    PROTOCOL_VERSION = 2.0  # Dynamixel Protocol Version
    BAUDRATE = 1_000_000  # Default Baud Rate
    DEVICENAME = "/dev/ttyACM0"  # Device Connection Path

    # init class
    def __init__(self):
        self.__port_handler = PortHandler(self.DEVICENAME)
        self.__packet_handler = PacketHandler(self.PROTOCOL_VERSION)
        self.__goal_velocity_sync_write = GroupSyncWrite(self.__port_handler, self.__packet_handler,
                                                         ADDR_XL_GOAL_VELOCITY, LEN_XL_GOAL_VELOCITY)
        self.__goal_position_sync_write = GroupSyncWrite(self.__port_handler, self.__packet_handler,
                                                         ADDR_XL_GOAL_POSITION, LEN_XL_GOAL_POSITION)

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

    # set torque on servo
    def set_torque(self, dxl_id, torque_state):
        self.__write_register1(dxl_id, ADDR_XL_TORQUE_ENABLE, torque_state)

    # set led on servo
    def set_led(self, dxl_id, led_state):
        self.__write_register1(dxl_id, ADDR_XL_LED, led_state)

    # set servo operating mode
    def set_operating_mode(self, dxl_id, control_mode):
        self.__write_register1(dxl_id, ADDR_XL_OPERATING_MODE, control_mode)

    # set goal velocity (drive servo)
    def set_goal_velocity(self, dxl_id, velocity):
        self.__write_register4(dxl_id, ADDR_XL_GOAL_VELOCITY, velocity)

    # set goal position (servo position)
    def set_goal_position(self, dxl_id, position):
        self.__write_register4(dxl_id, ADDR_XL_GOAL_POSITION, position)

    # set goal velocity to two dynamixel's (drive servos)
    def set_sync_goal_velocity(self, dxl1_id, dxl2_id, velocity_left, velocity_right):
        goal_velocity_left = self.__sync_value(velocity_left)
        goal_velocity_right = self.__sync_value(velocity_right)

        dxl1_param = self.__goal_velocity_sync_write.addParam(dxl1_id, goal_velocity_left)
        dxl2_param = self.__goal_velocity_sync_write.addParam(dxl2_id, goal_velocity_right)
        self.__validate_param(dxl1_id, dxl2_id, dxl1_param, dxl2_param)

        self.__write_sync_velocity_register4()

    # set goal position to two dynamixel's (servo positions)
    def set_sync_goal_position(self, dxl1_id, dxl2_id, position_left, position_right):
        goal_position_left = self.__sync_value(position_left)
        goal_position_right = self.__sync_value(position_right)

        dxl1_param = self.__goal_position_sync_write.addParam(dxl1_id, goal_position_left)
        dxl2_param = self.__goal_position_sync_write.addParam(dxl2_id, goal_position_right)
        self.__validate_param(dxl1_id, dxl2_id, dxl1_param, dxl2_param)

        self.__write_sync_position_register4()

    # get current servo temperature
    def get_present_temperature(self, dxl_id):
        dxl_temperature = self.__read_register1(dxl_id, ADDR_XL_PRESENT_TEMPERATURE)
        return dxl_temperature

    # get current servo velocity
    def get_present_velocity(self, dxl_id):
        dxl_velocity = self.__read_register4(dxl_id, ADDR_XL_PRESENT_VELOCITY)
        return dxl_velocity

    # get current servo position
    def get_present_position(self, dxl_id):
        dxl_position = self.__read_register4(dxl_id, ADDR_XL_PRESENT_POSITION)
        return dxl_position

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

    def __write_sync_velocity_register4(self):
        dxl_result = self.__goal_velocity_sync_write.txPacket()
        self.__validate_write(dxl_result, 0)
        self.__goal_velocity_sync_write.clearParam()

    def __write_sync_position_register4(self):
        dxl_result = self.__goal_position_sync_write.txPacket()
        self.__validate_write(dxl_result, 0)
        self.__goal_position_sync_write.clearParam()

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
