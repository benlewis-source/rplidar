import serial
from functools import reduce
import logging
import sys
import time
from collections import deque
import math
import numpy as np

class Command:
    """
    All the possible commands that can be sent to rplidars
    """
    Stop = 0x25
    Reset = 0x40
    Scan = 0x20
    ExpressScan = 0x82
    ForceScan = 0x21
    GetInfo = 0x50
    GetHealth = 0x52
    GetSampleRate = 0x59
    GetConf = 0x84


class ConfigEntry:
    """
    Commands for querying the device configuration
    """
    count = 0x70
    us_per_sample = 0x71
    max_distance = 0x74
    ans_type = 0x75
    typical = 0x7c
    name = 0x7f


class DataType:
    """
    The last (7th) byte of each response descriptor
    """
    ScanData = 0x81
    ExpressScanLegacy = 0x82
    ExpressScanExt = 0x84
    ExpressScanDense = 0x85
    DeviceInfo = 0x04
    HealthInfo = 0x06
    SampleRate = 0x15
    Configuration = 0x20


class RpLidarA1(object):
    def __init__(self, comport):
        # TODO: Make the serial object private once the driver is finished
        self.com = serial.Serial(
            port=comport,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            write_timeout=5.0,
            timeout=2.0
        )

        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
        self.log = logging.getLogger('rplidar')

        self.reset()
        device_info = self.com.read_all().decode()
        self.log.info("\n{}".format(device_info))

    def open(self):
        """
        Opens the connection to the serial device
        """
        self.com.open()

    def close(self):
        """
        Stops the motor and closes the connection to the serial device
        """
        self._set_motor_state(False)
        self.stop()
        self.com.close()

    def erase_input(self):
        """
        Erase everything in the input buffer
        """
        self.com.reset_input_buffer()
        if self.com.in_waiting != 0:
            raise Warning("non empty input buffer. Serial device is still transmitting data")

    def erase_output(self):
        """
        Erase everything in the output buffer
        """
        self.com.reset_output_buffer()
        assert self.com.out_waiting == 0

    def write(self, data):
        """
        Writes a list of bytes to the write buffer.
        Args:
            data (list): A list of bytes

        Returns: The number of bytes written
        """
        return self.com.write(data)

    def read(self, size=1):
        """
        Reads a number of bytes from the read buffer
        Args:
            size (int): The number of bytes to read from the read buffer

        Returns: A byte-string

        """
        return self.com.read(size)

    def request(self, cmd, data=None, clr_input=True):
        """
        Creates a request packet using the command byte and writes it to the write buffer. If additional data bytes are
        present, a calculated checksum will be included.

        Args:
            cmd: The command byte
            data (list): Data bytes (LSB first)
            clr_input (bool): clear the input buffer before sending the request

        Raises:
            RuntimeWarning if not all bytes were written to the read buffer
        """
        packet = [0xA5, cmd]
        if data:
            packet.append(len(data))
            packet += data
            cs = self._checksum(cmd, data)
            packet.append(cs)
        if clr_input:
            self.erase_input()
        res = self.write(packet)
        if res != len(packet):
            raise RuntimeWarning("Failed to write all bytes ({}/{})".format(res, len(packet)))

    def _checksum(self, cmd, data):
        """
        Calculates the checksum of a packet which contains data
        Args:
            cmd: The command byte
            data: The data byte(s)

        Returns:
            cs: The checksum of the packet
        """
        cs = 0 ^ 0xA5 ^ cmd ^ len(data) ^ reduce(lambda i, j: i ^ j, data)
        return cs

    def _get_response_descriptor(self):
        """
        Waits for the check byte (0xa5) then checks the identity of the 6th byte against `descriptor`
        Args:
            descriptor: The target descriptor

        Raises:
            RuntimeError if the response does not match the target descriptor

        """
        response = {}
        _raw = int.from_bytes(self.read(7), byteorder='little')
        response['start_flag_1'] = (_raw & 0x000000000000FF)
        response['start_flag_2'] = (_raw & 0x0000000000FF00) >> 0x8
        response['data_length'] = (_raw & 0x003FFFFFFF0000) >> 0x10
        response['send_mode'] = (_raw & 0x00C00000000000) >> 0x2E
        response['data_type'] = (_raw & 0xFF000000000000) >> 0x30

        if (response['start_flag_1'] != 0xA5) or (response['start_flag_2'] != 0x5A):
            raise RuntimeError('Response descriptor header not found')

        return response

    def _set_motor_state(self, state):
        """
        RPLIDAR A1 does not support setting motor speed via serial commands.
        Motor control is hardwired to the DTR pin of the serial adapter in an active low configruation

        Args:
            state (bool): True/False = On/Off
        """
        self.log.debug("Motor state: {}".format(['Off', 'On'][state]))
        self.com.dtr = not state

    def reset(self):
        """
        Make RPLIDAR core reset (reboot) itself by sending this request. A reset operation will make RPLIDAR revert to
        a similar state as it has just been powered up.
        """
        self.stop()
        self.erase_input()
        self.log.debug("Device reset")
        self.request(Command.Reset, clr_input=False)
        time.sleep(0.75)

    def stop(self):
        """
        RPLIDAR will exit the current scanning state once it receives
        the stop (0x25) request. The laser diode and the measurement
        system will be disables and the idle state will be entered.
        This request will be ignored when the RPLIDAR is in the idle
        or protection-stop state.
        """
        self.log.debug("Device stopped")
        self.request(Command.Stop, clr_input=False)
        time.sleep(0.1)

    def start_scan(self, force=False):
        """
        RPLIDAR will enter the scanning state once it receives this request. Each measurement sample result will be sent
        out using an individual data response packet. If the RPLIDAR has been in scanning state already, it will stop
        the current measurement and start a new round of scanning.
        The related response descriptor will be sent out immediately once it receives the request and accepts it. The
        data response packets related to every measurement sample results will be sent out continuously only after
        the motor rotation becomes stable. RPLIDAR will leave the scanning state once it receives a new request or it
        detects something is wrong.

        Args:
            force (bool): Start scanning in force mode

        Note:
            When force is False the device will only start scanning once motor rotation has stabilized
        """
        self.log.info("Entering standard scan mode")
        cmd = [Command.Scan, Command.ForceScan][force]
        self._set_motor_state(True)
        self.request(cmd)
        descriptor = self._get_response_descriptor()
        if descriptor['data_type'] != DataType.ScanData:
            raise RuntimeError("Invalid response descriptor data type")

    def start_express_scan(self):
        """
        Start scanning in express mode
        """
        self.log.info("Entering express scan mode")
        self._set_motor_state(True)
        self.request(Command.ExpressScan, [0]*5)
        self._check_response_descriptor(ResponseType.ExpressScanLegacy)

    def stop_scan(self):
        """
        Stops the motor and exits the current scan mode
        """
        self.stop()
        self._set_motor_state(False)

    def scan_data(self):
        """
        Generates points to a scan
        """
        # Need to read data in chuncks. Reading the buffer 5 bytes at a time is too slow
        # and will cause the buffer to overflow. 
        chunk_size = 250
        scan = []
        while True:
            # Read chunk
            data = self.read(chunk_size)
            # Process chunk in increments of 5 bytes
            for i in range(0, chunk_size, 5):
                _raw = int.from_bytes(data[i:i+5], byteorder='little')
                sample = {}
                sample['start_bit'] = (_raw & 0x0000000001)
                sample['inv_start_bit'] = (_raw & 0x0000000002) >> 0x01
                sample['quality'] = (_raw & 0x00000000FC) >> 0x02
                sample['check_bit'] = (_raw & 0x0000000100) >> 0x08
                sample['angle_q6'] = (_raw & 0x0000FFFE00) >> 0x09
                sample['angle'] = sample['angle_q6'] / 64.0
                sample['distance_q2'] = (_raw & 0xFFFF000000) >> 0x18
                sample['distance'] = sample['distance_q2'] / 4.0

                # Only save quality points
                if (sample['quality'] > 0) and (sample['distance'] > 10):
                    scan.append(sample)
                # Yield the points when a start bit is encountered
                if sample['start_bit']:
                    yield scan[:-1]
                    scan = [scan[-1]] 

    def read_express_scan_data(self):
        pass

    def get_device_info(self):
        """
        Retrieves device specific information
        """
        self.request(Command.GetInfo)
        descriptor = self._get_response_descriptor()
        if descriptor['data_type'] != DataType.DeviceInfo:
            raise RuntimeError("Invalid response descriptor data type")

        info = self.read(descriptor['data_length'])
        info_string = "\nModel: {}\nFirmware version: {}.{}\nHardware version: {}\nSerial number: {}".format(
            info[0], info[1], info[2], info[3], info[4:].hex()
        )
        self.log.info(info_string)
        self.erase_input()

    def get_device_health(self):
        """
        Checks the device status and error codes

        Returns:
            status: The health status code
        """
        self.request(Command.GetHealth)
        descriptor = self._get_response_descriptor()
        if descriptor['data_type'] != DataType.HealthInfo:
            raise RuntimeError("Invalid response descriptor data type")
        health = self.read(descriptor['data_length'])
        status = health[0]
        status_str = ['good', 'warning', 'error'][status]
        error_code = (health[2] << 8) | health[1]
        self.log.debug("Status: {} ({})".format(status, status_str))
        if status == 1:
            self.log.warning('Device health risk. Continued use may cause hardware failure\nError code: {}'.format(error_code))
        elif status == 2:
            self.log.error('Device error. Device in protection stop state\nError code: {}'.format(error_code))
        return status

    def get_sample_rate(self):
        """
        Gets the sample period of the device in standard and express mode. Useful when calculating the motor rotation
        speed

        Returns:
            t_standard: The sample period of the device in standard mode
            t_express: The sample period of the device in express mode

        """
        self.request(Command.GetSampleRate)
        descriptor = self._get_response_descriptor()
        if descriptor['data_type'] != DataType.SampleRate:
            raise RuntimeError("Invalid response descriptor data type")
        rate = self.read(descriptor['data_length'])
        t_standard = (rate[1] << 8) | rate[0]
        t_express = (rate[3] << 8) | rate[2]
        self.log.debug("T_standard: {}uS. T_express: {}uS)".format(t_standard, t_express))
        return t_standard, t_express

    def _get_scan_mode_count(self):
        raise NotImplementedError

    def _get_scan_mode_us_per_sample(self):
        raise NotImplementedError

    def _get_scan_mode_max_distance(self):
        raise NotImplementedError

    def _get_scan_mode_ans_type(self):
        raise NotImplementedError

    def _get_scan_mode_typical(self):
        raise NotImplementedError

    def _get_scan_mode_name(self):
        raise NotImplementedError
