import serial
from functools import reduce
import logging
import sys
import time
from collections import deque
import math
import numpy as np
from rplidar.definitions import Common, Config
from rplidar.definitions import Request
from rplidar.definitions import Response
# from rplidar.definitions import ResponseDesc
# from rplidar.definitions import DeviceInfo
from bitstring import BitArray


def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

def angle_diff(w0, w1):
    res = w1 - w0
    if res < 0:
        res += 360
    return res

class RpLidarA1M8(object):
    def __init__(self, comport):
        # TODO: Make the serial object private once the driver is finished
        self.com = serial.Serial(
            port=comport,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            write_timeout=5.0
            # timeout=2.0
        )

        self.log = logging.getLogger('rplidar')
        stream_handler = logging.StreamHandler(sys.stdout)
        stream_handler.setLevel(logging.DEBUG)
        self.log.addHandler(stream_handler)

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
            print("non empty input buffer. Serial device is still transmitting data")

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
        return BitArray(self.com.read(size))

    def checksum(self, request, payload):
        """
        Calculates the an 8bit (1byte) checksum 
        """
        checksum = 0 ^ 0xA5 ^ request.uint ^ len(payload.bytes) ^ reduce(lambda i, j: i ^ j, payload.bytes)
        return BitArray(uint=checksum, length=8)

    def send_request(self, request: BitArray, payload=None, clr_input=True):
        """
        Creates a request packet using the command byte and writes it to the write buffer. If additional data bytes are
        present, a calculated checksum will be included.

        Args:
            request: The request byte
            payload (BitArray): payload bytes (LSB first)
            clr_input (bool): clear the input buffer before sending the request

        Raises:
            RuntimeWarning if not all bytes were written to the read buffer
        """
        request_packet = Common.START_FLAG + request
        if payload is not None:
            request_packet += BitArray(uint=len(payload.bytes), length=8)
            request_packet += payload
            request_packet += self.checksum(request, payload)
        if clr_input:
            self.erase_input()
        res = self.write(request_packet.bytes)
        if res != len(request_packet.bytes):
            raise RuntimeWarning("Failed to write all bytes ({}/{})".format(res, len(request_packet.bytes)))

    def get_response(self):
        """
        Waits for the check byte (0xa5) then checks the identity of the 6th byte against `descriptor`
        Args:
            descriptor: The target descriptor

        Raises:
            RuntimeError if the response does not match the target descriptor
        """
        response = {}
        packet = self.read(7)
        response['start_flag_1'] = packet[0:8]
        response['start_flag_2'] = packet[8:16]
        # Add padding bits to make these a whole number of bytes
        response['data_length'] = (packet[16:46] + '0b00').uintle
        response['send_mode'] = packet[46:48] + '0b000000'
        response['data_type'] = packet[48:56]

        if (response['start_flag_1'] != Common.START_FLAG) or (response['start_flag_2'] != Common.START_FLAG_2):
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

    def stop(self):
        """
        RPLIDAR will exit the current scanning state once it receives
        the stop (0x25) request. The laser diode and the measurement
        system will be disables and the idle state will be entered.
        This request will be ignored when the RPLIDAR is in the idle
        or protection-stop state.
        """
        self.log.debug("Device stopped")
        self._set_motor_state(False)
        self.send_request(Request.STOP)
        time.sleep(0.1)

    def reset(self):
        """
        Make RPLIDAR core reset (reboot) itself by sending this request. A reset operation will make RPLIDAR revert to
        a similar state as it has just been powered up.
        """
        self.stop()
        self.log.debug("Device reset")
        self.send_request(Request.RESET)
        time.sleep(0.75)

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
        self.log.info("Initiating basic scan mode")
        self._set_motor_state(True)
        request = [Request.SCAN, Request.FORCE_SCAN][force]
        self.send_request(request)
        response = self.get_response()
        if response['data_type'] != [Response.SCAN, Response.FORCE_SCAN][force]:
            raise RuntimeError("Invalid response descriptor data type")
    
    def start_legacy_scan(self):
        self.log.info("Initiating legacy scan mode")
        self._set_motor_state(True)
        self.send_request(Request.EXPRESS_SCAN, payload=BitArray(b'\x00\x00\x00\x00\x00'))
        response = self.get_response()
        if response['data_type'] != Response.EXPRESS_SCAN_LEGACY:
            raise RuntimeError("Invalid response descriptor data type")

    def scan_data(self):
        """
        Generates points of a scan as a list of dictionaries

        original: [Quality 5:0|S|S] [Angle 6:0|C]  [Angle 14:7] [Distance 7:0] [Distance 15:8]
        byteswap: [Distance 15:8]   [Distance 7:0] [Angle 14:7] [Angle 6:0|C]  [Quality 5:0|S|S]
        """
        # Need to read data in chuncks. Reading the buffer 5 bytes at a time 
        # is too slow and will cause the buffer to overflow. 
        chunk_size = 250  # bytes
        scan = []
        while True:
            # Read chunk
            multi_packet = self.read(chunk_size)
            # Process chunk in increments of 40bits (5bytes)
            for packet in chunks(multi_packet, 40):
                # Swap byte order to make numerical values neighbours for cleaner slicing
                packet.byteswap()

                # We swapped the bytes so now BitpacketArray[0] == distance MSB and
                # packet[-1] == start bit
                sample = {}
                sample['start'] = packet[39]
                sample['start_inv'] = packet[38]
                sample['quality'] = packet[32:38].uint
                sample['check'] = packet[31]
                sample['degree'] = packet[16:31].uint / 64.0
                sample['radian'] = sample['degree'] * math.pi / 180
                sample['distance'] = packet[0:16].uint / 4.0

                # Only save valid points
                if sample['check'] and (sample['start'] != sample['start_inv']):
                    scan.append(sample)
                # Yield the points when a start bit is encountered
                if sample['start']:
                    yield scan[:-1]
                    scan = [scan[-1]] 

    def legacy_scan_data(self):
        packet_size = 84  # 84 bytes
        header_size = 4
        cabin_size = 5
        scan = []
        previous_packet = None
        previous_angle = 0
        while True:
            # TODO: Slice backwards and remove all the reversals
            packet = self.read(packet_size)
            packet.byteswap()
            packet.reverse()

            checksum = packet[0:4] + packet[8:12]
            checksum.reverse()

            sync1 = packet[4:8]
            sync1.reverse()
            sync2 = packet[12:16]
            sync2.reverse()
            start_angle_q6 = packet[16:31]
            start_angle_q6.reverse()
            start = packet[31]

            current_packet = {}
            current_packet['sync1'] = sync1.hex
            current_packet['sync2'] = sync2.hex
            current_packet['start_angle_q6'] = start_angle_q6.uint
            current_packet['start_angle'] = current_packet['start_angle_q6'] / 64.0
            current_packet['start_bit'] = start
            current_packet['samples'] = []

            for cabin in chunks(packet[32:], 40):
                angle1 = cabin[32:36] + cabin[0:2]
                angle1.reverse()
                angle2 = cabin[36:40] + cabin[16:18]
                angle2.reverse()
                distance1 = cabin[2:16]
                distance1.reverse()
                distance2 = cabin[18:32]
                distance2.reverse()


                cabin_data = {}
                cabin_data['angle1'] = angle1.int / 64
                cabin_data['distance1'] = distance1.uint
                cabin_data['angle2'] = angle2.int / 64
                cabin_data['distance2'] = distance1.uint

                current_packet['samples'].append({'delta_angle': cabin_data['angle1'], 'distance': cabin_data['distance1']})
                current_packet['samples'].append({'delta_angle': cabin_data['angle2'], 'distance': cabin_data['distance2']})

            if previous_packet is None:
                previous_packet = current_packet
                continue
            for k, sample in enumerate(previous_packet['samples'], start=0):
                angle = (previous_packet['start_angle'] + ((angle_diff(previous_packet['start_angle'], current_packet['start_angle']) * k) / 32) - sample['delta_angle']) % 360
                radian = angle * math.pi / 180
                scan.append({'radian': radian, 'angle': angle, 'distance': sample['distance']})
                if angle < (previous_angle - 1):
                    yield scan[:-1]
                    scan = [scan[-1]] 
                    previous_angle = 0
                else:
                    previous_angle = angle
            previous_packet = current_packet

    def get_device_info(self):
        """
        Retrieves device specific information
        """
        self.send_request(Request.GET_INFO)
        response = self.get_response()
        if response['data_type'] != Response.GET_INFO:
            raise RuntimeError("Invalid response descriptor data type")
        packet = self.read(response['data_length'])
        device_info = {
            'model': packet[0:8].hex,
            'firmware_version': f"{packet[16:24].uint}.{packet[8:16].uint}",
            'hardware_version': f"A1M8-R{packet[24:32].uint}",
            'serial_number': packet[32:128].hex
        }
        return device_info

    def get_device_health(self):
        """
        Checks the device status and error codes

        Returns:
            status: The health status code
        """
        self.send_request(Request.GET_HEALTH)
        descriptor = self.get_response()
        if descriptor['data_type'] != Response.GET_HEALTH:
            raise RuntimeError("Invalid response descriptor data type")
        packet = self.read(descriptor['data_length'])
        health = {
            'status': packet[0:8],
            'error_code': packet[8:24]
        }
        return health

    def get_sample_rate(self):
        """
        Gets the sample period of the device in standard and express mode. Useful when calculating the motor rotation
        speed

        Returns:
            t_standard: The sample period of the device in standard mode
            t_express: The sample period of the device in express mode

        """
        self.send_request(Request.GET_SAMPLERATE)
        descriptor = self.get_response()
        if descriptor['data_type'] != Response.GET_SAMPLERATE:
            raise RuntimeError("Invalid response descriptor data type")
        packet = self.read(descriptor['data_length'])
        sample_rate = {
            'standard': packet[0:16].uintle,
            'express': packet[16:32].uintle,
            'unit': 'uS'
        }
        return sample_rate

    def _get_scan_mode_count(self):
        self.send_request(Request.GET_LIDAR_CONF, Config.RPLIDAR_CONF_SCAN_MODE_COUNT)
        time.sleep(2)
        # print(self.com.read_all())
        # descriptor = self.get_response()
        # if descriptor['data_type'] != Response.GET_SAMPLERATE:
        #     raise RuntimeError("Invalid response descriptor data type")
        # raise NotImplementedError

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
