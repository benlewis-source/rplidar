import time
from typing import Generator

import serial
from serial import Serial

from rplidar.rplidar_protocol import *


class RPLidar(object):
    def __init__(self, port: str, baudrate: int):
        self.serial = Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=2.0,
        )

    def _send_request(self, request: Request):
        self.serial.write(request.bytes)
        self.serial.flush()

    def _read_response_descriptor(self) -> ResponseDescriptor:
        data = self.serial.read(DESCRIPTOR_LENGTH)
        descriptor = ResponseDescriptor(data)
        if descriptor.sync_byte1 != SYNC_BYTE1[0] or descriptor.sync_byte2 != SYNC_BYTE2[0]:
            raise RuntimeError(
                f"Error reading response descriptor. Expected sync bytes '{SYNC_BYTE1[0]:02x}' and '{SYNC_BYTE2[0]:02x}' but got '{descriptor.sync_byte1:02x}' and '{descriptor.sync_byte2:02x}'"
            )
        return descriptor

    def _read_response_data(self, descriptor: ResponseDescriptor) -> bytes:
        data = self.serial.read(descriptor.response_length)
        if len(data) != descriptor.response_length:
            raise RuntimeError(f"Error reading bytes. Expected {descriptor.response_length} bytes but got {len(data)}")
        return data

    def _get_conf(self, payload: bytes) -> bytes:
        self._send_request(Request(CMD_GET_LIDAR_CONF, payload))
        descriptor = self._read_response_descriptor()
        return self._read_response_data(descriptor)

    def connect(self):
        self.serial.open()

    def disconnect(self):
        self.serial.close()

    def isConnected(self):
        return self.serial.is_open()

    def reset(self):
        self._send_request(Request(CMD_RESET))
        time.sleep(0.75)
        print(self.serial.read_all().decode())

    def startScan(self, force: bool = False) -> Generator:
        command = CMD_FORCE_SCAN if force else CMD_SCAN
        self._send_request(Request(command))
        descriptor = self._read_response_descriptor()

        def func():
            while True:
                data = self._read_response_data(descriptor)
                yield Measurement(data)

        return func

    def startScanExpress(self, mode: int) -> Generator:
        self._send_request(Request(CMD_EXPRESS_SCAN, struct.pack("<BI", mode, 0x00000000)))
        descriptor = self._read_response_descriptor()

        if descriptor.response_type == ANS_CAPSULED:
            capsule_type = ScanCapsule
        elif descriptor.response_type == ANS_ULTRA_CAPSULED:
            capsule_type = ScanUltraCapsule
        elif descriptor.response_type == ANS_DENSE_CAPSULED:
            capsule_type = ScanDenseCapsule
        else:
            raise RuntimeError("Error interpreting response type")

        def func():
            data = self._read_response_data(descriptor)
            capsule_prev = capsule_type(data)
            capsule_current = None

            while True:
                data = self._read_response_data(descriptor)
                capsule_current = capsule_type(data)

                nodes = capsule_type._parse_capsule(capsule_prev, capsule_current)
                for index, node in enumerate(nodes):
                    yield Measurement(raw_bytes=None, measurement_hq=node)

                capsule_prev = capsule_current

        return func

    def getScanDataFrames(func: Generator):
        """
        Generates frames of 360* samples
        """
        frame = []
        # Iterate over the packets from the original generator
        for measurement in func():
            if measurement.start_flag:
                # If the start flag is encountered, yield the current frame (if any)
                if frame:
                    yield frame
                # Start a new frame with the start flag (the start flag itself is included in the frame)
                frame = [measurement]
            else:
                # Otherwise, add the packet to the current frame
                frame.append(measurement)

    def stop(self):
        self._send_request(Request(CMD_STOP))
        time.sleep(0.1)
        self.serial.read_all()

    def getHealth(self) -> Health:
        self._send_request(Request(CMD_GET_HEALTH))
        descriptor = self._read_response_descriptor()
        data = self._read_response_data(descriptor)
        return Health(data)

    def getDeviceInfo(self) -> DeviceInfo:
        self._send_request(Request(CMD_GET_INFO))
        descriptor = self._read_response_descriptor()
        data = self._read_response_data(descriptor)
        return DeviceInfo(data)

    def getSampleRate(self):
        self._send_request(Request(CMD_GET_SAMPLERATE))
        descriptor = self._read_response_descriptor()
        data = self._read_response_data(descriptor)
        return Samplerate(data)

    def getScanModeCount(self) -> int:
        data = self._get_conf(struct.pack("<I", CONF_SCAN_MODE_COUNT))
        count = struct.unpack("<H", data[4:6])[0]
        return count

    def getTypicalScanMode(self) -> ScanMode:
        data = self._get_conf(struct.pack("<I", CONF_SCAN_MODE_TYPICAL))
        idx = struct.unpack("<H", data[4:6])[0]
        return ScanMode(
            self._get_conf(struct.pack("<IH", CONF_SCAN_MODE_NAME, idx)),
            self._get_conf(struct.pack("<IH", CONF_SCAN_MODE_MAX_DISTANCE, idx)),
            self._get_conf(struct.pack("<IH", CONF_SCAN_MODE_US_PER_SAMPLE, idx)),
            self._get_conf(struct.pack("<IH", CONF_SCAN_MODE_ANS_TYPE, idx)),
        )

    def getAllSupportedScanModes(self) -> list[ScanMode]:
        modes = []
        for idx in range(self.getScanModeCount()):
            mode = ScanMode(
                self._get_conf(struct.pack("<IH", CONF_SCAN_MODE_NAME, idx)),
                self._get_conf(struct.pack("<IH", CONF_SCAN_MODE_MAX_DISTANCE, idx)),
                self._get_conf(struct.pack("<IH", CONF_SCAN_MODE_US_PER_SAMPLE, idx)),
                self._get_conf(struct.pack("<IH", CONF_SCAN_MODE_ANS_TYPE, idx)),
            )
            modes.append(mode)
        return modes

    def startMotor(self):
        self.serial.dtr = False

    def stopMotor(self):
        self.serial.dtr = True

    def grabScanData(self):
        pass


class RPLidarA1M8(RPLidar):
    def __init__(self, port: int, baudrate: int = 115200):
        super().__init__(port=port, baudrate=baudrate)
