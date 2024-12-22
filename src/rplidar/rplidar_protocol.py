import codecs
import struct

SYNC_BYTE1 = b"\xA5"
SYNC_BYTE2 = b"\x5A"

SEND_MODE_SINGLE = b"\x00"
SEND_MODE_MULTIPLE = b"\x01"

DESCRIPTOR_LENGTH = 7

CMD_STOP = b"\x25"
CMD_RESET = b"\x40"
CMD_SCAN = b"\x20"
CMD_EXPRESS_SCAN = b"\x82"
CMD_FORCE_SCAN = b"\x21"
CMD_GET_INFO = b"\x50"
CMD_GET_HEALTH = b"\x52"
CMD_GET_SAMPLERATE = b"\x59"
CMD_GET_LIDAR_CONF = b"\x84"

CONF_SCAN_MODE_COUNT = 0x00000070
CONF_SCAN_MODE_US_PER_SAMPLE = 0x00000071
CONF_SCAN_MODE_MAX_DISTANCE = 0x00000074
CONF_SCAN_MODE_ANS_TYPE = 0x00000075
CONF_SCAN_MODE_TYPICAL = 0x0000007C
CONF_SCAN_MODE_NAME = 0x0000007F

MODE_STANDARD = 0
MODE_EXPRESS = 1
MODE_BOOST = 2
MODE_STABILITY = 3

ANS_NORMAL = 0x81
ANS_CAPSULED = 0x82
ANS_HQ = 0x83
ANS_ULTRA_CAPSULED = 0x84
ANS_DENSE_CAPSULED = 0x85


VARBITSCALE_X2_SRC_BIT = 9
VARBITSCALE_X4_SRC_BIT = 11
VARBITSCALE_X8_SRC_BIT = 12
VARBITSCALE_X16_SRC_BIT = 14

VARBITSCALE_X2_DEST_VAL = 512
VARBITSCALE_X4_DEST_VAL = 1280
VARBITSCALE_X8_DEST_VAL = 1792
VARBITSCALE_X16_DEST_VAL = 3328

VBS_SCALED_BASE = [
    VARBITSCALE_X16_DEST_VAL,
    VARBITSCALE_X8_DEST_VAL,
    VARBITSCALE_X4_DEST_VAL,
    VARBITSCALE_X2_DEST_VAL,
    0,
]

VBS_SCALED_LVL = [4, 3, 2, 1, 0]

VBS_TARGET_BASE = [
    (0x1 << VARBITSCALE_X16_SRC_BIT),
    (0x1 << VARBITSCALE_X8_SRC_BIT),
    (0x1 << VARBITSCALE_X4_SRC_BIT),
    (0x1 << VARBITSCALE_X2_SRC_BIT),
    0,
]


class Request:
    def __init__(self, command: bytes, payload: bytes | None = None):
        self.command = command
        self.payload = payload
        self.bytes = SYNC_BYTE1 + self.command

        if payload:
            self.bytes += struct.pack("B", len(payload))
            self.bytes += payload
            self.bytes += struct.pack("B", self.checksum(self.bytes))

    def checksum(self, data: bytes) -> int:
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum


class ResponseDescriptor:
    def __init__(self, data: bytes):
        self.data = data

        self.sync_byte1 = data[0]
        self.sync_byte2 = data[1]
        size_q30_length_type = struct.unpack("<L", data[2:6])[0]
        self.response_length = size_q30_length_type & 0x3FFFFFFF
        self.send_mode = size_q30_length_type >> 30
        self.response_type = data[6]


class DeviceInfo:
    def __init__(self, data: bytes):
        self.model = data[0]
        self.firmware_minor = data[1]
        self.firmware_major = data[2]
        self.hardware = data[3]
        self.serialnumber = codecs.encode(data[4:], "hex").upper()
        self.serialnumber = codecs.decode(self.serialnumber, "ascii")

    def __str__(self):
        data = {
            "model": self.model,
            "firmware_minor": self.firmware_minor,
            "firmware_major": self.firmware_major,
            "hardware": self.hardware,
            "serialnumber": self.serialnumber,
        }
        return str(data)


class Health:
    def __init__(self, data: bytes):
        self.status = data[0]
        self.error_code = (data[1] << 8) + data[2]

    def __str__(self):
        data = {"status": self.status, "error_code": self.error_code}
        return str(data)


class Samplerate:
    def __init__(self, data: bytes):
        self.t_standard = data[0] + (data[1] << 8)
        self.t_express = data[2] + (data[3] << 8)

    def __str__(self):
        data = {"t_standard": self.t_standard, "t_express": self.t_express}
        return str(data)


class ScanMode:
    def __init__(self, name: bytes, max_distance: bytes, us_per_sample: bytes, ans_type: bytes):
        self.name = codecs.decode(name[4:-1], "ascii")
        self.max_distance = struct.unpack("<I", max_distance[4:8])[0]
        self.us_per_sample = struct.unpack("<I", us_per_sample[4:8])[0]
        self.ans_type = struct.unpack("<B", ans_type[4:5])[0]

        self.ans_types = {
            ANS_NORMAL: "NORMAL",
            ANS_CAPSULED: "CAPSULED",
            ANS_HQ: "HQ",
            ANS_ULTRA_CAPSULED: "ULTRA_CAPSULED",
            ANS_DENSE_CAPSULED: "DENSE_CAPSULED",
        }

    def __str__(self):
        data = {
            "name": self.name,
            "max_distance": self.max_distance,
            "us_per_sample": self.us_per_sample,
            "ans_type": self.ans_types[self.ans_type],
        }
        return str(data)


class Measurement:

    def __init__(self, raw_bytes=None, measurement_hq=None):
        if raw_bytes is not None:
            self.start_flag = bool(raw_bytes[0] & 0x1)
            self.quality = raw_bytes[0] >> 2
            self.angle = ((raw_bytes[1] >> 1) + (raw_bytes[2] << 7)) / 64.0
            self.distance = (raw_bytes[3] + (raw_bytes[4] << 8)) / 4.0

        elif measurement_hq is not None:
            self.start_flag = True if measurement_hq.start_flag == 0x1 else False
            self.quality = measurement_hq.quality
            self.angle = ((measurement_hq.angle_z_q14 * 90) >> 8) / 64.0
            self.distance = (measurement_hq.dist_mm_q2) / 4.0

    def __str__(self):
        data = {
            "start_flag": self.start_flag,
            "quality": self.quality,
            "angle": self.angle,
            "distance": self.distance,
        }
        return str(data)


class MeasurementHQ:

    def __init__(self, syncBit, angle_q6, dist_q2):
        self.start_flag = syncBit | ((not syncBit) << 1)
        self.quality = (0x2F << 2) if dist_q2 else 0
        self.angle_z_q14 = (angle_q6 << 8) // 90
        self.dist_mm_q2 = dist_q2

    def get_angle(self):
        return self.angle_z_q14 * 90.0 / 16384.0

    def get_distance(self):
        return self.dist_mm_q2 / 4.0


class Cabin:

    def __init__(self, raw_bytes):
        self.distance1 = (raw_bytes[0] >> 2) + (raw_bytes[1] << 6)
        self.distance2 = (raw_bytes[2] >> 2) + (raw_bytes[3] << 6)
        self.d_theta1 = (raw_bytes[4] & 0x0F) + ((raw_bytes[0] & 0x03) << 4)
        self.d_theta2 = (raw_bytes[4] >> 4) + ((raw_bytes[2] & 0x03) << 4)


class ScanCapsule:

    def __init__(self, raw_bytes):
        self.sync_byte1 = (raw_bytes[0] >> 4) & 0xF
        self.sync_byte2 = (raw_bytes[1] >> 4) & 0xF
        self.checksum = (raw_bytes[0] & 0xF) + ((raw_bytes[1] & 0xF) << 4)
        self.start_angle_q6 = raw_bytes[2] + ((raw_bytes[3] & 0x7F) << 8)
        self.start_flag = bool((raw_bytes[3] >> 7) & 0x1)
        self.cabins = list(map(Cabin, [raw_bytes[i : i + 5] for i in range(4, len(raw_bytes), 5)]))

    @classmethod
    def _parse_capsule(self, capsule_prev, capsule_current):

        nodes = []

        currentStartAngle_q8 = capsule_current.start_angle_q6 << 2
        prevStartAngle_q8 = capsule_prev.start_angle_q6 << 2

        diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8)
        if prevStartAngle_q8 > currentStartAngle_q8:
            diffAngle_q8 += 360 << 8

        angleInc_q16 = diffAngle_q8 << 3
        currentAngle_raw_q16 = prevStartAngle_q8 << 8

        for pos in range(len(capsule_prev.cabins)):

            dist_q2 = [0] * 2
            angle_q6 = [0] * 2
            syncBit = [0] * 2

            dist_q2[0] = capsule_prev.cabins[pos].distance1 << 2
            dist_q2[1] = capsule_prev.cabins[pos].distance2 << 2

            angle_offset1_q3 = capsule_prev.cabins[pos].d_theta1
            angle_offset2_q3 = capsule_prev.cabins[pos].d_theta2

            angle_q6[0] = (currentAngle_raw_q16 - (angle_offset1_q3 << 13)) >> 10
            syncBit[0] = 1 if ((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16 else 0
            currentAngle_raw_q16 += angleInc_q16

            angle_q6[1] = (currentAngle_raw_q16 - (angle_offset2_q3 << 13)) >> 10
            syncBit[1] = 1 if ((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16 else 0
            currentAngle_raw_q16 += angleInc_q16

            for cpos in range(2):

                if angle_q6[cpos] < 0:
                    angle_q6[cpos] += 360 << 6
                if angle_q6[cpos] >= (360 << 6):
                    angle_q6[cpos] -= 360 << 6

                node = MeasurementHQ(syncBit[cpos], angle_q6[cpos], dist_q2[cpos])
                nodes.append(node)

        return nodes


class DenseCabin:

    def __init__(self, raw_bytes):
        self.distance = (raw_bytes[0] << 8) + raw_bytes[1]


class ScanDenseCapsule:

    def __init__(self, raw_bytes):
        self.sync_byte1 = (raw_bytes[0] >> 4) & 0xF
        self.sync_byte2 = (raw_bytes[1] >> 4) & 0xF
        self.checksum = (raw_bytes[0] & 0xF) + ((raw_bytes[1] & 0xF) << 4)
        self.start_angle_q6 = raw_bytes[2] + ((raw_bytes[3] & 0x7F) << 8)
        self.start_flag = bool((raw_bytes[3] >> 7) & 0x1)
        self.cabins = list(map(DenseCabin, [raw_bytes[i : i + 2] for i in range(4, len(raw_bytes), 2)]))

    @classmethod
    def _parse_capsule(self, capsule_prev, capsule_current):

        nodes = []

        currentStartAngle_q8 = capsule_current.start_angle_q6 << 2
        prevStartAngle_q8 = capsule_prev.start_angle_q6 << 2

        diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8)
        if prevStartAngle_q8 > currentStartAngle_q8:
            diffAngle_q8 += 360 << 8

        angleInc_q16 = (diffAngle_q8 << 8) // 40
        currentAngle_raw_q16 = prevStartAngle_q8 << 8

        for pos in range(len(capsule_prev.cabins)):

            dist_q2 = 0
            angle_q6 = 0
            syncBit = 0

            syncBit = 1 if (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) else 0

            angle_q6 = currentAngle_raw_q16 >> 10
            if angle_q6 < 0:
                angle_q6 += 360 << 6
            if angle_q6 >= (360 << 6):
                angle_q6 -= 360 << 6
            currentAngle_raw_q16 += angleInc_q16

            dist_q2 = capsule_prev.cabins[pos].distance << 2

            node = MeasurementHQ(syncBit, angle_q6, dist_q2)
            nodes.append(node)

        return nodes


class UltraCabin:

    def __init__(self, raw_bytes):
        self.major = ((int(raw_bytes[1]) & 0xF) << 8) + int(raw_bytes[0])
        self.predict1 = ((int(raw_bytes[2]) & 0x3F) << 4) + ((int(raw_bytes[1]) >> 4) & 0xF)
        self.predict2 = ((int(raw_bytes[3]) & 0xFF) << 2) + ((int(raw_bytes[2]) >> 6) & 0x3)

        if self.predict1 & 0x200:
            self.predict1 |= 0xFFFFFC00
        if self.predict2 & 0x200:
            self.predict2 |= 0xFFFFFC00

    def __str__(self):
        data = {
            "major": hex(self.major),
            "predict1": hex(self.predict1),
            "predict2": hex(self.predict2),
        }
        return str(data)


class ScanUltraCapsule:

    def __init__(self, raw_bytes):
        self.sync_byte1 = (raw_bytes[0] >> 4) & 0xF
        self.sync_byte2 = (raw_bytes[1] >> 4) & 0xF
        self.checksum = (raw_bytes[0] & 0xF) + ((raw_bytes[1] & 0xF) << 4)
        self.start_angle_q6 = raw_bytes[2] + ((raw_bytes[3] & 0x7F) << 8)
        self.start_flag = bool((raw_bytes[3] >> 7) & 0x1)
        self.ultra_cabins = list(map(UltraCabin, [raw_bytes[i : i + 4] for i in range(4, len(raw_bytes), 4)]))

    def __str__(self):
        data = {
            "sync_byte1": hex(self.sync_byte1),
            "sync_byte2": hex(self.sync_byte2),
            "checksum": hex(self.checksum),
            "start_angle_q6": hex(self.start_angle_q6),
            "start_flag": self.start_flag,
            "ultra_cabins": [str(ultra_cabin) for ultra_cabin in self.ultra_cabins],
        }
        return str(data)

    @classmethod
    def _varbitscale_decode(self, scaled):

        scaleLevel = 0

        for i in range(len(VBS_SCALED_BASE)):

            remain = scaled - VBS_SCALED_BASE[i]
            if remain >= 0:
                scaleLevel = VBS_SCALED_LVL[i]
                return (VBS_TARGET_BASE[i] + (remain << scaleLevel), scaleLevel)

        return (0, scaleLevel)

    @classmethod
    def _parse_capsule(self, capsule_prev, capsule_current):

        nodes = []

        currentStartAngle_q8 = capsule_current.start_angle_q6 << 2
        prevStartAngle_q8 = capsule_prev.start_angle_q6 << 2

        diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8)
        if prevStartAngle_q8 > currentStartAngle_q8:
            diffAngle_q8 += 360 << 8

        angleInc_q16 = (diffAngle_q8 << 3) // 3
        currentAngle_raw_q16 = prevStartAngle_q8 << 8

        for pos in range(len(capsule_prev.ultra_cabins)):

            dist_q2 = [0] * 3
            angle_q6 = [0] * 3
            syncBit = [0] * 3

            dist_major = capsule_prev.ultra_cabins[pos].major

            # signed partical integer, using the magic shift here
            # DO NOT TOUCH

            dist_predict1 = capsule_prev.ultra_cabins[pos].predict1
            dist_predict2 = capsule_prev.ultra_cabins[pos].predict2

            dist_major2 = 0

            # prefetch next ...
            if pos == len(capsule_prev.ultra_cabins) - 1:
                dist_major2 = capsule_current.ultra_cabins[0].major
            else:
                dist_major2 = capsule_prev.ultra_cabins[pos + 1].major

            # decode with the var bit scale ...
            dist_major, scalelvl1 = ScanUltraCapsule._varbitscale_decode(dist_major)
            dist_major2, scalelvl2 = ScanUltraCapsule._varbitscale_decode(dist_major2)

            dist_base1 = dist_major
            dist_base2 = dist_major2

            if not (dist_major) and dist_major2:
                dist_base1 = dist_major2
                scalelvl1 = scalelvl2

            dist_q2[0] = dist_major << 2
            if (dist_predict1 == 0xFFFFFE00) or (dist_predict1 == 0x1FF):
                dist_q2[1] = 0
            else:
                dist_predict1 = dist_predict1 << scalelvl1
                dist_q2[1] = ((dist_predict1 + dist_base1) << 2) & 0xFFFFFFFF

            if (dist_predict2 == 0xFFFFFE00) or (dist_predict2 == 0x1FF):
                dist_q2[2] = 0
            else:
                dist_predict2 = dist_predict2 << scalelvl2
                dist_q2[2] = ((dist_predict2 + dist_base2) << 2) & 0xFFFFFFFF

            for cpos in range(3):

                syncBit[cpos] = 1 if (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) else 0

                offsetAngleMean_q16 = int(7.5 * 3.1415926535 * (1 << 16) / 180.0)

                if dist_q2[cpos] >= (50 * 4):

                    k1 = 98361
                    k2 = int(k1 / dist_q2[cpos])

                    offsetAngleMean_q16 = (
                        int(8 * 3.1415926535 * (1 << 16) / 180) - int(k2 << 6) - int((k2 * k2 * k2) / 98304)
                    )

                angle_q6[cpos] = (currentAngle_raw_q16 - int(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10
                currentAngle_raw_q16 += angleInc_q16

                if angle_q6[cpos] < 0:
                    angle_q6[cpos] += 360 << 6
                if angle_q6[cpos] >= (360 << 6):
                    angle_q6[cpos] -= 360 << 6

                node = MeasurementHQ(syncBit[cpos], angle_q6[cpos], dist_q2[cpos])
                nodes.append(node)

        return nodes
