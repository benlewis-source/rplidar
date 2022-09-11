from bitstring import BitArray

class Common:
    START_FLAG = BitArray(b'\xA5')
    START_FLAG_2 = BitArray(b'\x5A')

class Request:
    """
    Available requests
    """
    STOP = BitArray(b'\x25')
    RESET = BitArray(b'\x40')
    SCAN = BitArray(b'\x20')
    EXPRESS_SCAN = BitArray(b'\x82')
    FORCE_SCAN = BitArray(b'\x21')
    GET_INFO = BitArray(b'\x50')
    GET_HEALTH = BitArray(b'\x52')
    GET_SAMPLERATE = BitArray(b'\x59')
    GET_LIDAR_CONF = BitArray(b'\x84')

class Response:
    """
    The last (7th) byte of each response descriptor
    RESET and STOP requests do not have a response.
    """
    SCAN = BitArray(b'\x81')
    EXPRESS_SCAN_LEGACY = BitArray(b'\x82')
    EXPRESS_SCAN_EXTENDED = BitArray(b'\x84')
    EXPRESS_SCAN_DENSE = BitArray(b'\x85')
    FORCE_SCAN = BitArray(b'\x81')
    GET_INFO = BitArray(b'\x04')
    GET_HEALTH = BitArray(b'\x06')
    GET_SAMPLERATE = BitArray(b'\x15')
    GET_LIDAR_CONF = BitArray(b'\x20')


class Config:
    """
    Available configuration entries
    """
    RPLIDAR_CONF_SCAN_MODE_COUNT = BitArray(b'\x70')
    RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE = BitArray(b'\x71')
    RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE = BitArray(b'\x74')
    RPLIDAR_CONF_SCAN_MODE_ANS_TYPE = BitArray(b'\x75')
    RPLIDAR_CONF_SCAN_MODE_TYPICAL = BitArray(b'\x7C')
    RPLIDAR_CONF_SCAN_MODE_NAME = BitArray(b'\x7F')

class Status:
    """
    Statuses
    """
    GOOD: BitArray(b'\x00')
    WARNING: BitArray(b'\x01')
    ERROR: BitArray(b'\x02')

class SendMode:
    """
    Available send modes
    """
    SINGLE = BitArray(b'\x00')
    MULTIPLE = BitArray(b'\x01')
