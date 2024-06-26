# Import the socket module for networking functionality
import socket

# Define constants for trim values
TRIM_MAX = 1000
TRIM_MIN = -1000

# Initialize a variable to store autopilot status (0 represents off)
isAutoPilotOn = 0

# Define constants for MultiWii Serial Protocol (MSP) headers
MSP_HEADER_IN = "244d3c"  # "$M<"

# Define TCP/IP connection details
#TCP_IP = '192.168.4.1'
TCP_IP = '192.168.0.1'
TCP_PORT = 9060
#TCP_PORT = 23


# Define MSP message types
MSP_FC_VERSION = 3
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206
MSP_SET_MOTOR = 214
MSP_SET_ACC_TRIM = 239
MSP_ACC_TRIM = 240
MSP_EEPROM_WRITE = 250
MSP_SET_POS = 216
MSP_SET_COMMAND = 217
MSP_SET_1WIRE = 243

# Create a TCP socket connection
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((TCP_IP, TCP_PORT))

# Function to send MSP request data to the connected device
def sendRequestMSP(data):
    '''
    client: client object
    data: string created by createPacketMSP()
    '''
    client.send(bytes.fromhex(data))  # converts hex string to byte string

# Function to create MSP packets based on message type and payload
def createPacketMSP(msp, payload):
    '''
    msp (UINT8): message type 
    payload (list): Contains message content, e.g., rc values in case of sendRequestMSP_SET_RAW_RC
    '''
    bf = ""
    bf += MSP_HEADER_IN

    checksum = 0
    if (msp == MSP_SET_COMMAND):
        pl_size = 1  # Payload size
    else:
        pl_size = len(payload) * 2  # Each RC value is 2 bytes

    bf += '{:02x}'.format(pl_size & 0xFF)  # Formats the byte as a hex string
    checksum ^= pl_size  # ^ = XOR

    bf += '{:02x}'.format(msp & 0xFF)
    checksum ^= msp

    for k in payload:
        if (msp == MSP_SET_COMMAND):
            # MSP_SET_COMMAND has only 1 byte payload unlike other MSP commands
            bf += '{:02x}'.format(k & 0xFF)
            checksum ^= k & 0xFF

        else:
            bf += '{:02x}'.format(k & 0xFF)
            checksum ^= k & 0xFF
            bf += '{:02x}'.format((k >> 8) & 0xFF)
            checksum ^= (k >> 8) & 0xFF  # Little-Endian representation (MSB after LSB)

    bf += '{:02x}'.format(checksum)

    return bf

# Functions to send specific MSP commands with their respective payloads
def sendRequestMSP_SET_RAW_RC(channels):
    '''
    channels: list of 8 RC channel values
    '''
    sendRequestMSP(createPacketMSP(MSP_SET_RAW_RC, channels))

def sendRequestMSP_SET_COMMAND(commandType):
    '''
    commandType: integer between 0 and 6 (both inclusive)
    0 : None
    1 : Take-off
    2 : Land
    3 : Back flip
    4 : Front flip
    5 : Right flip
    6 : Left 
    '''
    sendRequestMSP(createPacketMSP(MSP_SET_COMMAND, [commandType]))

def sendRequestMSP_GET_DEBUG(requests):
    # Send multiple MSP requests for debugging purposes
    for i in range(len(requests)):
        sendRequestMSP(createPacketMSP(requests[i], []))

def sendRequestMSP_SET_ACC_TRIM(trim_roll, trim_pitch):
    # Send MSP command to set accelerometer trim values
    sendRequestMSP(createPacketMSP(MSP_ACC_TRIM, [trim_roll, trim_pitch]))

def sendRequestMSP_ACC_TRIM():
    # Send MSP command to request accelerometer trim values
    sendRequestMSP(createPacketMSP(MSP_ACC_TRIM, []))

def sendRequestMSP_EEPROM_WRITE():
    # Send MSP command to write to EEPROM
    sendRequestMSP(createPacketMSP(MSP_EEPROM_WRITE, []))