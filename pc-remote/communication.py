# Constants
COMM_PACKET_HEADER = 4
COMM_PACKET_HEADER_W_CRC = COMM_PACKET_HEADER + 2
MAX_COMM_PACKET_LEN = 58
MAX_BLUETOOTH_PACKET_LEN = 64
PKT_FILE_METADATA_LEN = 2
HEADER1 = 0xC3
HEADER2 = 0xFE

# Decode States
STATE_HEADER1 = 0
STATE_HEADER2 = 1
STATE_TYPE = 2
STATE_LEN = 3
STATE_DATA = 4
STATE_CRC1 = 5
STATE_CRC2 = 6

# Packet Types
PKT_STRING = 0
PKT_FILE_METADATA = 1
PKT_FILE_CONTENT = 2
PKT_FILE_SENT = 3
PKT_FILE_REQUEST = 4
PKT_COUNT = 5

class commPacket_t:
    def __init__(self):
        self.header1 = 0
        self.header2 = 0
        self.type = 0
        self.dataLen = 0
        self.data = bytearray(MAX_COMM_PACKET_LEN)
        self.crc = 0

class pktString_t:
    def __init__(self):
        self.str = ""
        self.strLen = 0

class pktFileMetadata_t:
    def __init__(self):
        self.packetCount = 0

class pktFileContent_t:
    def __init__(self):
        self.packetNo = 0
        self.data = bytearray(MAX_COMM_PACKET_LEN)
        self.dataLen = 0

class pktFileRequest_t:
    def __init__(self):
        self.packetNo = 0
        self.data = bytearray(MAX_COMM_PACKET_LEN)
        self.dataLen = 0

class pktFileSend_t:
    def __init__(self):
        self.sent = True


packet = commPacket_t()
state = STATE_HEADER1
dataIndex = 0
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def decodeBytes(buffer):
    decodedPackets = []
    index = 0

    global packet
    global state
    global dataIndex

    while index < len(buffer):
        if state == STATE_HEADER1:
            if buffer[index] == HEADER1:
                packet.header1 = buffer[index]
                state = STATE_HEADER2
        elif state == STATE_HEADER2:
            if buffer[index] == HEADER2:
                packet.header2 = buffer[index]
                state = STATE_TYPE
            else:
                state = STATE_HEADER1 # error
        elif state == STATE_TYPE:
            if buffer[index] < PKT_COUNT:
                packet.type = buffer[index]
                state = STATE_LEN
            else:
                state = STATE_HEADER1 # error
        elif state == STATE_LEN:
            if buffer[index] <= MAX_COMM_PACKET_LEN:
                packet.dataLen = buffer[index]
                if packet.dataLen == 0:
                    state = STATE_CRC1
                else:
                    state = STATE_DATA
                    dataIndex = 0
            else:
                state = STATE_HEADER1 # error
        elif state == STATE_DATA:
            if dataIndex < packet.dataLen:
                packet.data[dataIndex] = buffer[index]
                dataIndex += 1
            else:
                state = STATE_CRC1
        elif state == STATE_CRC1:
            packet.crc = buffer[index]
            state = STATE_CRC2
        elif state == STATE_CRC2:
            packet.crc += buffer[index] << 8
            
            if crc16(packet) == packet.crc:
                decodedPackets.append(decodePacket())
            
            state = STATE_HEADER1
        
        index += 1
    
    return decodedPackets
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def decodePacket():
    global packet
    
    if packet.type == PKT_STRING:
        return decodeStringPacket(packet)
    elif packet.type == PKT_FILE_METADATA:
        return decodeFileMetadataPacket(packet)
    elif packet.type == PKT_FILE_CONTENT:
        return decodeFileContentPacket(packet)
    elif packet.type == PKT_FILE_SENT:
        return pktFileSend_t()
    elif packet.type == PKT_FILE_REQUEST:
        return decodeFileRequestPacket(packet)
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def createStringPacket(pCommPacket, pPktString):	
    pCommPacket.header1 = HEADER1
    pCommPacket.header2 = HEADER2
    pCommPacket.type = PKT_STRING
    pCommPacket.dataLen = pPktString.strLen
    pCommPacket.data = pPktString.str.encode("ascii")
	
    crc = crc16(pCommPacket)
    pCommPacket.crc = crc
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def decodeStringPacket(pCommPacket):
    pPktString = pktString_t()
    pPktString.str = pCommPacket.data.decode("ascii")
    pPktString.strLen = pCommPacket.dataLen
    return pPktString
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def createFileMetadataPacket(pCommPacket, pPktMetadata):
    pCommPacket.header1 = HEADER1
    pCommPacket.header2 = HEADER2
    pCommPacket.type = PKT_FILE_METADATA
    pCommPacket.dataLen = PKT_FILE_METADATA_LEN
    pCommPacket.data = pPktMetadata.packetCount.to_bytes(2, byteorder='little', signed=False)

    crc = crc16(pCommPacket)
    pCommPacket.crc = crc
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def decodeFileMetadataPacket(pCommPacket):
    pPktMetadata = pktFileMetadata_t()
    pktFileMetadata_t.packetCount = int.from_bytes(pCommPacket.data[0:2], byteorder='little', signed=False)
    return pPktMetadata
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def createFileContentPacket(pCommPacket, pFileContent):
    pCommPacket.header1 = HEADER1
    pCommPacket.header2 = HEADER2
    pCommPacket.type = PKT_FILE_CONTENT
    pCommPacket.dataLen = pFileContent.dataLen
    packetNoBytes = pFileContent.packetNo.to_bytes(2, byteorder='little', signed=False)
    pCommPacket.data = pFileContent.data.extends(packetNoBytes)

    crc = crc16(pCommPacket)
    pCommPacket.crc = crc
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def decodeFileContentPacket(pCommPacket):
    pFileContent = pktFileContent_t()
    pFileContent.packetNo = int.from_bytes(pCommPacket.data[0:2], byteorder='little', signed=False)
    pFileContent.data = pCommPacket.data[:2]
    pFileContent.dataLen = len(pFileContent.data)
    return pFileContent
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def createFileSentPacket(pCommPacket):
    pCommPacket.header1 = HEADER1
    pCommPacket.header2 = HEADER2
    pCommPacket.type = PKT_FILE_SENT
    pCommPacket.dataLen = 0

    crc = crc16(pCommPacket)
    pCommPacket.crc = crc

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def createFileRequestPacket(pCommPacket, pFileRequest):
    pCommPacket.header1 = HEADER1
    pCommPacket.header2 = HEADER2
    pCommPacket.type = PKT_FILE_REQUEST
    pCommPacket.dataLen = pFileRequest.dataLen
    packetNoBytes = pFileRequest.packetNo.to_bytes(2, byteorder='little', signed=False)
    pCommPacket.data = pFileRequest.data.extends(packetNoBytes)

    crc = crc16(pCommPacket)
    pCommPacket.crc = crc
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def decodeFileRequestPacket(pCommPacket):
    pFileRequest = pktFileRequest_t()
    pFileRequest.packetNo = int.from_bytes(pCommPacket.data[0:2], byteorder='little', signed=False)
    pFileRequest.data = pCommPacket.data[:2]
    pFileRequest.dataLen = len(pFileRequest.data)
    return pFileRequest
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def convertCommPacketToByteArray(pCommPacket, includeCrc = True):
    buffer = bytearray(MAX_BLUETOOTH_PACKET_LEN)
    dataIndex = 0
    
    buffer[0] = pCommPacket.header1
    buffer[1] = pCommPacket.header2
    buffer[2] = pCommPacket.type
    buffer[3] = pCommPacket.dataLen
    
    dataIndex += 4
    
    for i in range(pCommPacket.dataLen):
        buffer[dataIndex + i] = pCommPacket.data[i]
    
    dataIndex += pCommPacket.dataLen
    
    if includeCrc:
        crcBytes = pCommPacket.crc.to_bytes(2, byteorder='little', signed=False)
        buffer[dataIndex + 0] = crcBytes[0]
        buffer[dataIndex + 1] = crcBytes[1]
        dataIndex += 2
    
    return buffer[:dataIndex]    
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def sendCommPacket(pCommPacket):
    buffer = convertCommPacketToByteArray(pCommPacket)    
    #bluetooth.send(buffer)
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
crc16Table = [
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 ]

def crc16(pCommPacket):
    buffer = convertCommPacketToByteArray(pCommPacket, False)
    crc = 0xDEAD; #seed
    for counter in range(len(buffer)):
        crc = (crc << 8) ^ crc16Table[((crc >> 8) ^ buffer[counter]) & 0x00FF]
        crc &= 0xFFFF
        counter += 1
    return crc
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

