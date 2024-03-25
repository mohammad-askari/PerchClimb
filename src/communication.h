#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <WString.h>

#define COMM_PACKET_HEADER	4
#define COMM_PACKET_HEADER_W_CRC COMM_PACKET_HEADER + 2
#define MAX_COMM_PACKET_LEN 238 // MAX_BLUETOOTH_PACKET_LEN - COMM_PACKET_HEADER - 2 (CRC)
#define MAX_FILECONTENT_DATALEN 235
#define MAX_BLUETOOTH_PACKET_LEN 244
#define PKT_FILE_METADATA_LEN 3
#define HEADER1 0xC3
#define HEADER2 0xFE
#define FILE_TYPE_SIMPLE 1
#define FILE_TYPE_EXTENDED 2

typedef enum {
	STATE_HEADER1,
	STATE_HEADER2,
	STATE_TYPE,
	STATE_LEN,
	STATE_DATA,
	STATE_CRC1,
	STATE_CRC2
} decodeState_t;

typedef struct {
	uint8_t header1;
	uint8_t header2;
	uint8_t type;
	uint8_t dataLen;
	uint8_t data[MAX_COMM_PACKET_LEN];
	uint16_t crc;
} commPacket_t;

typedef enum {
	PKT_STRING,
	PKT_FILE_METADATA,
	PKT_FILE_CONTENT,
	PKT_FILE_SENT,
	PKT_FILE_REQUEST,
	PKT_COUNT
} packetTypes_t;

typedef struct {
	uint8_t str[MAX_COMM_PACKET_LEN];
	uint8_t strLen;
} pktString_t;

typedef struct {
	uint16_t packetCount;
	uint8_t filetype;
} pktFileMetadata_t;

typedef struct {
	uint16_t packetNo;
	uint8_t filetype;
	uint8_t data[MAX_FILECONTENT_DATALEN];
	uint8_t dataLen;
} pktFileContent_t;

typedef struct {
	uint16_t packetNo;
	uint8_t data[MAX_FILECONTENT_DATALEN];
	uint8_t dataLen;
} pktFileRequest_t;

void decodePacket();
void decodeBytes(uint8_t *buffer, uint8_t bufferLen);
void createStringPacket(commPacket_t *pCommPacket, const pktString_t* pPktString);
void decodeStringPacket(const commPacket_t *pCommPacket, pktString_t* pPktString);
void createFileMetadataPacket(commPacket_t *pCommPacket, const pktFileMetadata_t* pPktMetadata);
void decodeFileMetadataPacket(const commPacket_t *pCommPacket, pktFileMetadata_t* pPktMetadata);
void createFileContentPacket(commPacket_t *pCommPacket, const pktFileContent_t* pFileContent);
void decodeFileContentPacket(const commPacket_t *pCommPacket, pktFileContent_t* pFileContent);
void createFileSentPacket(commPacket_t *pCommPacket);
void createFileRequestPacket(commPacket_t *pCommPacket, const pktFileRequest_t* pFileRequest);
void decodeFileRequestPacket(const commPacket_t *pCommPacket, pktFileRequest_t* pFileRequest);
void sendPacketViaBLE(const commPacket_t* pCommPacket);
void sendStringAsStringPacketViaBLE(String str);

#endif