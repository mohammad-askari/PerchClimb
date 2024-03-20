#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>

#define COMM_PACKET_HEADER	4
#define COMM_PACKET_HEADER_W_CRC COMM_PACKET_HEADER + 2
#define MAX_COMM_PACKET_LEN 58
#define MAX_BLUETOOTH_PACKET_LEN 64
#define PKT_FILE_METADATA_LEN 2
#define HEADER1 0xC3
#define HEADER2 0xFE

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
} pktFileMetadata_t;

typedef struct {
	uint16_t packetNo;
	uint8_t data;
	uint8_t dataLen;
} pktFileContent_t;

typedef struct {
	uint16_t packetNo;
	uint8_t data;
	uint8_t dataLen;
} pktFileRequest_t;

void decodePacket();
void decodeBytes(uint8_t *buffer, uint8_t bufferLen);

#endif