#include <stdio.h>

#define COMM_PACKET_HEADER	4
#define COMM_PACKET_HEADER_W_CRC COMM_PACKET_HEADER + 2
#define MAX_COMM_PACKET_LEN 58
#define MAX_BLUETOOTH_PACKET_LEN 64
#define PKT_FILE_METADATA_LEN 2
#define HEADER1 0xC3
#define HEADER2 0xFE

void decodePacket();
void decodeBytes(uint8_t *buffer, uint8_t bufferLen);

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

commPacket_t packet;
decodeState_t state = decodeState_t::STATE_HEADER;
uint8_t dataIndex = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void decodeBytes(uint8_t *buffer, uint8_t bufferLen)
{
	uint8_t index = 0;
	while(index < bufferLen)
	{
		switch(state)
		{
			case decodeStatus_t::STATE_HEADER1:
			{
				if( buffer[index] == HEADER1 )
				{
					packet.header1 = buffer[index];
					state = decodeState_t::STATE_HEADER2;
				}
				break;
			}
			case decodeStatus_t::STATE_HEADER2:
			{
				if( buffer[index] == HEADER2 )
				{
					packet.header2 = buffer[index];
					state = decodeState_t::STATE_TYPE;
				}
				else
					state = decodeState_t::STATE_HEADER1; // error
				break;
			}
			case decodeStatus_t::STATE_TYPE:
			{
				if(buffer[index] < packetTypes_t::PKT_COUNT)
				{
					packet.type = buffer[index];
					state = decodeState_t::STATE_LEN;
				}
				else
					state = decodeState_t::STATE_HEADER1; // error
				break;
			}
			case decodeStatus_t::STATE_LEN:
			{
				if(buffer[index] <= MAX_COMM_PACKET_LEN)
				{
					packet.dataLen = buffer[index];
					if(packet.dataLen == 0)
						state = decodeState_t::STATE_CRC1;
					else
					{
						state = decodeState_t::STATE_DATA;
						dataIndex = 0;
					}
				}
				else
					state = decodeState_t::STATE_HEADER1; // error
				break;
			}
			case decodeStatus_t::STATE_DATA:
			{
				if(dataIndex < packet.dataLen)
					packet.data[dataIndex++] = buffer[index];
				else
					state = decodeState_t::STATE_CRC1;
				break;
			}
			case decodeStatus_t::STATE_CRC1:
			{
				((uint_8*)&packet.crc)[0] = buffer[index];
				state = decodeState_t::STATE_CRC2;
				break;
			}
			case decodeStatus_t::STATE_CRC2:
			{
				((uint_8*)&packet.crc)[1] = buffer[index];
				
				if( crc16((uint8_t*)&packet, packet.dataLen + COMM_PACKET_HEADER) == packet.crc)
				{
					decodePacket();
				}
				state = decodeState_t::STATE_HEADER1;
				break;
			}
		}
		
		index++;
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void decodePacket()
{
	switch(packet.type)
	{
		case packetTypes_t::PKT_STRING:
		{
			pktString_t pktString;
			decodeStringPacket(&pPktString);
			//pPktString.str
			//pPktString.strLen			
			break;
		}
		case packetTypes_t::PKT_FILE_METADATA:
		{
			pktFileMetadata_t pktMetadata;
			decodeFileMetadataPacket(&pktMetadata);
			//pktMetadata.packetCount			
			break;
		}
		case packetTypes_t::PKT_FILE_CONTENT:
		{
			pktFileContent_t fileContent;
			decodeFileContentPacket(&fileContent);
			//fileContent.packetNo
			//fileContent.data
			//fileContent.dataLen
			break;
		}
		case packetTypes_t::PKT_FILE_SENT:
		{
			// do something
			break;
		}
		case packetTypes_t::PKT_FILE_REQUEST:
		{
			pktFileRequest_t fileRequest;
			decodeFileRequestPacket(&fileRequest);
			//fileRequest.packetNo
			//fileRequest.data
			//fileRequest.dataLen
			break;
		}
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void createStringPacket(commPacket_t *pCommPacket, const pktString_t* pPktString)
{	
	uint16_t crc;
	
	pCommPacket->header1 = HEADER1;
	pCommPacket->header2 = HEADER2;
	pCommPacket->type = packetTypes_t::PKT_STRING;
	pCommPacket->dataLen = pPktString->strLen;
	memcpy(pCommPacket->data, pPktString->str, pPktString->strLen);
	
	crc = crc16((uint8_t*)pCommPacket, pCommPacket->dataLen + COMM_PACKET_HEADER);
	pCommPacket->crc = crc;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void decodeStringPacket(const commPacket_t *pCommPacket, pktString_t* pPktString)
{
	memcpy(pPktString->str, pCommPacket->data, pCommPacket->dataLen);
	pPktString->str[pCommPacket->dataLen] = 0;
	pPktString->strLen = pCommPacket->dataLen;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void createFileMetadataPacket(commPacket_t *pCommPacket, const pktFileMetadata_t* pPktMetadata)
{
	uint16_t crc;
	
	pCommPacket->header1 = HEADER1;
	pCommPacket->header2 = HEADER2;
	pCommPacket->type = packetTypes_t::PKT_FILE_METADATA;
	pCommPacket->dataLen = PKT_FILE_METADATA_LEN;
	memcpy(pCommPacket->data, &pPktMetadata->packetCount, sizeof(uint16_t));
	
	crc = crc16((uint8_t*)pCommPacket, pCommPacket.dataLen + COMM_PACKET_HEADER);
	pCommPacket.crc = crc;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void decodeFileMetadataPacket(const commPacket_t *pCommPacket, pktFileMetadata_t* pPktMetadata)
{
	memcpy(pktFileMetadata_t->packetCount, pCommPacket->data, sizeof(uint16_t));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void createFileContentPacket(commPacket_t *pCommPacket, const pktFileContent_t* pFileContent)
{
	uint16_t crc;
	
	pCommPacket->header1 = HEADER1;
	pCommPacket->header2 = HEADER2;
	pCommPacket->type = packetTypes_t::PKT_FILE_CONTENT;
	pCommPacket->dataLen = pFileContent->dataLen;
	memcpy(pCommPacket->data, pFileContent->packetNo, sizeof(uint16_t));
	memcpy(pCommPacket->data + sizeof(uint16_t), pFileContent->data, pFileContent->dataLen);
	
	crc = crc16((uint8_t*)pCommPacket, pCommPacket->dataLen + COMM_PACKET_HEADER);
	pCommPacket->crc = crc;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void decodeFileContentPacket(const commPacket_t *pCommPacket, pktFileContent_t* pFileContent)
{
	memcpy(pFileContent->packetNo, pCommPacket->data, sizeof(uint16_t));
	memcpy(pFileContent->data, pCommPacket->data + sizeof(uint16_t), pCommPacket->dataLen);
	pFileContent->dataLen = pCommPacket->dataLen;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void createFileSentPacket(commPacket_t *pCommPacket)
{
	uint16_t crc;
	
	pCommPacket->header1 = HEADER1;
	pCommPacket->header2 = HEADER2;
	pCommPacket->type = packetTypes_t::PKT_FILE_SENT;
	pCommPacket->dataLen = 0;
	
	crc = crc16((uint8_t*)pCommPacket, pCommPacket->dataLen + COMM_PACKET_HEADER);
	pCommPacket->crc = crc;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void createFileRequestPacket(commPacket_t *pCommPacket, const pktFileRequest_t* pFileRequest)
{
	uint16_t crc;
	
	pCommPacket->header1 = HEADER1;
	pCommPacket->header2 = HEADER2;
	pCommPacket->type = packetTypes_t::PKT_FILE_REQUEST;
	pCommPacket->dataLen = pFileRequest->dataLen;
	memcpy(pCommPacket->data, pFileRequest->packetNo, sizeof(uint16_t));
	memcpy(pCommPacket->data + sizeof(uint16_t), pFileRequest->data, pFileRequest->dataLen);
	
	crc = crc16((uint8_t*)pCommPacket, pCommPacket->dataLen + COMM_PACKET_HEADER);
	pCommPacket->crc = crc;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void decodeFileRequestPacket(commPacket_t *pCommPacket, const pktFileRequest_t* pFileRequest)
{
	memcpy(pFileRequest->packetNo, pCommPacket->data, sizeof(uint16_t));
	memcpy(pFileRequest->data, pCommPacket->data + sizeof(uint16_t), pCommPacket->dataLen);
	pFileRequest->dataLen = pCommPacket->dataLen;	
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void sendCommPacket(const commPacket_t* pCommPacket)
{
	uint8_t buffer[MAX_BLUETOOTH_PACKET_LEN];
	memcpy(buffer, (uint8_t*)pCommPacket, pCommPacket->dataLen + COMM_PACKET_HEADER);
	memcpy(buffer + pCommPacket->dataLen + COMM_PACKET_HEADER, (uint8_t*)pCommPacket->crc, sizeof(uint16_t));
	
	bluetooth.send(buffer, pCommPacket->dataLen + COMM_PACKET_HEADER_W_CRC);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------





// send string
pktString_t pktString;
strcpy(pktString.str, "help");
pktString.strLen = 4;

commPacket_t commPacket;
createStringPacket(&commPacket, &pktString);
sendCommPacket(&commPacket);





// send file content
pktFileContent_t pktFileContent;
pktFileContent.packetNo = 1;
memcpy(pktFileContent.data, exp_data[i], sizeof(exp_type) * 5);
pktFileContent.dataLen = sizeof(exp_type) * 5;


commPacket_t commPacket;
createFileContentPacket(&commPacket, &pktString);
sendCommPacket(&commPacket);




//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * Copyright 2001-2010 Georges Menie (www.menie.org)
 * Copyright 2010 Salvatore Sanfilippo (adapted to Redis coding style)
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "crc16.h"

/* CRC16 implementation acording to CCITT standards.
 *
 * Note by @antirez: this is actually the XMODEM CRC 16 algorithm, using the
 * following parameters:
 *
 * Name                       : "XMODEM", also known as "ZMODEM", "CRC-16/ACORN"
 * Width                      : 16 bit
 * Poly                       : 1021 (That is actually x^16 + x^12 + x^5 + 1)
 * Initialization             : 0000
 * Reflect Input byte         : False
 * Reflect Output CRC         : False
 * Xor constant to output CRC : 0000
 * Output for "123456789"     : 31C3
 */

static const uint16_t crc16Table[256] = {
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
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

uint16_t crc16(const uint8_t* buffer, uint8_t len)
{   
  uint16_t crc = 0xDEAD; //seed
  for (uint8_t counter = 0; counter < len; counter++)
    crc = (crc << 8) ^ crc16Table[((crc >> 8) ^ *buffer++) & 0x00FF];
  return crc;
}