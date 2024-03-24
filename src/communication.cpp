#include "communication.h"
#include "crc16.h"
#include "main.h"
#include "functions.h"

commPacket_t packet;

// ———————————————— DATA DECODE & VERIFICATION STATE MACHINE ———————————————— //
/**
 * @brief Decodes incoming data, verifies CRC, and starts packet decoding.
 * @param[in] buffer 	pointer to the buffer containing the incoming data
 * @param[in] bufferLen length of the buffer
 **/
void decodeBytes(uint8_t *buffer, uint8_t bufferLen)
{
	static decodeState_t state = decodeState_t::STATE_HEADER1;
	static uint8_t dataIndex = 0;
	uint8_t index = 0;
	
	while(index < bufferLen)
	{
		switch(state)
		{
			case decodeState_t::STATE_HEADER1:
			{
				if (buffer[index] == HEADER1)
				{
					packet.header1 = buffer[index];
					state = decodeState_t::STATE_HEADER2;
				}
				break;
			}
			case decodeState_t::STATE_HEADER2:
			{
				if (buffer[index] == HEADER2)
				{
					packet.header2 = buffer[index];
					state = decodeState_t::STATE_TYPE;
				}
				else
					state = decodeState_t::STATE_HEADER1; // error
				break;
			}
			case decodeState_t::STATE_TYPE:
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
			case decodeState_t::STATE_LEN:
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
			case decodeState_t::STATE_DATA:
			{
				if(dataIndex < packet.dataLen)
				{
					packet.data[dataIndex++] = buffer[index];
					// if we got the last byte, we should switch to CRC1
					if(dataIndex == packet.dataLen)
						state = decodeState_t::STATE_CRC1;
				}
				else
					state = decodeState_t::STATE_CRC1;
				break;
			}
			case decodeState_t::STATE_CRC1:
			{
				((uint8_t*)&packet.crc)[0] = buffer[index];
				state = decodeState_t::STATE_CRC2;
				break;
			}
			case decodeState_t::STATE_CRC2:
			{
				((uint8_t*)&packet.crc)[1] = buffer[index];
				
				uint8_t packetLen = packet.dataLen + COMM_PACKET_HEADER;
				if (crc16((uint8_t*)&packet, packetLen) == packet.crc)
					decodePacket();
				
				state = decodeState_t::STATE_HEADER1;
				break;
			}
		}
		
		index++;
	}
}

// ————————————————————————————— PACKET DECODING ———————————————————————————— //
/**
 * @brief Decodes a checksummed incoming packet based on its type.
 **/
void decodePacket()
{
	Serial.println("Packet received from BLE");
	switch(packet.type)
	{
		case packetTypes_t::PKT_STRING:
		{
			pktString_t pktString;
			decodeStringPacket(&packet, &pktString);
			for(uint8_t i = 0; i < pktString.strLen; i++)
				processCommandBLE(pktString.str[i]);
			break;
		}
		case packetTypes_t::PKT_FILE_METADATA:
		{
			pktFileMetadata_t pktMetadata;
			decodeFileMetadataPacket(&packet, &pktMetadata);
			break;
		}
		case packetTypes_t::PKT_FILE_CONTENT:
		{
			pktFileContent_t fileContent;
			decodeFileContentPacket(&packet, &fileContent);
			break;
		}
		case packetTypes_t::PKT_FILE_SENT:
		{
			// !!!!!!!!!!!!!!!! DO SOMETHING !!!!!!!!!!!!!!!!
			break;
		}
		case packetTypes_t::PKT_FILE_REQUEST:
		{
			pktFileRequest_t fileRequest;
			decodeFileRequestPacket(&packet,&fileRequest);
			break;
		}
	}
}

// ————————————————————————— STRING PACKET CREATION ————————————————————————— //
/**
 * @brief Creates a communication packet of type string.
 * @param[in] pCommPacket pointer to the communication packet
 * @param[in] pPktString  pointer to the string packet type
 **/
void createStringPacket(commPacket_t *pCommPacket, const pktString_t* pPktString)
{
	pCommPacket->header1 = HEADER1;
	pCommPacket->header2 = HEADER2;
	pCommPacket->type = packetTypes_t::PKT_STRING;
	pCommPacket->dataLen = pPktString->strLen;
	memcpy(pCommPacket->data, pPktString->str, pPktString->strLen);
	
	uint8_t packetLen = pCommPacket->dataLen + COMM_PACKET_HEADER;
	pCommPacket->crc = crc16((uint8_t*)pCommPacket, packetLen);
}

// —————————————————————————— STRING PACKET DECODER ————————————————————————— //
/**
 * @brief Decodes an incoming communication packet of type string.
 * @param[in] pCommPacket pointer to the communication packet
 * @param[in] pPktString  pointer to the string packet type
 **/
void decodeStringPacket(const commPacket_t *pCommPacket, pktString_t* pPktString)
{
	memcpy(pPktString->str, pCommPacket->data, pCommPacket->dataLen);
	pPktString->str[pCommPacket->dataLen] = 0;
	pPktString->strLen = pCommPacket->dataLen;
}

// —————————————————————— FILE METADATA PACKET CREATION ————————————————————— //
/**
 * @brief Creates a communication packet of type file metadata.
 * @param[in] pCommPacket  pointer to the communication packet
 * @param[in] pPktMetadata pointer to the file metadata packet type
 **/
void createFileMetadataPacket(commPacket_t *pCommPacket, const pktFileMetadata_t* pPktMetadata)
{
	pCommPacket->header1 = HEADER1;
	pCommPacket->header2 = HEADER2;
	pCommPacket->type = packetTypes_t::PKT_FILE_METADATA;
	pCommPacket->dataLen = PKT_FILE_METADATA_LEN;
	memcpy(pCommPacket->data, &pPktMetadata->packetCount, sizeof(uint16_t));
	
	uint8_t packetLen = pCommPacket->dataLen + COMM_PACKET_HEADER;
	pCommPacket->crc = crc16((uint8_t*)pCommPacket, packetLen);
}

// —————————————————————— FILE METADATA PACKET DECODER —————————————————————— //
/**
 * @brief Decodes an incoming communication packet of type file metadata.
 * @param[in] pCommPacket  pointer to the communication packet
 * @param[in] pPktMetadata pointer to the file metadata packet type
 **/
void decodeFileMetadataPacket(const commPacket_t *pCommPacket, pktFileMetadata_t* pPktMetadata)
{
	memcpy(&pPktMetadata->packetCount, pCommPacket->data, sizeof(uint16_t));
}

// —————————————————————— FILE CONTENT PACKET CREATION —————————————————————— //
/**
 * @brief Creates a communication packet of type file content.
 * @param[in] pCommPacket  pointer to the communication packet
 * @param[in] pFileContent pointer to the file content packet type
 **/
void createFileContentPacket(commPacket_t *pCommPacket, const pktFileContent_t* pFileContent)
{
	pCommPacket->header1 = HEADER1;
	pCommPacket->header2 = HEADER2;
	pCommPacket->type = packetTypes_t::PKT_FILE_CONTENT;
	pCommPacket->dataLen = pFileContent->dataLen + sizeof(uint16_t);
	memcpy(pCommPacket->data, &pFileContent->packetNo, sizeof(uint16_t));
	memcpy(pCommPacket->data + sizeof(uint16_t), pFileContent->data, pFileContent->dataLen);
	
	uint8_t packetLen = pCommPacket->dataLen + COMM_PACKET_HEADER;
	pCommPacket->crc = crc16((uint8_t*)pCommPacket, packetLen);
}

// —————————————————————— FILE CONTENT PACKET DECODER —————————————————————— //
/**
 * @brief Decodes an incoming communication packet of type file content.
 * @param[in] pCommPacket  pointer to the communication packet
 * @param[in] pFileContent pointer to the file content packet type
 **/
void decodeFileContentPacket(const commPacket_t *pCommPacket, pktFileContent_t* pFileContent)
{
	memcpy(&pFileContent->packetNo, pCommPacket->data, sizeof(uint16_t));
	memcpy(&pFileContent->data, pCommPacket->data + sizeof(uint16_t), pCommPacket->dataLen);
	pFileContent->dataLen = pCommPacket->dataLen;
}

// ———————————————————————— FILE SENT PACKET CREATION ——————————————————————— //
/**
 * @brief Creates a confirmation packet to indicate that a file has been sent.
 * @param[in] pCommPacket pointer to the communication packet
 **/
void createFileSentPacket(commPacket_t *pCommPacket)
{
	pCommPacket->header1 = HEADER1;
	pCommPacket->header2 = HEADER2;
	pCommPacket->type = packetTypes_t::PKT_FILE_SENT;
	pCommPacket->dataLen = 0;
	
	uint8_t packetLen = pCommPacket->dataLen + COMM_PACKET_HEADER;
	pCommPacket->crc = crc16((uint8_t*)pCommPacket, packetLen);
}

// —————————————————————— FILE REQUEST PACKET CREATION —————————————————————— //
/**
 * @brief Creates a communication packet of type file request.
 * @param[in] pCommPacket  pointer to the communication packet
 * @param[in] pFileRequest pointer to the file request packet type
 **/
void createFileRequestPacket(commPacket_t *pCommPacket, const pktFileRequest_t* pFileRequest)
{
	pCommPacket->header1 = HEADER1;
	pCommPacket->header2 = HEADER2;
	pCommPacket->type = packetTypes_t::PKT_FILE_REQUEST;
	pCommPacket->dataLen = pFileRequest->dataLen;
	memcpy(pCommPacket->data, &pFileRequest->packetNo, sizeof(uint16_t));
	memcpy(pCommPacket->data + sizeof(uint16_t), &pFileRequest->data, pFileRequest->dataLen);
	
	uint8_t packetLen = pCommPacket->dataLen + COMM_PACKET_HEADER;
	pCommPacket->crc = crc16((uint8_t*)pCommPacket, packetLen);
}

// —————————————————————— FILE REQUEST PACKET DECODER —————————————————————— //
/**
 * @brief Decodes an incoming communication packet of type file request.
 * @param[in] pCommPacket  pointer to the communication packet
 * @param[in] pFileRequest pointer to the file request packet type
 **/
void decodeFileRequestPacket(const commPacket_t *pCommPacket, pktFileRequest_t* pFileRequest)
{
	memcpy(&pFileRequest->packetNo, pCommPacket->data, sizeof(uint16_t));
	memcpy(&pFileRequest->data, pCommPacket->data + sizeof(uint16_t), pCommPacket->dataLen);
	pFileRequest->dataLen = pCommPacket->dataLen;	
}

// ———————————————————————————— PACKET SENDING ———————————————————————————— //
void sendPacketViaBLE(const commPacket_t* pCommPacket)
{
	uint8_t buffer[MAX_BLUETOOTH_PACKET_LEN];
	memcpy(buffer, (uint8_t*)pCommPacket, pCommPacket->dataLen + COMM_PACKET_HEADER);
	memcpy(buffer + pCommPacket->dataLen + COMM_PACKET_HEADER, (uint8_t*)&pCommPacket->crc, sizeof(uint16_t));
	
	bleuart.write(buffer, pCommPacket->dataLen + COMM_PACKET_HEADER_W_CRC);
}

void sendStringAsStringPacketViaBLE(String str)
{
	commPacket_t commPacket;
	pktString_t stringPacket;
	int16_t index = 0, remainingCharacters = str.length(), charactersToSend;

	if(str.length() == 0)
		return;

	do
	{
		charactersToSend = remainingCharacters < 50 ? remainingCharacters : 50;
		
		memcpy(stringPacket.str, str.substring(index, index + charactersToSend).c_str(), charactersToSend);
		stringPacket.strLen = charactersToSend;
		createStringPacket(&commPacket, &stringPacket);
		sendPacketViaBLE(&commPacket);

		remainingCharacters -= charactersToSend;
		index += charactersToSend;

		delay(20);
		
	} while (remainingCharacters > 0);
}