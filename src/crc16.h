#ifndef CRC16_H
#define CRC16_H

#include <Arduino.h>

uint16_t crc16(const uint8_t* buffer, uint8_t len);

#endif