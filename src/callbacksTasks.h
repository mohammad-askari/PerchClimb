#ifndef CALLBACKSTASKS_H
#define CALLBACKSTASKS_H

#include <Arduino.h>

extern uint16_t ble_conn_handle;

void tsParser();
void tsBLEConn();
void tsBLELost();
void tsClimbOn();
void tsClimbOff();
void tsMotorUpdate();
void tsDataLogger();
void tsDataTransfer();

#endif