#ifndef CALLBACKSTASKS_H
#define CALLBACKSTASKS_H

#include <Arduino.h>

extern uint16_t ble_conn_handle;

void tsParser();
void tsSensors();
void tsBLEConn();
void tsKill();
void tsClimbOn();
void tsClimbOff();
void tsPreDescent();
void tsDescentOn();
void tsDescentOff();
void tsPreHover();
void tsHoverOn();
void tsHoverOff();
void tsPreUnperch();
void tsUnperchOn();
void tsUnperchOff();
void tsMotorUpdate();
void tsMotorUpdateDisabled();
void tsDataLogger();
void tsDataTransfer();

#endif