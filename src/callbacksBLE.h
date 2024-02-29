#ifndef CALLBACKSBLE_H
#define CALLBACKSBLE_H

#include <Arduino.h>
#include <bluefruit.h>

void bleConnect(uint16_t conn_handle);
void bleDisconnect(uint16_t conn_handle, byte reason);

#endif