#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

void setLED(const byte *pins, const char mode);
void servoSignal(const float &f, const unsigned long &dt,float &R);

#endif