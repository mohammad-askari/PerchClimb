#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>


void processCommandSerial(const char c);
void processCommandBLE(const char c);
void  setLED(const byte *pins, const char mode);
float clipAngle(float angle);
void  setupBLE();
void  setupCLI();
void  setupTasks();


#endif