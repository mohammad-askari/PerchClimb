#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

void processCommand(const char c, const byte size, byte &idx, char *str);
void setLED(const byte *pins, const char mode);
void setupBLE();
void setupCLI();
void setupTasks();

#endif