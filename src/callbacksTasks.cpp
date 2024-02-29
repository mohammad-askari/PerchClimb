#include "callbacksTasks.h"
#include "main.h"
#include "functions.h"

// ——————————————————————— PARSE INCOMING USER INPUTS ——————————————————————— //
/**
 * @brief Processes incoming serial/ble data bytes and parses into the CLI.
 **/
void tsParser() {
  // check BLE UART for user input and parse into the CLI
  while (bleuart.available())
  {
    int ch = bleuart.read(); // read a single byte from the BLE UART
    processCommand(ch, buffer_len, buffer_idx, buffer);
  }

  // check serial for user input and parse into the CLI
  while (Serial.available())
  {
    int ch = Serial.read();  // read a single byte from the serial
    processCommand(ch, buffer_len, buffer_idx, buffer);
  }
};

// ———————————————————————— CLIMB TIMER CALLBACK ——————————————————————— //
/**
 * @brief Starts timer for the climbing experiments and syncs servo timers.
 **/
void tsClimbTimer() {

};


void tsClimbOn() {
    
};


void tsClimbOff() {
    
};


void tsDataLogger() {
    
};