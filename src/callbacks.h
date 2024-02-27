#ifndef CALLBACKS_H
#define CALLBACKS_H

#include <Arduino.h>
#include <bluefruit.h>
#include <SimpleCLI.h>
#include <quadrature.h>  //LEVY//
#include <ESC.h>  //LEVY//
#include <Servo.h> //LEVY//

#include "functions.h"

extern const int pwm_range;

extern SimpleCLI cli;
extern BLEUart bleuart;
extern const byte ble_mtu;
extern const byte led_pin[];

extern ESC esc;
extern Servo servos[];

//LEVY// Variable necessary for the motorDrive funtion. 
// Ideally there would also be the pin definition for ENABLE and PHASE
extern Quadrature_encoder<8, 8> encoder;
extern const byte enable_pin, phase_pin;
extern const float gear_ratio;
extern const float spool_diameter;
//LEVY// END

void processCommand(const char c, const byte size, byte &idx, char *str);
void logData(cmd *cmd_ptr);
void cliHelp(cmd *cmd_ptr);
void throwError(cmd_error *err_ptr);

void formatMemory(cmd *cmd_ptr);
void eraseFile(cmd *cmd_ptr);
void motorDrive(cmd *cmd_ptr); //LEVY//
void motorHome(cmd *cmd_ptr); //LEVY//
void climb(cmd *cmd_ptr); //LEVY//

void bleCallback(cmd *cmd_ptr);
void bleConnect(uint16_t conn_handle);
void bleDisconnect(uint16_t conn_handle, byte reason);

#endif