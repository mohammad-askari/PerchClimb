#ifndef CALLBACKS_H
#define CALLBACKS_H

#include <Arduino.h>
#include <SimpleCLI.h>

void processCommand(const char c, const byte size, byte &idx, char *str);
void logData(cmd *cmd_ptr);
void cliHelp(cmd *cmd_ptr);
void throwError(cmd_error *err_ptr);

void formatMemory(cmd *cmd_ptr);
void eraseFile(cmd *cmd_ptr);
void motorDrive(cmd *cmd_ptr);
void motorHome(cmd *cmd_ptr);
void climb(cmd *cmd_ptr);

void bleCallback(cmd *cmd_ptr);
void bleConnect(uint16_t conn_handle);
void bleDisconnect(uint16_t conn_handle, byte reason);

#endif