#ifndef CALLBACKSCLI_H
#define CALLBACKSCLI_H

#include <Arduino.h>
#include <SimpleCLI.h>

void cliHelp(cmd *cmd_ptr);
void cliThrowError(cmd_error *err_ptr);
void cliLogData(cmd *cmd_ptr);
void cliTransferData(cmd *cmd_ptr);
void cliFormatMemory(cmd *cmd_ptr);
void cliEraseFile(cmd *cmd_ptr);
void cliMotorDrive(cmd *cmd_ptr);
void cliMotorHome(cmd *cmd_ptr);
void cliClimb(cmd *cmd_ptr);

#endif