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
void cliSetClimbDown(cmd *cmd_ptr);
void cliHover(cmd *cmd_ptr);
void cliSetPreHover(cmd *cmd_ptr);
void cliSetUnperch(cmd *cmd_ptr);
void cliUnperch(cmd *cmd_ptr);
void cliSetMode(cmd *cmd_ptr);
void cliSetFreq(cmd *cmd_ptr);
void cliSetPos(cmd *cmd_ptr);
void cliSetESC(cmd *cmd_ptr);
void cliSetExpDuration(cmd *cmd_ptr);
void cliSetExpDelay(cmd *cmd_ptr);
void cliSetOffset(cmd *cmd_ptr);
void cliSetRange(cmd *cmd_ptr);
void cliSetWingOpening(cmd *cmd_ptr);
void cliDebug(cmd *cmd_ptr);
void cliKill(cmd *cmd_ptr);
void cliKillSmooth(cmd *cmd_ptr);

#endif