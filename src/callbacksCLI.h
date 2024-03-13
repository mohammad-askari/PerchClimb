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
void setClimbDown(cmd *cmd_ptr);
void cliHover(cmd *cmd_ptr);
void setPreHover(cmd *cmd_ptr);
void setUnperch(cmd *cmd_ptr);
void cliUnperch(cmd *cmd_ptr);
void setMode(cmd *cmd_ptr);
void setFreq(cmd *cmd_ptr);
void setPos(cmd *cmd_ptr);
void setESC(cmd *cmd_ptr);
void setExpDuration(cmd *cmd_ptr);
void setExpDelay(cmd *cmd_ptr);
void setOffset(cmd *cmd_ptr);
void setRange(cmd *cmd_ptr);
void setWingOpening(cmd *cmd_ptr);
void debug(cmd *cmd_ptr);
void cliKill(cmd *cmd_ptr);
void cliKillSmooth(cmd *cmd_ptr);

#endif