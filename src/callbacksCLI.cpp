#include "callbacksCLI.h"
#include "main.h"
#include "functions.h"
#include "communication.h"

// ———————————————————————— CLI USAGE HELPER CALLBACK ——————————————————————— //
/**
 * @brief Prints the descriptions of all CLI commands and arguments.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliHelp(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Serial.print(cli.toString());
  sendStringAsStringPacketViaBLE(cli.toString() + String("\n"));
}

// —————————————————————————— INPUT ERROR CALLBACK —————————————————————————— //
/**
 * @brief Throws error upon inputting wrong command or argument.
 * @param[in] err_ptr pointer to the error struct data type
 **/
void cliThrowError(cmd_error *err_ptr) {
  CommandError e(err_ptr);  // wrapper class instance for the pointer

  // print error message
  Serial.print("ERROR:\t");
  Serial.println(e.toString());

  sendStringAsStringPacketViaBLE(String("ERROR:\t") + e.toString() + String("\n"));

  // print command usage
  if (e.hasCommand()) {
    bool show_description = false;
    Serial.print("\tDid you mean \"");
    Serial.print(e.getCommand().toString(show_description));
    Serial.println("\"?");

    sendStringAsStringPacketViaBLE(String("\tDid you mean \"") + e.getCommand().toString(show_description) + String("\"?\n"));
  }
}

// ———————————————————— EXPERIMENTAL DATA LOGGER CALLBACK ——————————————————— //
/**
 * @brief Logs the experimental data onto the QSPI memory.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliLogData(cmd *cmd_ptr) { // TODO: IMPLEMENT THIS FUNCTION
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  
  Serial.println("Logging IMU data at 208 Hz for 5 seconds to test.txt file.");
  sendStringAsStringPacketViaBLE(String("Logging IMU data at 208 Hz for 5 seconds to test.txt file.\n"));

  delay(5200);
  
  Serial.println("Data logging completed.");
  sendStringAsStringPacketViaBLE(String("Data logging completed.\n"));
}

// ——————————————————— EXPERIMENTAL DATA TRANSFER COMMANDS —————————————————— //
/**
 * @brief Transfers the logged data to the BLE connected central device.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliTransferData(cmd *cmd_ptr) { // TODO: MAKE NON-BLOCKING
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);
  transfer_include_commands = arg0.isSet();

  // THIS IS A BLOCKING FUNCTION!
  ts_data_transfer.restart();
}

// ——————————————————————————— QSPI MEMORY FORMAT ——————————————————————————— //
/**
 * @brief Formats the on-board 2MB QSPI memory using LittleFS file system.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliFormatMemory(cmd *cmd_ptr) { // TODO: IMPLEMENT THIS FUNCTION
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  
  Serial.print("Formatting the QSPI using LittleFS ...");
  sendStringAsStringPacketViaBLE(String("Formatting the QSPI using LittleFS ...\n"));
  
  delay(1500);
  
  Serial.println("Done.");
  sendStringAsStringPacketViaBLE(String("Done.\n"));
}

// ————————————————————————————— QSPI FILE ERASE ———————————————————————————— //
/**
 * @brief Erases a selected file or all the logged files on the QSPI memory.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliEraseFile(cmd *cmd_ptr) { // TODO: IMPLEMENT THIS FUNCTION
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  
  Serial.print("File: ");
  sendStringAsStringPacketViaBLE(String("File: "));

  int argNum = c.countArgs();
  for (int i = 0; i < argNum; i++) {
    Serial.print(c.getArgument(i).getValue());
    sendStringAsStringPacketViaBLE(c.getArgument(i).getValue());
  }
  delay(100);
  Serial.println(" successfully deleted.");
  sendStringAsStringPacketViaBLE(String(" successfully deleted.\n"));
}

// —————————————————————————— MOTOR DRIVE COMMANDS —————————————————————————— //
/**
 * @brief //LEVY// Drives the motor as specified by the parameters
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliMotorDrive(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0    = c.getArgument(0);
  Argument arg1    = c.getArgument(1);
  Argument arg2    = c.getArgument(2);
  Argument arg3    = c.getArgument(3);
  float turn_val   = arg0.getValue().toFloat();
  int power_val    = arg1.getValue().toInt();
  bool is_reverse  = arg2.isSet();
  bool is_distance = arg3.isSet();
  
  int power_byte  = map(power_val,0,100,0,pwm_range);

  // float ticks = 0;
  // float corr_factor = 1;
  // if (is_distance){
  //   ticks = (turn_val *12 * gear_ratio / (spool_diameter * PI)) * corr_factor;
  // }
  // else {
  //   ticks = turn_val * 12 * gear_ratio;
  // }

  if (is_reverse) clutch.reverse();
  else clutch.forward();

// FIXME: REPLACE WITH CLUTCH OBJECT
// long initial_pos = encoder.count();
  clutch.speed(power_byte);

  int time = millis();

  // while ((abs(encoder.count() - initial_pos) < ticks) && (millis()-time) < turn_val*100)
  while ((millis()-time) < turn_val*100)
  {
    delay(10);
  }
  clutch.stop();
  // Serial.print("Reached position: ");
  // Serial.println(encoder.count());
  Serial.print("Elapsed time: ");
  Serial.println(millis()-time);
  sendStringAsStringPacketViaBLE(String("Elapsed time: ") + String(millis()-time) + String("\n"));
}

// —————————————————————————— MOTOR HOME COMMANDS —————————————————————————— //
/**
 * @brief //LEVY// Sets the reference position for the motor and brings it back
 *        to the refence position if demanded
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliMotorHome(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument set_arg = c.getArgument(1);
  bool set_home    = set_arg.isSet();
  if (set_home) {
    // encoder.reset_count();
    String str = "Current position set to home\n";
    Serial.println(str);
    sendStringAsStringPacketViaBLE(str);
  }

// FIXME: REPLACE WITH CLUTCH OBJECT
/*   Argument return_arg = c.getArgument(2);
  bool return_home = return_arg.isSet();
  if (return_home){
    Argument power_arg = c.getArgument(0);
    int power_val = power_arg.getValue().toInt();
    int power_byte = map(power_val,0,100,0,pwm_range);

    long initial_position = encoder.count();
    bool reverse = (initial_position > 0);
    if (reverse) clutch.reverse();
    else clutch.forward();

    clutch.speed(power_byte);

    int time = millis();

    while ((encoder.count()*(1-2*int(!reverse)) > 0) && (millis()-time) < 10000)
    {
      delay(10);
    }
    clutch.stop();

    Serial.print("Reached position: ");
    Serial.println(encoder.count());
    Serial.print("Elapsed time: ");
    Serial.println(millis()-time);
  } */
}

// ————————————————————— EXPERIMENT PARAMETERS COMMANDS ————————————————————— //
/**
 * @brief //LEVY// Sets the position of one or all servos
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetPos(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);
  Argument arg1 = c.getArgument(1);
  int servoID   = arg0.getValue().toInt();
  int pos       = arg1.getValue().toInt();

  if (servoID == 99){
    Serial.print("All servos");
    sendStringAsStringPacketViaBLE(String("All servos"));

    for(byte i = 0; i < servo_num; i++) {
      actuator[i]->setPosition(pos);
    }
  }  
  else{
    servoID = constrain(servoID, 0, servo_num-1);
    actuator[servoID]->setPosition(pos);
    
    Serial.print("Servo ");
    Serial.print(servoID);
    sendStringAsStringPacketViaBLE(String("Servo ") + String(servoID));
  }

  Serial.print(" to position ");
  Serial.println(pos);
  sendStringAsStringPacketViaBLE(String(" to position ") + String(pos) + String("\n"));

}

/**
 * @brief //LEVY// Sets the frequency of all the servos
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetFreq(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);
  Argument arg1 = c.getArgument(1);
  int servoID   = arg0.getValue().toInt();
  float freq    = arg1.getValue().toFloat();

  if (servoID == 99){
    Serial.print("All servos");
    sendStringAsStringPacketViaBLE(String("All servos"));

    for(byte i = 0; i < servo_num; i++) {
      actuator[i]->setFrequency(freq);
    }
  }
  else{
    Serial.print("Servo ");
    sendStringAsStringPacketViaBLE(String("Servo "));
    
    servoID = constrain(servoID, 0, servo_num-1);
    
    Serial.print(servoID);
    sendStringAsStringPacketViaBLE(String(servoID));
    
    actuator[servoID]->setFrequency(freq);
  }
  
  Serial.print(" at frequency ");
  Serial.println(freq);
  sendStringAsStringPacketViaBLE(String(" at frequency ") + String(freq) + String("\n"));
}

/**
 * @brief //LEVY// Sets the speed of the ESC
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetESC(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);   
  esc_speed     = arg0.getValue().toInt();

  Serial.print("ESC speed set to ");
  Serial.println(esc_speed);
  sendStringAsStringPacketViaBLE(String("ESC speed set to ") + String(esc_speed) + String("\n"));
}

/**
 * @brief //LEVY// Sets the actuation mode of one or all servos (step/ramp)
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetMode(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);  
  Argument arg1 = c.getArgument(1);  
  int servoID   = arg0.getValue().toInt();
  bool is_ramp  = arg1.isSet();

  if (servoID == 99){
    Serial.print("All servos");
    sendStringAsStringPacketViaBLE(String("All servos"));

    for(byte i = 0; i < servo_num; i++) {
      if (!is_ramp){
        actuator[i]->setMode(STEP);
      }
      else{
        actuator[i]->setMode(RAMP);
      }
    }
  }  
  else{
    Serial.print("Servo ");
    sendStringAsStringPacketViaBLE(String("Servo "));

    servoID = constrain(servoID, 0, servo_num-1);
    
    Serial.print(servoID);
    sendStringAsStringPacketViaBLE(String(servoID));
    
    if (!is_ramp){
      actuator[servoID]->setMode(STEP);
    }
    else{
      actuator[servoID]->setMode(RAMP);
    }
  }

  Serial.print(" set to ");
  Serial.println(!is_ramp ? "step" : "ramp");
  sendStringAsStringPacketViaBLE(String(" set to ") + String(!is_ramp ? "step" : "ramp") + String("\n"));
}

/**
 * @brief //LEVY// Sets the duration of the experiment
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetExpDuration(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);   
  exp_duration  = arg0.getValue().toFloat();

  Serial.print("Experiment duration set to ");
  Serial.print(exp_duration);
  Serial.println(" (s)");
  sendStringAsStringPacketViaBLE(String("Experiment duration set to ") + String(exp_duration) + String(" (s)\n"));
}

/**
 * @brief //LEVY// Sets the delay of the experiment
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetExpDelay(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);   
  int delay_sec = arg0.getValue().toInt();

  exp_delayed = delay_sec;

  Serial.print("Experiment delay set to ");
  Serial.print(delay_sec);
  Serial.println(" (s)");
  sendStringAsStringPacketViaBLE(String("Experiment delay set to ") + String(delay_sec) + String(" (s)\n"));
}

/**
 * @brief //LEVY// Sets the offset of one or multiple servos
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetOffset(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);
  Argument arg1 = c.getArgument(1);
  int servoID   = arg0.getValue().toInt();
  int offset    = arg1.getValue().toInt();

  if (servoID == 99){
    Serial.print("All servos");
    sendStringAsStringPacketViaBLE(String("All servos"));

    for(byte i = 0; i < servo_num; i++) {
      actuator[i]->setOffset(offset);
    }
  }
  else{
    Serial.print("Servo ");
    sendStringAsStringPacketViaBLE(String("Servo "));

    servoID = constrain(servoID, 0, servo_num-1);
    
    Serial.print(servoID);
    sendStringAsStringPacketViaBLE(String(servoID));
    
    actuator[servoID]->setOffset(offset);
  }

  Serial.print(" set to offset ");
  Serial.println(offset);
  sendStringAsStringPacketViaBLE(String(" set to offset ") + String(offset) + String("\n"));
}

/**
 * @brief //LEVY// Sets the range of one or multiple servos
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetRange(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);
  Argument arg1 = c.getArgument(1);
  int servoID   = arg0.getValue().toInt();
  int range     = arg1.getValue().toInt();

  if (servoID == 99){
    Serial.print("All servos");
    sendStringAsStringPacketViaBLE(String("All servos"));

    for(byte i = 0; i < servo_num; i++) {
      actuator[i]->setRange(range);
    }
  }
  else{
    Serial.print("Servo ");
    sendStringAsStringPacketViaBLE(String("Servo "));

    servoID = constrain(servoID, 0, servo_num-1);
    
    Serial.print(servoID);
    sendStringAsStringPacketViaBLE(String(servoID));
    
    actuator[servoID]->setRange(range);
  }

  Serial.print(" set to range ");
  Serial.println(range);
  sendStringAsStringPacketViaBLE(String(" set to range ") + String(range) + String("\n"));
}

/**
 * @brief //LEVY// Prints the debugging data to serial port
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliDebug(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  for (byte i = 0; i < servo_num; i++) { 
    actuator[i]->print(); 
  }
}

/**
 * @brief //LEVY// Sets the pre-hover ascent parameters
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetPreHover(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0   = c.getArgument(0);
  Argument arg1   = c.getArgument(1);
  Argument arg2   = c.getArgument(2);
  Argument arg3   = c.getArgument(3);
  pre_hover_esc   = arg0.getValue().toInt();
  pre_hover_time  = arg1.getValue().toFloat();
  hover_use_hooks = arg2.isSet();
  transition_esc  = arg3.getValue().toInt();

  Serial.print("Pre-hovering set to ESC speed of ");
  Serial.print(pre_hover_esc);
  sendStringAsStringPacketViaBLE(String("Pre-hovering set to ESC speed of ") + String(pre_hover_esc));

  if (hover_use_hooks) {
    Serial.print(", with transition from ");
    Serial.print(transition_esc);
    sendStringAsStringPacketViaBLE(String(", with transition from ") + String(transition_esc));
  }

  Serial.print(", for ");
  Serial.print(pre_hover_time);
  Serial.println(" (s)");
  sendStringAsStringPacketViaBLE(String(", for ") + String(pre_hover_time) + String(" (s)\n"));
}

/**
 * @brief //LEVY// Sets the pre-hover ascent parameters
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetClimbDown(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0     = c.getArgument(0);
  Argument arg1     = c.getArgument(1);
  Argument arg2     = c.getArgument(2);
  Argument arg3     = c.getArgument(3);
  Argument arg4     = c.getArgument(4);
  Argument arg5     = c.getArgument(5);
  Argument arg6     = c.getArgument(6);
  pre_descent_esc   = arg0.getValue().toInt();
  transition_esc    = arg1.getValue().toInt();
  post_descent_esc  = arg2.getValue().toInt();
  pre_descent_time  = arg3.getValue().toFloat();
  post_descent_time = arg4.getValue().toFloat();
  descent_freq      = arg5.getValue().toFloat();
  is_freefall_mode  = arg6.isSet();

  Serial.print("Pre-descent at ");
  Serial.print(pre_descent_esc);
  Serial.print(" ESC for ");
  Serial.print(pre_descent_time);
  Serial.print(" (s), post-descent at ");
  Serial.print(post_descent_esc);
  Serial.print(" ESC for ");
  Serial.print(post_descent_time);
  Serial.print(" (s), with descent ESC transition of ");
  Serial.print(transition_esc);
  sendStringAsStringPacketViaBLE(String("Pre-descent at ") + String(pre_descent_esc) + String(" ESC for ") + String(pre_descent_time) + 
                                 String(" (s), post-descent at ") + String(post_descent_esc) + String(" ESC for ") + String(post_descent_time) + 
                                 String(" (s), with descent ESC transition of ") + String(transition_esc));
  
  if (is_freefall_mode)
  {
    Serial.println(" in free-fall mode");
    sendStringAsStringPacketViaBLE(String(" in free-fall mode\n"));
  }
  else
  {
    Serial.print(" at ");
    Serial.print(descent_freq);
    Serial.println(" Hz");
    sendStringAsStringPacketViaBLE(String(" at ") + String(descent_freq) + String(" Hz\n"));
  }
}

/**
 * @brief //LEVY// Sets the wing opening parameters
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetWingOpening(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);
  Argument arg1 = c.getArgument(1);
  Argument arg2 = c.getArgument(2);
  wing_opening_duration = arg0.getValue().toFloat();
  int speed_percent     = arg1.getValue().toInt();
  is_opening_reverse    = arg2.isSet();
  const char* direction_str = is_opening_reverse ? "closing" : "opening";

  // set the phase pin to the desired direction // FIXME: REPLACE WITH CLUTCH
  if (is_opening_reverse) 
    clutch.reverse();
  else 
    clutch.forward();

  // enable the wing opening under motor update task
  is_wing_opening = true;

  Serial.print("Wing ");
  Serial.print(direction_str);
  Serial.print(" set to ");
  Serial.print(speed_percent);
  Serial.print("% speed for ");
  Serial.print(wing_opening_duration);
  Serial.println(" (s)");
  sendStringAsStringPacketViaBLE(String("Wing opening set to ") + String(speed_percent) + String("% speed for ") + String(wing_opening_duration) + String(" (s)\n"));
}

/**
 * @brief Sets the unperching experiment parameters
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliSetUnperch(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0        = c.getArgument(0);
  Argument arg1        = c.getArgument(1);
  Argument arg2        = c.getArgument(2);
  Argument arg3        = c.getArgument(3);
  Argument arg4        = c.getArgument(4);
  Argument arg5        = c.getArgument(5);
  pre_unperch_duration = arg0.getValue().toFloat();
  pre_unperch_esc      = arg1.getValue().toInt();
  tilt_esc             = arg2.getValue().toInt();
  takeoff_duration     = arg3.getValue().toFloat();
  takeoff_esc          = arg4.getValue().toInt();
  takeoff_pitch        = arg5.getValue().toFloat();

  Serial.print("Pre-unperch at ");
  Serial.print(pre_unperch_esc);
  Serial.print(" ESC for ");
  Serial.print(pre_unperch_duration);
  Serial.print(" (s), tilt back at ");
  Serial.print(tilt_esc);
  Serial.print(" ESC, takeoff at ");
  Serial.print(takeoff_esc);
  Serial.print(" ESC for ");
  Serial.print(takeoff_duration);
  Serial.print(" (s), at the desired ");
  Serial.print(takeoff_pitch);
  Serial.println(" (deg) pitch");
  sendStringAsStringPacketViaBLE(String("Pre-unperch at ") + String(pre_unperch_esc) + String(" ESC for ") + String(pre_unperch_duration) + 
                                 String(" (s), tilt back at ") + String(tilt_esc) + String(" ESC, takeoff at ") + String(takeoff_esc) + 
                                 String(" ESC for ") + String(takeoff_duration) + String(" (s), at the desired ") + String(takeoff_pitch) + String(" (deg) pitch\n"));
}

// ————————————————————————— RUN EXPERIMENT COMMANDS ———————————————————————— //

/**
 * @brief //LEVY// Starts the climbing experiment with the specified delay
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliHover(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  ts_pre_hover.restartDelayed(exp_delayed * TASK_SECOND);
  
  Serial.print("Hovering starts in ");
  Serial.print(exp_delayed);
  Serial.println(" (s)");
  sendStringAsStringPacketViaBLE(String("Hovering starts in ") + String(exp_delayed) + String(" (s)\n"));
}

/**
 * @brief //LEVY// Starts the climbing experiment with the specified delay
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliClimb(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0    = c.getArgument(0);
  String direction = arg0.getValue();
  direction.toLowerCase();

  // disable the wing loosening by default, unless activated by wing command
  climb_wing_loosening = false;
  
  if (direction == "up") {
    ts_pre_climb.restartDelayed(exp_delayed * TASK_SECOND);
  }
  else if (direction == "down") {
    ts_pre_descent.restartDelayed(exp_delayed * TASK_SECOND);
  }

  Serial.print("Climbing ");
  Serial.print(direction);
  Serial.print(" starts in ");
  Serial.print(exp_delayed);
  Serial.println(" (s)");
  sendStringAsStringPacketViaBLE(String("Climbing ") + direction + String(" starts in ") + String(exp_delayed) + String(" (s)\n"));
}

/**
 * @brief Starts the unperching experiment with the specified delay
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliUnperch(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  
  ts_pre_unperch.restartDelayed(exp_delayed * TASK_SECOND);
  
  Serial.print("Unperching starts in ");
  Serial.print(exp_delayed);
  Serial.println(" (s)");
  sendStringAsStringPacketViaBLE(String("Unperching starts in ") + String(exp_delayed) + String(" (s)\n"));
}

/**
 * @brief Kill the mission
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliKill(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  ts_kill.restart();
  Serial.println("Mission aborted.");
  sendStringAsStringPacketViaBLE(String("Mission aborted.\n"));
}

/**
 * @brief //LEVY// Kill the CLIMB UP mission in a smooth way
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliKillSmooth(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  ts_climb_off.restart(); // FIXME: only works for climb up
  Serial.println("Mission aborted with smooth falloff.");
  sendStringAsStringPacketViaBLE(String("Mission aborted with smooth falloff.\n"));
}