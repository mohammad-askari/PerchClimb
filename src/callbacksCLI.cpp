#include "callbacksCLI.h"
#include "main.h"
#include "functions.h"

// ———————————————————————— CLI USAGE HELPER CALLBACK ——————————————————————— //
/**
 * @brief Prints the descriptions of all CLI commands and arguments.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliHelp(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Serial.print(cli.toString());
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

  // print command usage
  if (e.hasCommand()) {
    bool show_description = false;
    Serial.print("\tDid you mean \"");
    Serial.print(e.getCommand().toString(show_description));
    Serial.println("\"?");
  }
}

// ———————————————————— EXPERIMENTAL DATA LOGGER CALLBACK ——————————————————— //
/**
 * @brief Logs the experimental data onto the QSPI memory.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliLogData(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Serial.println("Logging IMU data at 208 Hz for 5 seconds to test.txt file.");
  delay(5200);
  Serial.println("Data logging completed.");
}

// ——————————————————— EXPERIMENTAL DATA TRANSFER COMMANDS —————————————————— //
/**
 * @brief Transfers the logged data to the BLE connected central device.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliTransferData(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  
  // THIS IS A BLOCKING FUNCTION!
  ts_data_transfer.restart();
}

// ——————————————————————————— QSPI MEMORY FORMAT ——————————————————————————— //
/**
 * @brief Formats the on-board 2MB QSPI memory using LittleFS file system.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliFormatMemory(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Serial.print("Formatting the QSPI using LittleFS ...");
  delay(1500);
  Serial.println("Done.");
}

// ————————————————————————————— QSPI FILE ERASE ———————————————————————————— //
/**
 * @brief Erases a selected file or all the logged files on the QSPI memory.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliEraseFile(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Serial.print("File: ");
  int argNum = c.countArgs();
  for (int i = 0; i < argNum; i++) {
    Serial.print(c.getArgument(i).getValue());
  }
  delay(100);
  Serial.println(" successfully deleted.");
}

// —————————————————————————— MOTOR DRIVE COMMANDS —————————————————————————— //
/**
 * @brief //LEVY// Drives the motor as specified by the parameters
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliMotorDrive(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument turn_arg = c.getArgument(0);
  float turn_val = turn_arg.getValue().toFloat();
  Argument is_distance_arg = c.getArgument(3);
  bool is_distance = is_distance_arg.isSet();
  float ticks = 0;
  float corr_factor = 1;
  if (is_distance){
    ticks = (turn_val *12 * gear_ratio / (spool_diameter * PI)) * corr_factor;
  }
  else {
    ticks = turn_val * 12 * gear_ratio;
  }

  Argument power_arg = c.getArgument(1);
  int power_val = power_arg.getValue().toInt();
  int power_byte = map(power_val,0,100,0,pwm_range);

  Argument reverse = c.getArgument(2);
  bool reverse_val = reverse.isSet();

  if (reverse_val) digitalWrite(phase_pin,LOW);
  else digitalWrite(phase_pin,HIGH);

  long initial_pos = encoder.count();
  analogWrite(enable_pin,power_byte);

  int time = millis();

  while ((abs(encoder.count() - initial_pos) < ticks) && (millis()-time) < turn_val*100)
  {
    delay(10);
  }
  analogWrite(enable_pin,0);
  Serial.print("Reached position: ");
  Serial.println(encoder.count());
  Serial.print("Elapsed time: ");
  Serial.println(millis()-time);
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
  bool set_home = set_arg.isSet();
  if (set_home) {
    // encoder.reset_count();
    Serial.println("Current position set to home");
  }

  Argument return_arg = c.getArgument(2);
  bool return_home = return_arg.isSet();
  if (return_home){
    Argument power_arg = c.getArgument(0);
    int power_val = power_arg.getValue().toInt();
    int power_byte = map(power_val,0,100,0,pwm_range);

    long initial_position = encoder.count();
    bool reverse = (initial_position > 0);
    if (reverse) digitalWrite(phase_pin,LOW);
    else digitalWrite(phase_pin,HIGH);

    analogWrite(enable_pin,power_byte);

    int time = millis();

    while ((encoder.count()*(1-2*int(!reverse)) > 0) && (millis()-time) < 10000)
    {
      delay(10);
    }
    analogWrite(0,0);

    Serial.print("Reached position: ");
    Serial.println(encoder.count());
    Serial.print("Elapsed time: ");
    Serial.println(millis()-time);
  }
}

// ———————————————————————————————————————————————————— EXPERIMENT PARAMETERS COMMANDS ———————————————————————————————————————————————————— //
/**
 * @brief //LEVY// Set the position of one or all servos
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setPos(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   
  int servoID = arg0.getValue().toInt();
  
  Argument arg1 = c.getArgument(1);  
  int pos = arg1.getValue().toInt();

  if (servoID == 99){
    Serial.print("All servos");
    for(byte i = 0; i < servo_num; i++) {
      actuator[i].setPosition(pos);
    }
  }  
  else{
    Serial.print("Servo ");
    servoID = constrain(servoID, 0, servo_num-1);
    Serial.print(servoID);
    actuator[servoID].setPosition(pos);
  }

  Serial.print(" to position ");
  Serial.println(pos);

}

/**
 * @brief //LEVY// Set the frequency of all the servos
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setFreq(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   
  int servoID = arg0.getValue().toInt();

  Argument arg1 = c.getArgument(1);  
  float freq = arg1.getValue().toFloat();

  if (servoID == 99){
    Serial.print("All servos");
    for(byte i = 0; i < servo_num; i++) {
      actuator[i].setFrequency(freq);
    }
  }
  else{
    Serial.print("Servo ");
    servoID = constrain(servoID, 0, servo_num-1);
    Serial.print(servoID);
    actuator[servoID].setFrequency(freq);
  }
  Serial.print(" at frequency ");
  Serial.println(freq);
}

/**
 * @brief //LEVY// Set the speed of the ESC
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setESC(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   
  int speed = arg0.getValue().toInt();

  esc_speed = speed;
  esc.arm();
  esc.speed(500);
  Serial.print("ESC speed set to ");
  Serial.println(speed);
}

/**
 * @brief //LEVY// Set the actuation mode of one or all servos (step/linear)
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setMode(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   
  int servoID = arg0.getValue().toInt();
  
  Argument arg1 = c.getArgument(1);  
  bool linear = arg1.isSet();

  if (servoID == 99){
    Serial.print("All servos");
    for(byte i = 0; i < servo_num; i++) {
      if (!linear){
        actuator[i].mode = STEP;
      }
      else{
        actuator[i].mode = LINEAR;
      }
    }
  }  
  else{
    Serial.print("Servo ");
    servoID = constrain(servoID, 0, servo_num-1);
    Serial.print(servoID);
    if (!linear){
      actuator[servoID].mode = STEP;
    }
    else{
      actuator[servoID].mode = LINEAR;
    }
  }

  Serial.print(" set to ");
  Serial.println(!linear ? "STEP" : "LINEAR");
}

/**
 * @brief //LEVY// Set the duration of the experiment
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setExpDuration(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   
  exp_duration = arg0.getValue().toFloat();

  Serial.print("Experiment duration set to ");
  Serial.print(exp_duration);
  Serial.println(" (s)");
}

/**
 * @brief //LEVY// Set the delay of the experiment
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setExpDelay(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   
  int delay = arg0.getValue().toInt();

  exp_delayed = delay;

  Serial.print("Experiment delay set to ");
  Serial.print(delay);
  Serial.println(" (s)");
}

/**
 * @brief //LEVY// Set the offset of one or multiple servos
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setOffset(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   
  int servoID = arg0.getValue().toInt();

  Argument arg1 = c.getArgument(1);  
  int offset = arg1.getValue().toInt();

  if (servoID == 99){
    Serial.print("All servos");
    for(byte i = 0; i < servo_num; i++) {
      actuator[i].setOffset(offset);
    }
  }
  else{
    Serial.print("Servo ");
    servoID = constrain(servoID, 0, servo_num-1);
    Serial.print(servoID);
    actuator[servoID].setOffset(offset);
  }
  Serial.print(" set to offset ");
  Serial.println(offset);
}

/**
 * @brief //LEVY// Set the range of one or multiple servos
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setRange(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   
  int servoID = arg0.getValue().toInt();

  Argument arg1 = c.getArgument(1);  
  int range = arg1.getValue().toInt();

  if (servoID == 99){
    Serial.print("All servos");
    for(byte i = 0; i < servo_num; i++) {
      actuator[i].setRange(range);
    }
  }
  else{
    Serial.print("Servo ");
    servoID = constrain(servoID, 0, servo_num-1);
    Serial.print(servoID);
    actuator[servoID].setRange(range);
  }
  Serial.print(" set to range ");
  Serial.println(range);
}

/**
 * @brief //LEVY// Send the debug data in serial
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void debug(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  for (int i = 0; i < 6; i++)
  {
    actuator[i].print();
  }
}

/**
 * @brief //LEVY// Set the pre-hover ascent parameters
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setPreHover(cmd *cmd_ptr) {
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
  if (hover_use_hooks) {
    Serial.print(", with transition from ");
    Serial.print(transition_esc);
  }
  Serial.print(", for ");
  Serial.print(pre_hover_time);
  Serial.println(" (s)");

}

// —————————————————————————————————————————————— RUN EXPERIMENT COMMANDS —————————————————————————————————————————————————— //

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
}

/**
 * @brief //LEVY// Starts the climbing experiment with the specified delay
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliClimb(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);
  String direction = arg0.getValue();
  direction.toLowerCase();
  
  if (direction == "up") {
    ts_climb_on.restartDelayed(exp_delayed * TASK_SECOND);
  }
  else if (direction == "down") {
    ts_pre_descent.restartDelayed(exp_delayed * TASK_SECOND);
  }

  Serial.print("Climbing ");
  Serial.print(direction);
  Serial.print(" starts in ");
  Serial.print(exp_delayed);
  Serial.println(" (s)");
}

/**
 * @brief //LEVY// Set the pre-hover ascent parameters
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setClimbDown(cmd *cmd_ptr) {
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
  if (is_freefall_mode) 
    Serial.println(" in free-fall mode");
  else {
    Serial.print(" at ");
    Serial.print(descent_freq);
    Serial.println(" Hz");
  }
}

/**
 * @brief //LEVY// Starts the climbing experiment with the specified delay
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setWingOpening(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Argument arg0 = c.getArgument(0);
  Argument arg1 = c.getArgument(1);
  wing_opening_duration = arg0.getValue().toFloat();
  int speed_percent     = arg1.getValue().toInt();
  dc_speed = map(speed_percent,0,100,0,pwm_range);

  // enable the wing opening under motor update task
  is_wing_opening = true;

  Serial.print("Wing opening set to ");
  Serial.print(speed_percent);
  Serial.print("%% speed for ");
  Serial.print(wing_opening_duration);
  Serial.println(" (s)");
}

/**
 * @brief Set the unperch parameters
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setUnperch(cmd *cmd_ptr) {
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
}

/**
 * @brief Kill the mission
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliKill(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! UGLY WAY TO KILL THE MISSION
  ts_ble_lost.restart();
  Serial.println("Mission aborted.");
}

/**
 * @brief //LEVY// Kill the CLIMB UP mission in a smooth way
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void cliKillSmooth(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  ts_climb_off.restart();
  Serial.println("Mission aborted with smooth falloff.");
}