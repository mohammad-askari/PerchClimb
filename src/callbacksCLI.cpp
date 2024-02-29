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
  
  Argument info    = c.getArgument("info");
  Argument time    = c.getArgument("time");
  Argument imu     = c.getArgument("imu");
  Argument current = c.getArgument("current");
  bool is_info     = info.isSet();
  bool is_time     = time.isSet();
  bool is_imu      =  imu.isSet();
  bool is_current  = current.isSet();
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
  float correction_factor = 1;
  if (is_distance){
    ticks = (turn_val *12 * gear_ratio / (spool_diameter * PI)) * correction_factor;
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

  long initial_position = encoder.count();
  analogWrite(enable_pin,power_byte);

  int time = millis();

  while ((abs(encoder.count() - initial_position) < ticks) && (millis()-time) < 5000)
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
 * @brief //LEVY// Sets the reference position for the motor and brings it back to the
 *        refence position if demanded
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

    while ((encoder.count() * (1 - 2*int(!reverse)) > 0) && (millis()-time) < 10000)
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

// —————————————————————————— EXPERIMENT COMMANDS —————————————————————————— //
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
    for(byte i = 0; i < 6; i++) {
      actuator[i].setPosition(pos);
    }
  }  
  else{
    Serial.print("Servo ");
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
    Serial.println("All servos");
    for(byte i = 0; i < 6; i++) {
      actuator[i].setFrequency(freq);
    }
  }
  else{
    actuator[servoID].setFrequency(freq);
    Serial.print("Servo ");
    Serial.print(servoID);
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
    for(byte i = 0; i < 6; i++) {
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
    Serial.print(servoID);
    if (!linear){
      actuator[servoID].mode = STEP;
    }
    else{
      actuator[servoID].mode = LINEAR;
    }

  }

  Serial.print("Set to ");
  Serial.println(!linear ? "STEP" : "LINEAR");
}

/**
 * @brief //LEVY// Set the duration of the experiment
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setExpDuration(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   
  int duration = arg0.getValue().toInt();

  exp_duration = duration;

}

/**
 * @brief //LEVY// Set the speed of the wing opening DC motor
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setDCSpeed(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   
  int speed = arg0.getValue().toInt();

  dc_speed = speed;

}

/**
 * @brief //LEVY// Set the offset of one or multiple servos
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setOffset(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  bool servoID[6];
  for (int i = 0; i < 6; i++){
    Argument arg = c.getArgument(i);
    servoID[i] = arg.isSet();
  }
  
  Argument ofst_arg = c.getArgument(7);
  int offset = ofst_arg.getValue().toInt();
  
  for (int i = 0; i < 6; i++)
  {
    if (servoID[i]){
      actuator[i].offset = offset;
    }
  }
}

/**
 * @brief //LEVY// Set the range of one or multiple servos
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void setRange(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  bool servoID[6];
  for (int i = 0; i < 6; i++){
    Argument arg = c.getArgument(i);
    servoID[i] = arg.isSet();
  }
  
  Argument rng_arg = c.getArgument(7);
  int range = rng_arg.getValue().toInt();
  
  for (int i = 0; i < 6; i++)
  {
    if (servoID[i]){
      actuator[i].range = range;
    }
  }
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