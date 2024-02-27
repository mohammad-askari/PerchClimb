#include "callbacks.h"

// ——————————————————————— PARSE INCOMING USER INPUTS ——————————————————————— //
/**
 * @brief Processes incoming serial/ble data bytes and parses into the CLI.
 * @param[in] c    single character input from serial/ble buffer
 * @param[in] size size of the processing buffer array
 * @param[in] idx  index position of the processing buffer
 * @param[in] str  pointer to the processing buffer array
 **/
void processCommand(const char c, const byte size, byte &idx, char *str) {
  switch (c) {
    // if a delimeter is received, reset buffer and enable CLI parsing
    case ',':
    case '\r':
    case '\n':
      if (!idx) break;  // skip leading delimeters if buffer is empty
      str[idx] = '\0';  // adding terminating null byte
      cli.parse(str);
      idx = 0;
      break;

    // add non-delimeter regular bytes to the buffer
    default:
      if (!idx && c == ' ') break;  // skip leading space if buffer is empty
      if (idx < (size - 1)) str[idx++] = c;
      break;
  }
}

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
void throwError(cmd_error *err_ptr) {
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

// ———————————————————————— IMU DATA LOGGER CALLBACK ———————————————————————— //
/**
 * @brief Logs the IMU data onto the QSPI memory.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void logData(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Serial.println("Logging IMU data at 208 Hz for 5 seconds to test.txt file.");
  delay(5200);
  Serial.println("Data logging completed.");
}

// ——————————————————————————— QSPI MEMORY FORMAT ——————————————————————————— //
/**
 * @brief Formats the on-board 2MB QSPI memory using LittleFS file system.
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void formatMemory(cmd *cmd_ptr) {
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
void eraseFile(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
  Serial.print("File: ");
  int argNum = c.countArgs();
  for (int i = 0; i < argNum; i++) {
    Serial.print(c.getArgument(i).getValue());
  }
  delay(100);
  Serial.println(" successfully deleted.");
}

// —————————————————————————— BLE CONTROL COMMANDS —————————————————————————— //
/**
 * @brief ...
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void bleCallback(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer
}

// ——————————————————————————— BLE CONNECT ACTIONS —————————————————————————— //
/**
 * @brief Callback invoked when central device connects to the peripheral
 * @param[in] conn_handle connection handle to where this event happens
 **/
void bleConnect(uint16_t conn_handle)
{
  // get the reference to current connection and print connected device name
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  char central_name[32] = {0};
  conn->getPeerName(central_name, sizeof(central_name));
  Serial.print("BLE Connected to "); 
  Serial.println(central_name);
  setLED(led_pin,'G');

  // customization for throughput maximum data transmission speed
  conn->requestPHY();                // change PHY to 2Mbps (BLE v5.0+)
  conn->requestDataLengthUpdate();   // enable data length extension (BLE v4.2+)
  conn->requestMtuExchange(ble_mtu); // change maximum transmission unit
  
  delay(500); // delay a bit for all the request to complete
  // print the current connection parameters
  Serial.print("BLE PHY: "); Serial.println(conn->getPHY());
  Serial.print("BLE DLE: "); Serial.println(conn->getDataLength());
  Serial.print("BLE MTU: "); Serial.println(conn->getMtu());
}

// ————————————————————————— BLE DISCONNECT ACTIONS ————————————————————————— //
/**
 * @brief Callback invoked when a connection is dropped, displaying the reason
 * @param[in] conn_handle connection handle to where this event happens
 * @param[in] reason      BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void bleDisconnect(uint16_t conn_handle, byte reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("BLE Disconnected, reason = 0x");
  Serial.println(reason, HEX);
  setLED(led_pin,'O');
}

// —————————————————————————— MOTOR DRIVE COMMANDS —————————————————————————— //
/**
 * @brief //LEVY// Drives the motor as specified by the parameters
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void motorDrive(cmd *cmd_ptr) {
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
void motorHome(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument set_arg = c.getArgument(1);
  bool set_home = set_arg.isSet();
  if (set_home) {
    encoder.reset_count();
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

// —————————————————————————— MOTOR DRIVE COMMANDS —————————————————————————— //
/**
 * @brief //LEVY// Make the robot climb
 * @param[in] cmd_ptr pointer to the command stuct data type
 **/
void climb(cmd *cmd_ptr) {
  Command c(cmd_ptr);  // wrapper class instance for the pointer

  Argument arg0 = c.getArgument(0);   // thrust [%]
  int throttle = arg0.getValue().toInt();

  Argument arg1 = c.getArgument(1);   // time
  int time = arg1.getValue().toInt();
  Serial.println(time);

  Argument arg2 = c.getArgument(2);   // frequency of wing twist [Hz]
  int freq = arg2.getValue().toInt();

  Argument arg3 = c.getArgument(3);   // bool: use rudder, 0 → no, 1 → yes
  bool use_rudder = arg3.isSet();

  
  esc.speed(throttle);

  // float angle = sin(global_time*freq)*90+90;
  // angle = angle * aileron_range + aileron_offset; 
  // servos[0].write(angle);
  // servos[1].write(angle);

  
  // if (use_rudder){
  //   servos[0].write(-1 * (angle * rudder_range + rudder_offset));
  // }
  
}