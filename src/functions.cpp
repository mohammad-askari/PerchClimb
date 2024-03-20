#include "functions.h"
#include "main.h"
#include "callbacksBLE.h"
#include "callbacksCLI.h"
#include "callbacksTasks.h"

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

// ———————————————————————————— LED COLOR CONTROL ——————————————————————————— //
/**
 * @brief Sets the built-in 3-in-1 LED color or turns it off.
 * @param[in] pins array of pin numbers for red, green, blue LEDs
 * @param[in] mode (R)ed, (G)reen, (B)lue, or (O)ff
 **/
void setLED(const byte *pins, const char mode) {
  const byte r = 0;
  const byte g = 1;
  const byte b = 2;

  digitalWrite(pins[r], HIGH);
  digitalWrite(pins[g], HIGH);
  digitalWrite(pins[b], HIGH);

  switch (mode) {
    case 'R':
      digitalWrite(pins[r], LOW);
      break;

    case 'G':
      digitalWrite(pins[g], LOW);
      break;

    case 'B':
      digitalWrite(pins[b], LOW);
      break;
  }
}

// —————————————————— BLE CUSTOMIZATION & ADVERTISING SETUP ————————————————— //
/**
 * @brief Sets up the BLE peripheral device, services, and advertising packet
 **/
void setupBLE() {
  // configure the BLE peripheral device settings and callbacks
  Bluefruit.autoConnLed(true); // blink LED when not connected
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setName("PerchClimb");
  Bluefruit.setTxPower(8);     // check supported values (max 8 dBm)
  Bluefruit.Periph.setConnectCallback(bleConnect);
  Bluefruit.Periph.setDisconnectCallback(bleDisconnect);
  Bluefruit.Periph.setConnInterval(6, 12); // in unit of 0.625 ms (7.5 - 15 ms)

  // configure and start BLE device information service
  bledis.setManufacturer("Seeed Studio");
  bledis.setModel("XIAO BLE nRF52840 Sense");
  bledis.begin();

  // configure and start BLE UART service
  bleuart.begin();

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! // TODO: METHOD TO ADD NEW CHARACTERISTIC
  /*const uint8_t BLEUART_UUID_CHR_TXD[] =
  {
      0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
      0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x60
  };
  BLECharacteristic counterChr = BLECharacteristic(BLEUART_UUID_CHR_TXD);
  counterChr.setProperties(CHR_PROPS_WRITE_WO_RESP);
  counterChr.setPermission(SECMODE_OPEN, SECMODE_OPEN);

  // Set the "size" of the characteristic variable, in bytes. In this case,
  4 bytes/32 bits counterChr.setFixedLen(10);

  // Set a human-readable descriptor; this help when inspecting the service
  counterChr.setUserDescriptor("ASDF");

  // Now 'bind' the characteristic to the service by starting it
  counterChr.begin();*/


  // configure advertising packet and start BLE advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);       // include UART 128-bit UUID
  Bluefruit.ScanResponse.addName();                // no room for name in packet
  Bluefruit.Advertising.restartOnDisconnect(true); // advertise on disconnect
  Bluefruit.Advertising.setInterval(32, 244);      // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);        // fast mode seconds
  Bluefruit.Advertising.start(0);                  // advertise forever
}

// ———————————————————————— CLI COMMANDS DEFINITIONS ———————————————————————— //
/**
 * @brief Defines the CLI commands and their respective callbacks
 **/
void setupCLI() {
  // define CLI usage help command and callback
  Command cmd_help = cli.addCommand("help", cliHelp);
  cmd_help.setDescription("\tShows command info");

  // define QSPI format command and callback
  Command cmd_format = cli.addCommand("format", cliFormatMemory);
  cmd_format.setDescription("\tErases a certain or all the files on QSPI");

  // define QSPI erase command and callback
  Command cmd_erase = cli.addCommand("erase", cliEraseFile);
  cmd_erase.addArgument("n/ame,file/name");
  cmd_erase.setDescription("\tFormats the on-board QSPI memory");

  // define IMU data logger command, callback, and relevant arguments
  Command cmd_log = cli.addCommand("log", cliLogData);
  cmd_log.addArgument("n/ame,file/name");
  cmd_log.addArgument("t/ime", "30");
  cmd_log.addArgument("d/elay", "0");
  cmd_log.addArgument("f/req/uency", "100");
  cmd_log.setDescription("\tLogs IMU data to a file on QSPI");

  // define BLE transfer command, callback, and relevant arguments
  Command cmd_ble = cli.addCommand("transfer", cliTransferData);
  cmd_ble.addFlagArgument("info");
  cmd_ble.addFlagArgument("time");
  cmd_ble.addFlagArgument("imu");
  cmd_ble.addFlagArgument("current");
  cmd_ble.setDescription("\tEstablishes file transfer via BLE");

  // define motor drive command, callback, and relevant arguments
  Command cmd_drive = cli.addCommand("m/otor,d/rive", cliMotorDrive);
  cmd_drive.addPositionalArgument("t/urn/s", "1");
  cmd_drive.addPositionalArgument("p/ow/er", "100");
  cmd_drive.addFlagArgument("r/ev/erse");
  cmd_drive.addFlagArgument("d/ist/ance");
  cmd_drive.setDescription("\tDrives the wing motor based on the parameters.");

  // define motor home command, callback, and relevant arguments
  Command cmd_home = cli.addCommand("h/ome", cliMotorHome);
  cmd_home.addPositionalArgument("p/ow/er", "100");
  cmd_home.addFlagArgument("s/et");
  cmd_home.addFlagArgument("r/et/urn, g/o");
  cmd_home.setDescription("\tSets the reference position for the wing motor and "
                          "brings it back to the refence position if demanded");

  // define servo position command, callback, and relevant arguments
  Command cmd_position = cli.addCommand("goto", setPos);
  cmd_position.addPositionalArgument("id", "0");
  cmd_position.addPositionalArgument("pos", "0");
  cmd_position.setDescription("\tSets the position of one or all servos.");

  // define servo frequency command, callback, and relevant arguments
  Command cmd_freq = cli.addCommand("freq", setFreq);
  cmd_freq.addPositionalArgument("id", "0");
  cmd_freq.addPositionalArgument("f/req", "0");
  cmd_freq.setDescription("\tSets the frequency of all the servos.");

  // define servo mode command, callback, and relevant arguments
  Command cmd_mode = cli.addCommand("mode", setMode);
  cmd_mode.addPositionalArgument("id", "0");
  cmd_mode.addFlagArgument("r/amp");
  cmd_mode.setDescription("\tSets the mode of of all the servos.");

  // define servo offset command, callback, and relevant arguments
  Command cmd_offset = cli.addCommand("offset", setOffset);
  cmd_offset.addPositionalArgument("id", "0");
  cmd_offset.addPositionalArgument("o/ffset", "0");
  cmd_offset.setDescription("\tSets the offset of one or multiple servos.");

  // define servo range command, callback, and relevant arguments
  Command cmd_range = cli.addCommand("range", setRange);
  cmd_range.addPositionalArgument("id", "0");
  cmd_range.addPositionalArgument("r/ange", "100");
  cmd_range.setDescription("\tSets the range of one or multiple servos.");

  // define ESC speed command, callback, and relevant arguments
  Command cmd_esc = cli.addCommand("esc", setESC);
  cmd_esc.addPositionalArgument("s/peed");
  cmd_esc.setDescription("\tSets the speed of the ESC.");

  // define experiment duration command, callback, and relevant arguments
  Command cmd_duration = cli.addCommand("duration", setExpDuration);
  cmd_duration.addPositionalArgument("t", "10");
  cmd_duration.setDescription("\tSets the duration of the experiment.");

  // define experiment duration command, callback, and relevant arguments
  Command cmd_delay = cli.addCommand("delay", setExpDelay);
  cmd_delay.addPositionalArgument("t", "10");
  cmd_delay.setDescription("\tSets the start delay of the experiment.");

  // define kill command with no arguments to stop experiments immediately
  Command cmd_kill = cli.addCommand("a/bort,k/ill", cliKill);
  cmd_kill.setDescription("\tKill command to abort experiments.");

  // define smooth stop command with no arguments to stop experiments immediately
  Command cmd_kill_smooth = cli.addCommand("s/top", cliKillSmooth);
  cmd_kill_smooth.setDescription("\tStop command to stop experiments smoothly.");

  // define the hovering parameters
  Command cmd_pre_hover = cli.addCommand("prehover", setPreHover);
  cmd_pre_hover.addPositionalArgument("esc", "1000");
  cmd_pre_hover.addPositionalArgument("t/ime", "0");
  cmd_pre_hover.addFlagArgument("h/ook/s");
  cmd_pre_hover.addPositionalArgument("transition", "1500");
  cmd_pre_hover.setDescription("\tSets optional pre-hover ascent parameters.");

  // define hover command with no arguments to start delayed experiments
  Command cmd_hover = cli.addCommand("hover", cliHover);
  cmd_hover.setDescription("\tStarts hovering experiment.");

  // define climb down command with no arguments to start delayed experiments
  Command cmd_climb_down = cli.addCommand("descend", setClimbDown);
  cmd_climb_down.addPositionalArgument("pre_esc", "1000");
  cmd_climb_down.addPositionalArgument("transition_esc", "1500");
  cmd_climb_down.addPositionalArgument("post_esc", "1000");
  cmd_climb_down.addPositionalArgument("before", "0");
  cmd_climb_down.addPositionalArgument("after", "0");
  cmd_climb_down.addPositionalArgument("freq", "0");
  cmd_climb_down.addFlagArgument("f/ree/fall");
  cmd_climb_down.setDescription("\tSets climbing down parameters.");

  // define climb command with no arguments to start delayed experiments
  Command cmd_climb = cli.addCommand("climb", cliClimb);
  cmd_climb.addPositionalArgument("direction", "up");
  cmd_climb.setDescription("\tStarts climbing up/down experiment.");

  // define wing opening command
  Command cmd_wing_time = cli.addCommand("w/ing", setWingOpening);
  cmd_wing_time.addPositionalArgument("t/ime", "1");
  cmd_wing_time.addPositionalArgument("s/peed", "100");
  cmd_wing_time.setDescription("\tSets wing opening parameters.");

  // define climb down command with no arguments to start delayed experiments
  Command cmd_set_unperch = cli.addCommand("setunperch", setUnperch);
  cmd_set_unperch.addPositionalArgument("pre_time", "20");
  cmd_set_unperch.addPositionalArgument("pre_esc", "1500");
  cmd_set_unperch.addPositionalArgument("tilt_esc", "1000");
  cmd_set_unperch.addPositionalArgument("takeoff_time", "1");
  cmd_set_unperch.addPositionalArgument("takeoff_esc", "2000");
  cmd_set_unperch.addPositionalArgument("takeoff_pitch", "45");
  cmd_set_unperch.setDescription("\tSets unperching parameters.");

  // define tilt back command and start delayed experiments
  Command cmd_unperch = cli.addCommand("unperch", cliUnperch);
  cmd_unperch.setDescription("\tStarts unperching experiment");

  // define debug command, callback, and relevant arguments
  Command cmd_debug = cli.addCommand("debug", debug);
  cmd_debug.setDescription("\tTurns on/off the debug flag.");

  // set error callback
  cli.setOnError(cliThrowError);


}

// —————————————————————— TASKS SETUP & INITIALIZATION —————————————————————— //
/**
 * @brief Initializes the task scheduler and adds tasks to the scheduler
 **/
void setupTasks() {
  scheduler.init();
	
  scheduler.addTask(ts_parser);
  scheduler.addTask(ts_sensors);
	scheduler.addTask(ts_ble_conn);
	scheduler.addTask(ts_kill);
	scheduler.addTask(ts_climb_on);
	scheduler.addTask(ts_climb_off);
	scheduler.addTask(ts_pre_descent);
	scheduler.addTask(ts_descent_on);
	scheduler.addTask(ts_descent_off);
	scheduler.addTask(ts_pre_hover);
	scheduler.addTask(ts_hover_on);
	scheduler.addTask(ts_hover_off);
	scheduler.addTask(ts_pre_unperch);
  scheduler.addTask(ts_unperch_on);
  scheduler.addTask(ts_unperch_off);
	scheduler.addTask(ts_motor_update);
	scheduler.addTask(ts_data_logger);
	scheduler.addTask(ts_data_transfer);
	
  ts_motor_update.setOnDisable(&tsMotorUpdateDisabled); 
  ts_parser.enable();
  ts_sensors.enable();
}