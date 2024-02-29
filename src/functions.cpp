#include "functions.h"
#include "main.h"
#include "callbacksBLE.h"
#include "callbacksCLI.h"

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
  cmd_ble.setDescription("\tEstablishes connection or does file transfer via BLE");

  // define motor drive command, callback, and relevant arguments
  Command cmd_drive = cli.addCommand("m/otor,d/rive", cliMotorDrive);
  cmd_drive.addPositionalArgument("t/urn/s", "1");
  cmd_drive.addPositionalArgument("p/ow/er", "100");
  cmd_drive.addFlagArgument("r/ev/erse");
  cmd_drive.addFlagArgument("d/ist/ance");
  cmd_drive.setDescription("\tDrives the wing motor according to the parameters.");

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
  cmd_position.setDescription("\tSet the position of one or all servos.");

  // define servo frequency command, callback, and relevant arguments
  Command cmd_freq = cli.addCommand("freq", setFreq);
  cmd_freq.addPositionalArgument("id", "0");
  cmd_freq.addPositionalArgument("freq", "0");
  cmd_freq.setDescription("\tSet the frequency of all the servos.");

  // define servo mode command, callback, and relevant arguments
  Command cmd_mode = cli.addCommand("mode", setMode);
  cmd_mode.addPositionalArgument("id", "0");
  cmd_mode.addFlagArgument("lin");
  cmd_mode.setDescription("\tSet the mode of of all the servos.");

  // define servo offset command, callback, and relevant arguments
  Command cmd_offset = cli.addCommand("ofst", setOffset);
  cmd_offset.addFlagArgument("a1");
  cmd_offset.addFlagArgument("a2");
  cmd_offset.addFlagArgument("a3");
  cmd_offset.addFlagArgument("a4");
  cmd_offset.addFlagArgument("a5");
  cmd_offset.addFlagArgument("a6");
  cmd_offset.addPositionalArgument("o/ffset", "0");
  cmd_offset.setDescription("\tSet the offset of one or multiple servos.");

  // define servo range command, callback, and relevant arguments
  Command cmd_range = cli.addCommand("rng", setRange);
  cmd_range.addFlagArgument("a1");
  cmd_range.addFlagArgument("a2");
  cmd_range.addFlagArgument("a3");
  cmd_range.addFlagArgument("a4");
  cmd_range.addFlagArgument("a5");
  cmd_range.addFlagArgument("a6");
  cmd_range.addPositionalArgument("r/ange", "0");
  cmd_range.setDescription("\tSet the range of one or multiple servos.");

  // define ESC speed command, callback, and relevant arguments
  Command cmd_esc = cli.addCommand("esc", setESC);
  cmd_esc.addPositionalArgument("s/peed");
  cmd_esc.setDescription("\tSet the speed of the ESC.");

  // define experiment duration command, callback, and relevant arguments
  Command cmd_duration = cli.addCommand("expt", setExpDuration);
  cmd_duration.addPositionalArgument("t", "10");
  cmd_duration.setDescription("\tSet the duration of the experiment.");

  // define debug command, callback, and relevant arguments
  Command cmd_debug = cli.addCommand("dbg", debug);
  cmd_debug.setDescription("\tSet the duration of the experiment.");

  // set error callback
  cli.setOnError(cliThrowError);
}

// ———————————————————————— TASKS SETUP & INITIALIZATION ——————————————————————— //
/**
 * @brief Initializes the task scheduler and adds tasks to the scheduler
 **/
void setupTasks() {
  scheduler.init();
	
  scheduler.addTask(ts_parser);
	scheduler.addTask(ts_ble_conn);
	scheduler.addTask(ts_ble_lost);
	scheduler.addTask(ts_climb_off);
	scheduler.addTask(ts_climb_on);
	scheduler.addTask(ts_data_logger);
	
  ts_parser.enable();
}