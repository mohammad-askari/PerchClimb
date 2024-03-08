/**
 * @file main.cpp
 * @author Mohammad Askari (mohammad.askari@epfl.ch)
 * @brief This program ...
 * @version 1.0
 * @date 2024-02
 *
 * @copyright Copyright (c) 2023, Mohammad Askari, Laboratory of Intelligent
 *            Systems (LIS), EPFL, Lausanne, Switzerland (https://lis.epfl.ch).
 *
 **/
// —————————————————————————————————————————————————————————————————————————— //
//                                DEPENDENCIES                                //
// —————————————————————————————————————————————————————————————————————————— //
#include <TScheduler.hpp>
#include "main.h"
#include "functions.h"
#include "callbacksBLE.h"
#include "callbacksCLI.h"
#include "callbacksTasks.h"

// __________________________  MAIN FUNCTION FLAGS  _________________________ //
bool DEBUG = false;   // enables extra serial connection in debug mode
bool MANUAL = false; // enables manual pilotting using transmitter inputs

// ————————————————————————————— BOARD VARIABLES ———————————————————————————— //
const int adc_res = 12;                    // xiao's ADC resolution (12-bit max)
const int pwm_res = 8;                     // xiao's PWM resolution (16-bit max)
const int adc_range = pow(2, adc_res) - 1; // xiao's ADC maximum value
const int pwm_range = pow(2, pwm_res) - 1; // xiao's PWM maximum value
const long baud_rate = 115200;             // serial data rate (bits per second)

const byte led_pin[] = {LED_RED, LED_GREEN, LED_BLUE}; // 3-in-1 LED pins

// —————————————————————————————— BLE VARIABLES ————————————————————————————— //
BLEDis bledis;                     // BLE service for device information
BLEUart bleuart;                   // BLE service for UART communication
const byte ble_dle  = 251;         // BLE data length (v4.2+: 251, else: 27)
const byte ble_mtu  = ble_dle - 4; // BLE maximum transmission unit
byte ble_packet_len = ble_mtu - 3; // BLE maximum packet/buffer length

// —————————————————————————————— IMU VARIABLES ————————————————————————————— //
LSM6DS3 imu(I2C_MODE, 0x6A);  // I2C device address
Madgwick filter;

// ————————————————————————————— SERVO VARIABLES ———————————————————————————— //
// ————— ORDER: aileron, elevator, rudder, clutch, body hook, tail hook ————— //
const byte    servo_pin[]    = {     7,      2,      3,      4,      5,      6};
const int     servo_offset[] = {    +0,     +0,     +0,      0,     +9,     +5};
const int     servo_range[]  = {  +100,   +100,   +100,    +15,    +30,    +15};
const float   servo_freq[]   = {    +0,     +0,     +0,     +0,     +0,     +0};
const drive_t servo_linear[] = {  STEP,   STEP,   STEP,   STEP,   STEP,   STEP};
const byte    servo_num      = sizeof(servo_pin) / sizeof(servo_pin[0]);
Actuator actuator[servo_num];

// —————————————————————————————— ESC VARIABLES ————————————————————————————— //
const byte esc_pin = 1;                       // ESC PWM pin
const int  esc_min = 1000;                    // ESC minimum speed pulse [μs]
const int  esc_max = 2000;                    // ESC maximum speed pulse [μs]
const int  esc_arm = 500;                     // ESC arm value pulse [μs]
int esc_speed;                                // ESC speed parameter [μs]
ESC esc(esc_pin, esc_min, esc_max, esc_arm);  // ESC motor object

// ————————————————————— WING MOTOR & ENCODER VARIABLES ————————————————————— //
const byte encoder_pin[] = {8, 8}; // quadrature encoder pins
const byte phase_pin  = 9;         // DC motor direction control pin
const byte enable_pin = 10;        // DC motor speed control PWM pin
const float gear_ratio = 297.92;   // DC motor gear ratio (faster motor: 150.58)
const float spool_diameter = 10;   // wing-opening mechanism spool diameter [mm]
int dc_speed;                      // DC motor variable for adjusting speed
Quadrature_encoder<8, 8> encoder;

// ———————————————————————— CURRENT SENSOR VARIABLES ———————————————————————— //
const byte current_pin = A0;

// ———————————————————————————— PARSER VARIABLES ———————————————————————————— //
SimpleCLI cli;                       // command line interface (CLI) object
const byte buffer_len = ble_mtu - 2; // size of the input buffer characters
byte buffer_idx;                     // position index variable for the buffer
char buffer[buffer_len];             // CLI buffer array to parse user inputs

// ——————————————————————— EXPERIMENTAL DATA VARIABLES —————————————————————— //
const int drop_freq = 10;                 // gradual ESC speed drop rate [Hz]
const int move_freq = 100;                // motors movement update rate [Hz]
const int log_freq  = 100;                // data logging frequency [Hz]
const int log_max   = 60;                 // maximum data logging duration [s]
const int data_len  = log_max * log_freq; // maximum size of data arrays
int   data_idx = 0;                       // position index of the data arrays
char  exp_info[200];                      // experimental information string
float exp_duration = 10;                  // experimental duration [s]
int   exp_delayed = 10;                   // experimental start delay [s]
unsigned long start_time;                 // start time of the experiment
exp_data_t exp_data[data_len];            // experimental data array

// —————————————————————— EXPERIMENT-SPECIFIC VARIABLES ————————————————————— //
float pre_hover_time;                       // pre-hover ascent time [s]
int   pre_hover_esc;                        // pre-hover ESC speed [μs]
bool  hover_use_hooks;                      // flag to set hooks usage for
int   transition_esc;                       // transition start ESC speed

float pre_descent_time;                     // pre-descent  hover time [s]
int   pre_descent_esc;                      // pre-descent  ESC speed [μs]
float post_descent_time;                    // post-descent hover time [s]
int   post_descent_esc;                     // post-descent ESC speed [μs]
float descent_freq;                         // descent frequency [Hz]
bool  is_freefall_mode;                     // flag to enable freefall mode

// ———————————————————————— TASK SCHEDULER VARIABLES ———————————————————————— //
// ———— TASK PARAMETERS: interval [ms/μs], #executions, callback function ——— //
TsTask ts_parser       (TASK_IMMEDIATE,        TASK_FOREVER, &tsParser);
TsTask ts_ble_conn     (TASK_IMMEDIATE,        TASK_ONCE,    &tsBLEConn);
TsTask ts_ble_lost     (TASK_IMMEDIATE,        TASK_ONCE,    &tsBLELost);
TsTask ts_climb_on     (TASK_IMMEDIATE,        TASK_ONCE,    &tsClimbOn);
TsTask ts_climb_off    (TASK_IMMEDIATE,        TASK_ONCE,    &tsClimbOff);
TsTask ts_pre_descent  (TASK_IMMEDIATE,        TASK_ONCE,    &tsPreDescent);
TsTask ts_descent_on   (TASK_SECOND/drop_freq, TASK_FOREVER, &tsDescentOn);
TsTask ts_descent_off  (TASK_IMMEDIATE,        TASK_ONCE,    &tsDescentOff);
TsTask ts_pre_hover    (TASK_IMMEDIATE,        TASK_ONCE,    &tsPreHover);
TsTask ts_hover_on     (TASK_SECOND/drop_freq, TASK_FOREVER, &tsHoverOn);
TsTask ts_hover_off    (TASK_IMMEDIATE,        TASK_ONCE,    &tsHoverOff);
TsTask ts_motor_update (TASK_SECOND/move_freq, TASK_FOREVER, &tsMotorUpdate);
TsTask ts_data_logger  (TASK_SECOND/log_freq,  TASK_FOREVER, &tsDataLogger);
TsTask ts_data_transfer(TASK_HOUR,             TASK_ONCE,    &tsDataTransfer);
TsScheduler scheduler; // scheduler object to run tasks in order


// —————————————————————————————————————————————————————————————————————————— //
//                               SETUP FUNCTION                               //
// —————————————————————————————————————————————————————————————————————————— //
void setup()
{
  // set up serial data communication and transmission speed
  Serial.begin(baud_rate);
  if (DEBUG) { while (!Serial) delay(10); } // wait for serial in debug mode

  // set up ADC and PWM resolutions
  analogReadResolution(adc_res);
  analogWriteResolution(pwm_res);

  // set up general purpose input/output pin modes
  for (byte i = 0; i < 3; i++) pinMode(led_pin[i], OUTPUT);
  for (byte i = 0; i < 2; i++) pinMode(encoder_pin[i], INPUT);
  pinMode(esc_pin, OUTPUT);
  pinMode(enable_pin, OUTPUT);
  pinMode(phase_pin, OUTPUT);
  pinMode(current_pin, INPUT);

  // initializing the servo variables and their positions
  for(byte i = 0; i < servo_num; i++) {
    actuator[i].init(servo_pin[i], servo_offset[i], servo_range[i], 
                    servo_freq[i], servo_linear[i]);
    if (DEBUG) actuator[i].print();
  }

  // arming the ESC and make it ready to take commands
  delay(1000);
  esc.arm();

  // setting up the wing-opening motor and encoder
  encoder.begin();

  // setting up the IMU, its registers, and the Madgwick filter
  if (imu.begin() != 0) Serial.println("IMU error");
  filter.begin(SAMPLE_RATE);
  imu.writeRegister(LSM6DS3_CTRL1_XL, ACC_ODR_104Hz);
  // imu.writeRegister(LSM6DS3_CTRL2_G, GYRO_ODR_416Hz);

  // configure the BLE services, characteristics, and callbacks
  setupBLE();

  // configure the CLI and define the commands
  setupCLI();

  // configure the task scheduler and add the tasks to the scheduler
  setupTasks();
  
  delay(5000);
  Serial.println("Setup done");
}


// —————————————————————————————————————————————————————————————————————————— //
//                                LOOP FUNCTION                               //
// —————————————————————————————————————————————————————————————————————————— //
void loop() {
  // run the task scheduler to execute the tasks in order
  scheduler.execute();



  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! UGLY FUNCTIONALITY TESTING CODE
  // static unsigned long prev_time = millis();
  
  // if (millis() - prev_time >= 2000) {
  //   Serial.println("ONE LINE OF DATA");
  //   bleuart.write( (uint8_t*) exp_data, sizeof(exp_data_t)*6);
  //   prev_time = millis();
  // }

  // if (data_idx >= data_len)
  // {
  //   char P[60];
  //   char Q[240];
  //   // Serial.println("SENDING DATA NOW");
  //   for (int i = 0; i < data_idx; i = i+6) {
  //       for (int j = 0; j < 6; j++)
  //       {

  //         // memcpy(P + j * 10 + 0, &exp_data[i+j].time, 2);
  //         // memcpy(P + j * 10 + 2, &exp_data[i+j].current, 2);
  //         // memcpy(P + j * 10 + 4, &exp_data[i+j].roll, 2);
  //         // memcpy(P + j * 10 + 6, &exp_data[i+j].pitch, 2);
  //         // memcpy(P + j * 10 + 8, &exp_data[i+j].yaw, 2);

  //         snprintf(Q, 240, "%d, %d, %d, %d, %d", 
  //                 exp_data[i+j].time, exp_data[i+j].current, 
  //                 exp_data[i+j].roll, exp_data[i+j].pitch, exp_data[i+j].yaw);
  //         bleuart.write(Q, strlen(Q));
  //          delay(50);
  //         // Serial.print(exp_data[i+j].time); Serial.print(",");
  //         // Serial.print(exp_data[i+j].current); Serial.print(",");
  //         // Serial.print(exp_data[i+j].roll); Serial.print(",");
  //         // Serial.print(exp_data[i+j].pitch); Serial.print(",");
  //         // Serial.println(exp_data[i+j].yaw);
          
  //       }
        
  //       //P = (uint8_t*) exp_data;
  //       //P += sizeof(exp_data_t)*i;
  //       //bleuart.write(P, sizeof(exp_data_t)*6);
  //       //delay(50);
  //   }
  //   ts_data_logger.disable();
  // }
}