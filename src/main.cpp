/**
 * @file main.cpp
 * @author Mohammad Askari (mohammad.askari@epfl.ch)
 * @brief This program ...
 * @version 1.0
 * @date 2024-02
 *
 * @copyright Copyright (c) 2024, Mohammad Askari, Laboratory of Intelligent
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
bool DEBUG = false;  // enables extra serial connection in debug mode
bool MANUAL = false; // TODO: enables manual pilotting using transmitter inputs

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
float roll, pitch, yaw;      // filtered Euler angles [deg]
LSM6DS3 imu(I2C_MODE, 0x6A); // I2C IMU device address
Madgwick filter;             // Madgwick sensor fusion algorithm object

// ————————————————————————————— SERVO VARIABLES ———————————————————————————— //
//                    name    , pin, offset, range, freq, mode
Actuator aileron  ("aileron"  ,  7 ,  +0   , +100 , +0.0, STEP);
Actuator elevator ("elevator" ,  2 ,  +0   , +100 , +0.0, STEP); // FIXME pin 2
Actuator rudder   ("rudder"   ,  3 ,  +0   , +90  , +0.0, STEP); // FIXME pin 3
Actuator wing_lock("clutch"   ,  4 ,  +0   , +12  , +0.0, STEP);
Actuator body_hook("body hook",  5 ,  +0   , +25  , +0.0, STEP);
Actuator tail_hook("tail hook",  6 ,  +28  , +20  , +0.0, STEP);

Actuator* actuator[] = {&aileron,   &elevator,  &rudder, 
                        &wing_lock, &body_hook, &tail_hook};
const byte servo_num = sizeof(actuator) / sizeof(actuator[0]);

// —————————————————————————————— ESC VARIABLES ————————————————————————————— //
const byte esc_pin = 1;                      // ESC PWM pin
const int  esc_min = 1000;                   // ESC minimum speed pulse [μs]
const int  esc_max = 2000;                   // ESC maximum speed pulse [μs]
const int  esc_arm = 1000;                   // ESC arm value pulse     [μs]
int esc_speed;                               // ESC speed parameter     [μs]
ESC esc(esc_pin, esc_min, esc_max, esc_arm); // ESC motor object

// ——————————————————————————————— PID CONTROL —————————————————————————————— //
//            kp ,  ki ,  kd ,   dt ,  min out,  max out
PID pid_roll (1.0,  0.0,  0.2,  0.01,    -1.0 ,    +1.0);
PID pid_pitch(1.0,  0.1,  0.2,  0.01,    -1.0 ,    +1.0);
PID pid_yaw  (1.0,  0.0,  0.2,  0.01,    -1.0 ,    +1.0);

// ————————————————————— WING MOTOR & ENCODER VARIABLES ————————————————————— //
const byte encoder_pin[] = {8};   // single or dual-channel encoder pin(s) // FIXME pin 8
const byte enable_pin = 9;        // DC motor speed control PWM pin
const byte phase_pin  = 10;       // DC motor direction control pin
const byte cpr = 12;              // dual-channel encoder counts per revolution
const float gear_ratio = 297.92;  // DC motor gear ratio (faster motor: 150.58)
const float spool_diameter = 10;  // wing-opening mechanism spool diameter [mm]
int dc_speed = pwm_range;         // DC motor variable for adjusting speed
Clutch clutch(wing_lock);         // wing-opening mechanism (clutch) object

// ———————————————————————————— PARSER VARIABLES ———————————————————————————— //
SimpleCLI cli;                       // command line interface (CLI) object
const byte buffer_len = ble_mtu - 2; // size of the input buffer characters
byte buffer_idx;                     // position index variable for the buffer
char cliBuffer[buffer_len];          // CLI buffer array to parse user inputs

// ——————————————————————— EXPERIMENTAL DATA VARIABLES —————————————————————— //
const int drop_freq = 10;                  // gradual ESC speed drop rate  [Hz]
const int move_freq = 100;                 // motors movement update rate  [Hz]
const int log_freq  = 100;                 // data logging frequency       [Hz]
const int filt_freq = log_freq * 10;       // data filtering frequency     [Hz]
const int log_max   = 60;                  // maximum data logging duration [s]
const int data_len  = log_max * log_freq;  // maximum size of data arrays
int   data_idx = 0;                        // position index of the data arrays
char  exp_info[200];                       // experimental information string
float exp_duration = 10;                   // experimental duration    [s]
int   exp_delayed = 10;                    // experimental start delay [s]
exp_data_t exp_data[data_len];             // logged time and sensor data array
cmd_data_t cmd_data[data_len];             // logged actuator commands data array
bool transfer_include_commands;            // flag to enable full data transfer

// ———————————————————————— CURRENT SENSOR VARIABLES ———————————————————————— //
const byte current_pin = 0;                // current sensor analog pin
int current_samples[filt_freq / log_freq]; // current sensor samples array [ADC]
int current_average;                       // current sensor average value [ADC]

// —————————————————————— EXPERIMENT-SPECIFIC VARIABLES ————————————————————— //
bool  climb_wing_loosening;                // flag to loosen wings for climbing

float pre_hover_time;                      // pre-hover ascent time [s]
int   pre_hover_esc;                       // pre-hover ESC speed  [μs]
bool  hover_use_hooks;                     // flag to use hooks for hovering
int   transition_esc;                      // transition start ESC speed
 
float pre_descent_time;                    // pre-descent hover time  [s]
int   pre_descent_esc;                     // pre-descent ESC speed  [μs]
float post_descent_time;                   // post-descent hover time [s]
int   post_descent_esc;                    // post-descent ESC speed [μs]
float descent_freq;                        // descent frequency      [Hz]
bool  is_freefall_mode;                    // flag to enable freefall mode
 
float wing_opening_duration;               // wing opening duration [s]
bool  is_wing_opening;                     // flag to enable wing opening
bool  is_opening_reverse;                  // flag to switch to wing closing
 
float pre_unperch_duration;                // pre-unperch hover duration [s]
int   pre_unperch_esc;                     // pre-unperch ESC speed     [μs]
int   tilt_esc;                            // tilt ESC speed            [μs]
float takeoff_duration;                    // initial takeoff duration   [s]
int   takeoff_esc;                         // initial takeoff ESC speed [μs]
float takeoff_pitch;                       // desired takeoff pitch    [deg]
long  takeoff_start_time;                  // initial takeoff time flag
bool  is_start_of_takeoff;                 // flag to set enable takeoff
bool  is_level_flight;                     // flag to set level flight mode
 
// ———————————————————————— TASK SCHEDULER VARIABLES ———————————————————————— //
// ———— TASK PARAMETERS: interval [ms/μs], #executions, callback function ——— //
TsTask ts_parser       (TASK_IMMEDIATE,        TASK_FOREVER, &tsParser);
TsTask ts_sensors      (TASK_SECOND/log_freq,  TASK_FOREVER, &tsSensors);
TsTask ts_ble_conn     (TASK_IMMEDIATE,        TASK_ONCE,    &tsBLEConn);
TsTask ts_pre_climb    (TASK_IMMEDIATE,        TASK_ONCE,    &tsPreClimb);
TsTask ts_climb_on     (TASK_IMMEDIATE,        TASK_ONCE,    &tsClimbOn);
TsTask ts_climb_off    (TASK_SECOND/move_freq, TASK_FOREVER, &tsClimbOff);
TsTask ts_pre_descent  (TASK_IMMEDIATE,        TASK_ONCE,    &tsPreDescent);
TsTask ts_descent_on   (TASK_SECOND/move_freq, TASK_FOREVER, &tsDescentOn);
TsTask ts_descent_off  (TASK_IMMEDIATE,        TASK_ONCE,    &tsDescentOff);
TsTask ts_pre_hover    (TASK_IMMEDIATE,        TASK_ONCE,    &tsPreHover);
TsTask ts_hover_on     (TASK_SECOND/drop_freq, TASK_FOREVER, &tsHoverOn);
TsTask ts_hover_off    (TASK_IMMEDIATE,        TASK_ONCE,    &tsHoverOff);
TsTask ts_pre_unperch  (TASK_IMMEDIATE,        TASK_ONCE,    &tsPreUnperch);
TsTask ts_unperch_on   (TASK_SECOND/move_freq, TASK_FOREVER, &tsUnperchOn);
TsTask ts_unperch_off  (TASK_IMMEDIATE,        TASK_ONCE,    &tsUnperchOff);
TsTask ts_motor_update (TASK_SECOND/move_freq, TASK_FOREVER, &tsMotorUpdate);      
TsTask ts_data_logger  (TASK_SECOND/log_freq,  TASK_FOREVER, &tsDataLogger);
TsTask ts_data_transfer(TASK_HOUR,             TASK_ONCE,    &tsDataTransfer);
TsTask ts_kill         (TASK_IMMEDIATE,        TASK_ONCE,    &tsKill);
TsScheduler scheduler; // scheduler object to run tasks in order


// —————————————————————————————————————————————————————————————————————————— //
//                               SETUP FUNCTION                               //
// —————————————————————————————————————————————————————————————————————————— //
void setup()
{
  // set up serial data communication and transmission speed
  Serial.begin(baud_rate);
  if (DEBUG) { while (!Serial) delay(10); } // wait for serial in debug mode
  delay(1000);

  // set up ADC and PWM resolutions
  analogReadResolution(adc_res);
  analogWriteResolution(pwm_res);

  // set up general purpose input/output pin modes
  pinMode(current_pin, INPUT);

  // initializing the servo objects and move to initial positions
  for(byte i = 0; i < servo_num; i++) {
    actuator[i]->init();
    if (DEBUG) actuator[i]->print();
  }
  body_hook.setPosition(RANGE_MAX); // retracting the body hook
  tail_hook.setPosition(RANGE_MAX); // retracting the tail hook

  // initializing the wing-opening mechanism variables
  clutch.pins(enable_pin, phase_pin, encoder_pin[0]);
  clutch.init(wing_lock, ENGAGED, cpr, gear_ratio, spool_diameter, pwm_range);
  if (DEBUG) clutch.print();

  // arming the ESC and making it ready to take commands
  esc.arm();

  // setting up the IMU, its registers, and the Madgwick filter
  if (imu.begin() != 0) {
    Serial.println("IMU error");
    setLED(led_pin,'R');
  }
  filter.begin(log_freq);
  imu.writeRegister(LSM6DS3_CTRL1_XL, ACC_ODR_104Hz);
  // imu.writeRegister(LSM6DS3_CTRL2_G, GYRO_ODR_416Hz);

  // configure the BLE services, characteristics, and callbacks
  setupBLE();

  // configure the CLI and define the commands
  setupCLI();

  // configure the task scheduler and add the tasks to the scheduler
  setupTasks();

  // this message will help to identify restart of the system
  Serial.println("Setup completed. Starting the main loop...");
}

// —————————————————————————————————————————————————————————————————————————— //
//                                LOOP FUNCTION                               //
// —————————————————————————————————————————————————————————————————————————— //
void loop() {  
  // run the task scheduler to execute the tasks in order
  scheduler.execute();
  
  // for (byte i = 0; i < servo_num; i++) actuator[i]->print();
  static unsigned long prev_time  = 0;
  static unsigned long loop_count = 0;
  loop_count++;

  unsigned long now = millis();
  if (now - prev_time > 1000) {
    Serial.println(loop_count);
    loop_count = 0;
    prev_time  = now;
  }

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! UGLY FUNCTIONALITY TESTING CODE
  // if (millis()%1000 == 0) {
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