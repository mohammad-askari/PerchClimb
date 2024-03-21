#ifndef MAIN_H
#define MAIN_H

// —————————————————————————————————————————————————————————————————————————— //
//                            LIBRARIES & VARIABLES                           //
// —————————————————————————————————————————————————————————————————————————— //
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <SimpleCLI.h>
#include <LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <quadrature.h>
#include <TSchedulerDeclarations.hpp>

#include "actuator.h"
#include "esc.h"
#include "clutch.h"

// __________________________  MAIN FUNCTION FLAGS  _________________________ //
extern bool DEBUG;
extern bool MANUAL;

// ————————————————————————————— BOARD VARIABLES ———————————————————————————— //
extern const int adc_res, pwm_res, adc_range, pwm_range;
extern const long baud_rate;
extern const byte led_pin[];

// —————————————————————————————— BLE VARIABLES ————————————————————————————— //
extern BLEDis bledis;
extern BLEUart bleuart;
extern const byte ble_dle;
extern const byte ble_mtu;
extern byte ble_packet_len;

// —————————————————————————————— IMU VARIABLES ————————————————————————————— //
#define LSM6DS3_CTRL1_XL 0x10
#define LSM6DS3_CTRL2_G 0x11
#define ACC_ODR_104Hz 0x40
#define GYRO_ODR_416Hz 0x60
extern float roll, pitch, yaw;
extern LSM6DS3 imu;
extern Madgwick filter;

// ————————————————————————————— SERVO VARIABLES ———————————————————————————— //
extern Actuator aileron, elevator, rudder, wing_lock, body_hook, tail_hook;
extern Actuator* actuator[];
extern const byte servo_num;

// —————————————————————————————— ESC VARIABLES ————————————————————————————— //
extern const byte esc_pin;
extern const int esc_min, esc_max, esc_arm;
extern int esc_speed;
extern ESC esc;

// ————————————————————— WING MOTOR & ENCODER VARIABLES ————————————————————— //
extern const byte phase_pin;
extern const byte enable_pin;
extern const byte encoder_pin[];
extern const byte cpr;
extern const float gear_ratio;
extern const float spool_diameter;
extern int dc_speed;
extern Clutch clutch;

// ———————————————————————— CURRENT SENSOR VARIABLES ———————————————————————— //
extern const byte current_pin;
extern int current;

// ———————————————————————————— PARSER VARIABLES ———————————————————————————— //
extern SimpleCLI cli;

// ——————————————————————— EXPERIMENTAL DATA VARIABLES —————————————————————— //
typedef struct {
    uint16_t time;
    int16_t  current;
    int16_t  roll;
    int16_t  pitch;
    int16_t  yaw;
} exp_data_t;

extern const int  drop_freq;
extern const int  move_freq;
extern const int  log_freq;
extern const int  log_max;
extern const int  data_len;
extern int        data_idx;
extern char       exp_info[];
extern float      exp_duration;
extern int        exp_delayed;
extern unsigned long start_time;
extern exp_data_t exp_data[];

// —————————————————————— EXPERIMENT-SPECIFIC VARIABLES ————————————————————— //
extern float pre_hover_time;
extern int pre_hover_esc;
extern bool hover_use_hooks;
extern int transition_esc;

extern float pre_descent_time;
extern int   pre_descent_esc;
extern float post_descent_time;
extern int   post_descent_esc;
extern float descent_freq;
extern bool  is_freefall_mode;

extern float wing_opening_duration;
extern bool  is_wing_opening;

extern float pre_unperch_duration;
extern int   pre_unperch_esc;
extern int   tilt_esc; 
extern float takeoff_duration;
extern int   takeoff_esc;
extern float takeoff_pitch;
extern long  takeoff_start_time;
extern bool  is_start_of_takeoff;

// ———————————————————————— TASK SCHEDULER VARIABLES ———————————————————————— //
extern TsTask ts_parser;
extern TsTask ts_ble_parser;
extern TsTask ts_sensors;
extern TsTask ts_ble_conn;
extern TsTask ts_kill;
extern TsTask ts_climb_on;
extern TsTask ts_climb_off;
extern TsTask ts_pre_descent;
extern TsTask ts_descent_on;
extern TsTask ts_descent_off;
extern TsTask ts_pre_hover;
extern TsTask ts_hover_on;
extern TsTask ts_hover_off;
extern TsTask ts_pre_unperch;
extern TsTask ts_unperch_on;
extern TsTask ts_unperch_off;
extern TsTask ts_motor_update;
extern TsTask ts_data_logger;
extern TsTask ts_data_transfer;

extern TsScheduler scheduler;

#endif