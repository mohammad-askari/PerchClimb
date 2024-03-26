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
#include "pid.h"

// __________________________  MAIN FUNCTION FLAGS  _________________________ //
extern bool DEBUG, MANUAL;

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
extern const char ble_name[];

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
extern const byte phase_pin, enable_pin, encoder_pin[], cpr;
extern const float gear_ratio, spool_diameter;
extern int dc_speed;
extern Clutch clutch;

// —————————————————————————————— PID CONTROL ——————————————————————————————— //
extern PID pid_roll, pid_pitch, pid_yaw;

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

typedef struct { 
    int16_t  throttle;
    int8_t   aileron;
    int8_t   elevator;
    int8_t   rudder;
    int8_t   clutch;
    int8_t   body_hook;
    int8_t   tail_hook;
    int16_t  wing_open;
} cmd_data_t;

extern const int drop_freq, move_freq, log_freq, filt_freq, log_max, data_len;
extern int data_idx, exp_delayed;
extern char exp_info[];
extern float exp_duration;
extern exp_data_t exp_data[];
extern cmd_data_t cmd_data[];
extern bool transfer_include_commands;

// ———————————————————————— CURRENT SENSOR VARIABLES ———————————————————————— //
extern const byte current_pin;
extern int current_samples[], current_average;

// —————————————————————— EXPERIMENT-SPECIFIC VARIABLES ————————————————————— //
extern bool  climb_wing_loosening;

extern float pre_hover_time;
extern int   pre_hover_esc, transition_esc;
extern bool  hover_use_hooks;

extern float pre_descent_time, post_descent_time, descent_freq;
extern int   pre_descent_esc, post_descent_esc;
extern bool  is_freefall_mode;

extern float wing_opening_duration;
extern bool  is_wing_opening, is_opening_reverse;

extern float pre_unperch_duration, takeoff_duration, takeoff_pitch;
extern int   pre_unperch_esc, tilt_esc, takeoff_esc;
extern long  takeoff_start_time;
extern bool  is_start_of_takeoff;
extern bool  is_level_flight;

// ———————————————————————— TASK SCHEDULER VARIABLES ———————————————————————— //
extern TsTask ts_parser, ts_sensors, ts_ble_conn, ts_kill, ts_motor_update;

extern TsTask ts_pre_climb,   ts_climb_on,   ts_climb_off;
extern TsTask ts_pre_descent, ts_descent_on, ts_descent_off;
extern TsTask ts_pre_hover,   ts_hover_on,   ts_hover_off;
extern TsTask ts_pre_unperch, ts_unperch_on, ts_unperch_off;

extern TsTask ts_data_logger, ts_data_transfer;

extern TsScheduler scheduler;

#endif