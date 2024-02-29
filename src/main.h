#ifndef MAIN_H
#define MAIN_H

// —————————————————————————————————————————————————————————————————————————— //
//                            LIBRARIES & VARIABLES                           //
// —————————————————————————————————————————————————————————————————————————— //
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <SimpleCLI.h>
#include <Servo.h>
#include <ESC.h>
#include <LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <avr/dtostrf.h>
#include <quadrature.h>
#include <TSchedulerDeclarations.hpp>

#include "actuator.h"

// __________________________  MAIN FUNCTION FLAGS  _________________________ //
extern bool DEBUG;
extern bool MANUAL;

// ————————————————————————————— BOARD VARIABLES ———————————————————————————— //
extern const int adc_res;
extern const int pwm_res;
extern const int adc_range;
extern const int pwm_range;
extern const long baud_rate;
extern const byte led_pin[];

// —————————————————————————————— BLE VARIABLES ————————————————————————————— //
extern BLEDis bledis;
extern BLEUart bleuart;
extern const byte ble_dle;
extern const byte ble_mtu;
extern byte ble_packet_len;

// —————————————————————————————— IMU VARIABLES ————————————————————————————— //
#define SAMPLE_RATE 100
#define LSM6DS3_CTRL1_XL 0x10
#define LSM6DS3_CTRL2_G 0x11
#define ACC_ODR_104Hz 0x40
#define GYRO_ODR_416Hz 0x60
extern LSM6DS3 imu;
extern Madgwick filter;

// ————————————————————————————— SERVO VARIABLES ———————————————————————————— //
extern const byte servo_pin[];
extern const int servo_offset[];
extern const int servo_range[];
extern const float servo_freq[];
extern const drive_t servo_linear[];
extern const byte servo_num;
extern Actuator actuator[];

// —————————————————————————————— ESC VARIABLES ————————————————————————————— //
extern const byte esc_pin;
extern const int esc_min;
extern const int esc_max;
extern const int esc_arm;
extern int esc_speed;
extern ESC esc;

// ————————————————————— WING MOTOR & ENCODER VARIABLES ————————————————————— //
extern const byte encoder_pin[];
extern const byte phase_pin;
extern const byte enable_pin;
extern const float gear_ratio;
extern const float spool_diameter;
extern int dc_speed;
extern Quadrature_encoder<8, 8> encoder;

// ———————————————————————— CURRENT SENSOR VARIABLES ———————————————————————— //
extern const byte current_pin;

// ———————————————————————————— PARSER VARIABLES ———————————————————————————— //
extern SimpleCLI cli;
extern const byte buffer_len;
extern byte buffer_idx;
extern char buffer[];
extern const byte current_pin;

// ———————————————————————— TASK SCHEDULER VARIABLES ———————————————————————— //
extern TsTask ts_parser;
extern TsTask ts_ble_conn;
extern TsTask ts_ble_lost;
extern TsTask ts_climb_off;
extern TsTask ts_climb_on;
extern TsTask ts_data_logger;
extern TsTask ts_data_transfer;

extern TsScheduler scheduler;


typedef struct {
    uint16_t time;
    int16_t  current;
    int16_t  roll;
    int16_t  pitch;
    int16_t  yaw;
} exp_data_t;

extern const int  move_freq;
extern const int  log_freq;
extern const int  log_max;
extern const int  data_len;
extern int        data_idx;
extern char       exp_info[];
extern int        exp_duration;
extern unsigned long start_time;
extern exp_data_t exp_data[];

#endif