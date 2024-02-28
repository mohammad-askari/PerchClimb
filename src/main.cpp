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
#include "callbacks.h"

// __________________________  MAIN FUNCTION FLAGS  _________________________ //
bool DEBUG = true;   // enables extra serial connection in debug mode
bool MANUAL = false; // enables manual pilotting using transmitter inputs

// ————————————————————————————— BOARD VARIABLES ———————————————————————————— //
const int adc_res = 10;                    // xiao's ADC resolution (12-bit max)
const int pwm_res = 8;                     // xiao's PWM resolution (16-bit max)
const int adc_range = pow(2, adc_res) - 1; // xiao's ADC maximum value
const int pwm_range = pow(2, pwm_res) - 1; // xiao's PWM maximum value
const long baud_rate = 115200;             // serial data rate (bits per second)

const byte led_pin[] = {LED_RED, LED_GREEN, LED_BLUE}; // 3-in-1 LED pins

// —————————————————————————————— BLE VARIABLES ————————————————————————————— //
BLEDis bledis;                    // BLE service for device information
BLEUart bleuart;                  // BLE service for UART communication
const byte ble_dle = 251;         // BLE data length (v4.2+: 251, otherwise: 27)
const byte ble_mtu = ble_dle - 4; // BLE maximum transmission unit buffer size

// —————————————————————————————— IMU VARIABLES ————————————————————————————— //
#define SAMPLE_RATE 100       // filtering sample rate [Hz]
#define LSM6DS3_CTRL1_XL 0x10 // control register for accelerometer
#define LSM6DS3_CTRL2_G 0x11  // control register for gyroscope
#define ACC_ODR_104Hz 0x40    // setting 104Hz ODR for accelerometer
#define GYRO_ODR_416Hz 0x60   // setting 416Hz ODR for gyroscope
LSM6DS3 imu(I2C_MODE, 0x6A);  // I2C device address
Madgwick filter;

// ————————————————————————————— SERVO VARIABLES ———————————————————————————— //
// ————— ORDER: aileron, elevator, rudder, clutch, body hook, tail hook ————— //
const byte    servo_pin[]    = {     1,      2,      3,      4,      5,      6};
const int     servo_offset[] = {    +0,     +0,     +0,    -25,     +9,    -25};
const int     servo_range[]  = {   +80,    +50,   +100,    -25,    -30,    +33};
const float   servo_freq[]   = {  +0.5,   -0.5,     +0,     +0,     +0,     +0};
const drive_t servo_linear[] = {  STEP, LINEAR,   STEP,   STEP,   STEP,   STEP};
const byte    servo_num      = sizeof(servo_pin) / sizeof(servo_pin[0]);
Actuator actuator[servo_num];

// —————————————————————————————— ESC VARIABLES ————————————————————————————— //
const byte esc_pin = 7;
const int  esc_min = 1000; // ESC minimum speed
const int  esc_max = 2000; // ESC maximum speed
const int  esc_arm = 500;  // ESC arm value
int esc_speed;             // ESC variable for adjusting speed
ESC esc(esc_pin, esc_min, esc_max, esc_arm);

// ————————————————————— WING MOTOR & ENCODER VARIABLES ————————————————————— //
const byte encoder_pin[] = {8, 8}; // quadrature encoder pins
const byte phase_pin  = 9;         // DC motor direction control pin
const byte enable_pin = 10;        // DC motor speed control PWM pin
const float gear_ratio = 297.92;   // DC motor gear ratio (faster motor: 150.58)
const float spool_diameter = 10;   // spool diameter [mm]
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
// char pitch[64];
// char roll[64];
// char yaw[64];
char current[64];
// char timex[64];

void add_measure(){
	// char buffer[8];
	// strcat(pitch, dtostrf(filter.getPitch(), 4, 0, buffer));
	// strcat(pitch, ",");
	// strcat(roll, dtostrf(filter.getRoll(), 4, 0, buffer));
	// strcat(roll, ",");
	// strcat(yaw, dtostrf(filter.getYaw(), 4, 0, buffer));
	// strcat(yaw, ",");
	// char long_buffer[32];
	// sprintf(long_buffer, "%d", millis());
	// strcat(timex, long_buffer);
	// strcat(timex, ",");
	strcat(current, "2");
	strcat(current, ",");
	// strcat(current, analogRead(PIN_CURRENT))

	// data_time.writeValue(timex);
	// data_pitch.writeValue(pitch);
	// data_roll.writeValue(roll);
	// data_yaw.writeValue(yaw);
	// data_current.writeValue(current);
}

void clear_measures(){

}

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
  for (byte i = 0; i < 3; i++)
    pinMode(led_pin[i], OUTPUT);
  for (byte i = 0; i < 2; i++)
    pinMode(encoder_pin[i], INPUT);
  pinMode(esc_pin, OUTPUT);
  pinMode(enable_pin, OUTPUT);
  pinMode(phase_pin, OUTPUT);
  pinMode(current_pin, INPUT);

  // configure the BLE services, characteristics, and callbacks
  setupBLE();

  // configure the CLI and define the commands
  setupCLI();

  // initializing the servo variables and their positions
  for(byte i = 0; i < servo_num; i++) {
    actuator[i].init(servo_pin[i], servo_offset[i], servo_range[i], 
                    servo_freq[i], servo_linear[i]);
    if (DEBUG) actuator[i].print();
  }

  // arming the ESC and make it ready to take commands
  esc.arm();
  delay(5000);

  // setting up the wing motor encoder
  encoder.begin();

  // setting up the IMU, its registers, and the Madgwick filter
  if (imu.begin() != 0)
    Serial.println("IMU error");
  filter.begin(SAMPLE_RATE);
  imu.writeRegister(LSM6DS3_CTRL1_XL, ACC_ODR_104Hz);
  // imu.writeRegister(LSM6DS3_CTRL2_G, GYRO_ODR_416Hz);
}


// —————————————————————————————————————————————————————————————————————————— //
//                                LOOP FUNCTION                               //
// —————————————————————————————————————————————————————————————————————————— //
void loop()
{
  // check BLE UART for user input and parse into the CLI
  while (bleuart.available())
  {
    int ch = bleuart.read(); // read a single byte from the BLE UART
    processCommand(ch, buffer_len, buffer_idx, buffer);
  }

  // check serial for user input and parse into the CLI
  while (Serial.available())
  {
    int ch = Serial.read();  // read a single byte from the serial
    processCommand(ch, buffer_len, buffer_idx, buffer);
  }

  if (DEBUG) 
  {
    for(byte i = 0; i < 2; i++) {
        actuator[i].move();
        actuator[i].printSignal();
        delay(10);
      }
  }
  
  /**
    int speedESC;
    int speedMIN = 1000;
    int speedMAX = 2000;
    int range = (speedMAX - speedMIN);
    int speedIncrement = counterESC % range;
    if (speedIncrement == 0)
    {
      signESC = -1 * signESC;
    }
    if (signESC > 0)
    {
      speedESC = speedMIN + speedIncrement;
    }
    else
    {
      speedESC = speedMAX - speedIncrement;
    }

    myESC.speed(speedESC); // tell ESC to go to the oESC speed value
    Serial.print("Speed: ");
    Serial.print(speedESC);
    Serial.print(". Current: ");
    Serial.println(analogRead(CURRENT) / float(adc_range));

    // SERVO
    int posServo;
    int posMIN = 0;
    int posMAX = 180;
    int posRange = posMAX - posMIN;
    int posIncrement = counterServo % posRange;
    if (posIncrement == 0)
    {
      signServo = -1 * signServo;
    }

    if (signServo > 0)
    {
      posServo = posMIN + posIncrement;
    }
    else
    {
      posServo = posMAX - posIncrement;
    }

    for (byte i = 0; i < servo_num; i++)
    {
      servos[i].write(posServo); // tell servo to go to position in variable 'pos'
    }
    Serial.print("Servo Position: ");
    Serial.println(posServo);

    // IMU

    char buffer[8];
    static unsigned long previousTime = millis();

    unsigned long currentTime = millis();

    if (currentTime - previousTime >= 1000 / SAMPLE_RATE)
    {
      float ax, ay, az;
      float gx, gy, gz;

      ax = myIMU.readFloatAccelX();
      ay = myIMU.readFloatAccelY();
      az = myIMU.readFloatAccelZ();
      gx = myIMU.readFloatGyroX();
      gy = myIMU.readFloatGyroY();
      gz = myIMU.readFloatGyroZ();

      // Serial.print(">counter:");  Serial.println(counter);

      filter.updateIMU(gx, gy, gz, ax, ay, az);

      // Get the current yaw value

      // Display yaw, pitch, and roll
      Serial.print("Roll:");
      Serial.print(dtostrf(filter.getRoll(), 4, 0, buffer));
      Serial.print("\t");
      Serial.print("Pitch:");
      Serial.print(dtostrf(filter.getPitch(), 4, 0, buffer));
      Serial.print("\t");
      Serial.print("Yaw:");
      Serial.println(dtostrf(filter.getYaw(), 4, 0, buffer));

      previousTime = millis();

      // At the end of every minute, display the total yaw drift and reset it
    }

    counterESC++;
    counterServo++;
    delay(10);
    **/
}