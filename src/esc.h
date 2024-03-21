#ifndef ESC_H
#define ESC_H

#include <Arduino.h>
#include <Servo.h>

#define ESC_MIN 1000
#define ESC_MAX 2000
#define ESC_ARM 500
#define CALIBRATION_DELAY 5000

class ESC {
public:
  ESC(byte pin, int min_pulse = ESC_MIN, int max_pulse = ESC_MAX,
      int arm_pulse = ESC_ARM, int stop_pulse = ESC_ARM,
      int calib_delay = CALIBRATION_DELAY);

  void calibrate();
  void arm();
  void stop();
  void speed(int speed);
  void setMinPulse(int min_pulse);
  void setMaxPulse(int max_pulse);
  void setArmPulse(int arm_pulse);
  void setStopPulse(int stop_pulse);
  void setCalibDelay(int calib_delay);

  int getSpeed();

private:
  byte pin;
  int pulse;
  int min_pulse;
  int max_pulse;
  int arm_pulse;
  int stop_pulse;
  int calib_delay;
  Servo servo;
};

#endif