#include "esc.h"

ESC::ESC(byte pin, int min_pulse, int max_pulse, int arm_pulse, int stop_pulse,
         int calib_delay) {
  this->pin = pin;
  this->min_pulse = min_pulse;
  this->max_pulse = max_pulse;
  this->arm_pulse = arm_pulse;
  this->stop_pulse = stop_pulse;
  this->calib_delay = calib_delay;
}

void ESC::calibrate() {
  servo.attach(pin);
  servo.writeMicroseconds(max_pulse);
  delay(calib_delay);
  servo.writeMicroseconds(min_pulse);
  delay(calib_delay);
  arm();
}

void ESC::arm() {
  servo.attach(pin);
  servo.writeMicroseconds(arm_pulse);
}

void ESC::speed(int speed) {
  speed = constrain(speed, min_pulse, max_pulse);
  if (this->pulse != speed) {
    this->pulse = speed;
    servo.writeMicroseconds(pulse);
  }
}

void ESC::stop() {
  servo.writeMicroseconds(stop_pulse); 
}

void ESC::setMinPulse  (int min_pulse  ) { this->min_pulse   = min_pulse;   }
void ESC::setMaxPulse  (int max_pulse  ) { this->max_pulse   = max_pulse;   }
void ESC::setArmPulse  (int arm_pulse  ) { this->arm_pulse   = arm_pulse;   }
void ESC::setStopPulse (int stop_pulse ) { this->stop_pulse  = stop_pulse;  }
void ESC::setCalibDelay(int calib_delay) { this->calib_delay = calib_delay; }

int ESC::getSpeed() {
  return pulse;
}