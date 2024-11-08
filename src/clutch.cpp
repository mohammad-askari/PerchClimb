#include "clutch.h"
#include "communication.h"

// use of a static member to enable attach interrupt usage in the class
Clutch *Clutch::clutch_ptr = nullptr;
Clutch::Clutch(Actuator &servo) : servo(servo) { clutch_ptr = this; }

// sets the pins for the clutch and determines if a quadrature encoder is used
void Clutch::pins(byte pwm_pin, byte dir_pin, byte enc1_pin, byte enc2_pin) {
  this->pwm_pin  = pwm_pin;
  this->dir_pin  = dir_pin;
  this->enc1_pin = enc1_pin;
  this->enc2_pin = enc2_pin;
  this->is_dual_encoder = (enc2_pin != UNDEFINED);

  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  pinMode(enc1_pin, INPUT_PULLUP);
  if (is_dual_encoder) pinMode(enc2_pin, INPUT_PULLUP);
}

// initializes the clutch object variables and sets the initial state
void Clutch::init(Actuator servo, clutch_t state,
                  byte cpr, float gear, float spool, int pwm_max) {
  this->servo   = servo;
  this->state   = state;
  this->cpr     = cpr;
  this->gear    = gear;
  this->spool   = spool;
  this->pwm_max = pwm_max;
  this->direction = STOP;

  stop();
  resetCounts();

  // if (!is_dual_encoder) { // TODO: IMPLEMENT ENCODER INTERRUPT FUNCTIONALITY
  //   this->cpr = cpr / 2;
  //   attachInterrupt(enc1_pin, Clutch::interruptSingle, CHANGE);
  // }
  // else {
  //   this->cpr = cpr;
  //   attachInterrupt(enc1_pin, Clutch::interruptDual, CHANGE);
  //   attachInterrupt(enc2_pin, Clutch::interruptDual, CHANGE);
    
  //   // initial encoder state stored in the last 2 bits of the readings variable
  //   byte s = 0;
  //   if (analogRead(enc1_pin)) s |= 1;
  //   if (analogRead(enc2_pin)) s |= 2;
  //   this->readings = s;
  // }
}

// static interrupt functions to count pulses based on encoder type
void Clutch::interruptSingle() {
    if (clutch_ptr != nullptr)
        clutch_ptr->countPulseSingle();
}
void Clutch::interruptDual() {
    if (clutch_ptr != nullptr)
        clutch_ptr->countPulseDual();
}

void Clutch::engage()    { state = ENGAGED;    servo.setPosition(RANGE_MAX); }
void Clutch::disengage() { state = DISENGAGED; servo.setPosition(RANGE_MIN); }
void Clutch::forward()   { direction = OPEN;   digitalWrite(dir_pin, HIGH);  }
void Clutch::reverse()   { direction = CLOSE;  digitalWrite(dir_pin, LOW);   }

void Clutch::stop()      { speed(0); } // TODO: IMPLEMENT direction = STOP;

void Clutch::speed(int speed) {
  speed = constrain(speed, 0, pwm_max);
  if (this->pwm != speed) {
    this->pwm = speed;
    analogWrite(pwm_pin, pwm);
  }
}

int  Clutch::getSpeed()    { return direction * pwm; }
long Clutch::getCounts()   { return counts; }
void Clutch::resetCounts() { counts = 0; }

// print variables for debugging purposes
void Clutch::print() {
  byte len = 70;
  char str[len];
  const char* state_str = state ? "engaged" : "disengaged";
  if (!is_dual_encoder)
    snprintf(str, len, "clutch: %s, pwm: %d, dir: %+2d, counts: %ld, %d", 
             state_str, pwm, direction, counts, digitalRead(enc1_pin));
  else
    snprintf(str, len, "clutch: %s, pwm: %d, dir: %+2d, counts: %ld, %d, %d", 
             state_str, pwm, direction, counts, digitalRead(enc1_pin), digitalRead(enc2_pin));

  Serial.println(str);
  sendStringAsStringPacketViaBLE(str + String("\n"));
}

// single-channel encoder pulse counting based on commanded direction
void Clutch::countPulseSingle() {
  counts++;
  // switch (direction) {
  //   case OPEN:
  //     counts++; break;
  //   case CLOSE:
  //     counts--; break;
  //   default:
  //     break;
  // }
}

// dual-channel encoder pulse counting based on inferred direction
// Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
//                            _______         _______       
//                pin1 ______|       |_______|       |______ pin1
//  negative <---         _______         _______         __      --> positive
//                pin2 __|       |_______|       |_______|   pin2
//
//             new     new     old     old
//            pin 2	  pin 1   pin 2	  pin 1	    result   
//            —————	  —————   —————	  —————	  ———————————
//              0	      0	      0	      0     no movement
//              0	      0	      0	      1         +1
//              0	      0	      1	      0         -1
//              0	      0	      1	      1         +2  (assume pin1 edges only)
//              0	      1	      0	      0         -1
//              0	      1	      0	      1     no movement
//              0	      1	      1	      0         -2  (assume pin1 edges only)
//              0	      1	      1	      1         +1
//              1	      0	      0	      0         +1
//              1	      0	      0	      1         -2  (assume pin1 edges only)
//              1	      0	      1	      0     no movement
//              1	      0	      1	      1         -1
//              1	      1	      0	      0         +2  (assume pin1 edges only)
//              1	      1	      0	      1         -1
//              1	      1	      1	      0         +1
//              1	      1	      1	      1     no movement
//
void Clutch::countPulseDual() {
  byte s = readings & 3;             // retains "old pin 2" and "old pin 1" bits
  if (digitalRead(enc1_pin)) s |= 4; // adds "new pin 1" state as the 3rd bit
  if (digitalRead(enc2_pin)) s |= 8; // adds "new pin 2" state as the 4th bit
  switch (s) {
    case 0: case 5: case 10: case 15:
      direction = STOP; break;
    case 1: case 7: case 8: case 14:
      counts++; 
      direction = OPEN; break;
    case 2: case 4: case 11: case 13:
      counts--;
      direction = CLOSE; break;
    case 3: case 12:
      counts += 2;
      direction = OPEN; break;
    default:
      counts -= 2;
      direction = CLOSE; break;
  }
  readings = (s >> 2);
}