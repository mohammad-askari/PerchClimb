#ifndef CLUTCH_H
#define CLUTCH_H

#include <Arduino.h>
#include "actuator.h"

#define UNDEFINED 255

typedef enum {
  CLOSE = -1,
  STOP  =  0,
  OPEN  = +1
} direction_t;

typedef enum {
  DISENGAGED = 0,
  ENGAGED    = 1
} clutch_t;

class Clutch {
  public:
    Clutch(Actuator &servo);
    void pins(byte dir_pin, byte pwm_pin, byte enc1_pin, byte enc2_pin = UNDEFINED);
    void init(Actuator servo, clutch_t state, byte cpr, float gear, float spool, int pwm_max);

    void engage();
    void disengage();
    void forward();
    void reverse();
    void stop();
    void speed(int speed);
    
    int  getSpeed();
    long getCounts();
    void resetCounts();


    void print();
    // void printSignal(int motor_idx);

  private:
    float        gear;
    float        spool;
    byte         cpr;
    clutch_t     state;
    direction_t  direction;
    Actuator     &servo;

    int  pwm;
    int  pwm_max;
    byte pwm_pin;
    byte dir_pin;
    byte enc1_pin;
    byte enc2_pin;
    bool is_dual_encoder;
    volatile long counts;
    volatile byte readings;

    static Clutch *clutch_ptr;
    static void interruptSingle();
    static void interruptDual();
    void countPulseSingle();
    void countPulseDual();
};

#endif