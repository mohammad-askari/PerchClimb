#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include <Servo.h>

#define RANGE_MIN -100
#define RANGE_MAX +100
#define SERVO_MIN 0
#define SERVO_MAX 180

typedef enum {
  STEP = 0,
  RAMP = 1
} signal_t;

class Actuator {
  public:
    Actuator(const char* name, byte pin, int offset, 
             int range, float frequency, signal_t mode);
    void init();

    void setOffset(int offset);
    void setRange(int range);
    void setPosition(int position);
    void setFrequency(float frequency);
    void setMode(signal_t mode);
    void setTime(unsigned long start_time);

    float getPosition();

    void move();
    void reset();
    void print();
    void printSignal();

  private:
    const char* name;
    byte pin;
    int offset;
    int range;
    float frequency;
    signal_t mode;

    int position;
    int last_position;
    int min_pos;
    int max_pos;
    unsigned long start_time;
    Servo servo;

    void setLimits();
    void updatePosition();
};

#endif