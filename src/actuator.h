#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include <Servo.h>

#define RANGE_MIN -100
#define RANGE_MAX +100
#define SERVO_MIN 0
#define SERVO_MAX 180

typedef enum
{
  STEP = 0,
  LINEAR = 1
} drive_t;

class Actuator
{
public:
  int offset;
  int range;
  int position;
  float frequency;
  drive_t mode;
  Servo servo;

  Actuator() {}

  void init(byte pin, int offset, int range, float frequency, drive_t mode);

  void setRange(int range);
  void setOffset(int offset);
  void setFrequency(float frequency);
  void setPosition(int position);
  void setTime(unsigned long start_time);

  void move();
  void reset();
  void print();
  void printSignal();

private:
  byte pin;
  int min_pos;
  int max_pos;
  int last_position;
  unsigned long start_time;

  void setLimits();
  void updatePosition();
};

#endif