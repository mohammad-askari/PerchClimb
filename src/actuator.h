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
  int8_t offset;
  int8_t range;
  int8_t position;
  float frequency;
  drive_t mode;
  Servo servo;

  Actuator() {}

  void init(uint8_t pin, int8_t offset, int8_t range,
            float frequency, drive_t mode);

  void setRange(int8_t range);
  void setOffset(int8_t offset);
  void setFrequency(int8_t frequency);
  void setPosition(int8_t position);
  void setTime(unsigned long start_time);

  void move();
  void reset();
  void print();
  void printSignal();

private:
  uint8_t pin;
  int8_t min_pos;
  int8_t max_pos;
  int8_t last_position;
  unsigned long start_time;

  void setLimits();
  void updatePosition();
};

#endif