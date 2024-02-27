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
  int8_t position;
  int8_t offset;
  int8_t range;
  float frequency;
  drive_t mode;
  Servo servo;

  // object constructor
  Actuator() {}

  // sets initial variables, and attaches servo pins
  void init(uint8_t pin, int8_t offset, int8_t range, 
            float frequency = 0, drive_t mode = drive_t::STEP)
  {
    this->pin = pin;
    this->position = offset;
    this->offset = offset;
    this->range = range;
    this->frequency = frequency;
    this->mode = mode;
    this->servo.attach(pin);

    servo.attach(pin);
    servo.write(position);
  }

  void move()
  {
    int min_pos = offset - range;
    int max_pos = offset + range;
    int pos     = constrain(position, min_pos, max_pos);
    int angle   = map(pos, RANGE_MIN, RANGE_MAX, SERVO_MIN, SERVO_MAX);
    servo.write(angle);
  }

  void reset()
  {
    position = offset;
    move();
  }

  void print()
  {
    char str[80];
    sprintf(str, "pin: %d\tpos: %d\toffset: %4d\t range: %d\t freq: %.1f\t mode: %d",
            pin, position, offset, range, frequency, mode);
    Serial.println(str);
  }

private:
  uint8_t pin;
};

#endif