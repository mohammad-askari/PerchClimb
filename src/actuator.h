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

  // object constructor
  Actuator() {}

  // initializes variables, attaches servo pins, and moves to initial position
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
    setConstraints();
    move();
  }

  // set variable methods
  void setRange(int8_t range) { this->range = range; setConstraints(); }
  void setOffset(int8_t offset) { this->offset = offset; setConstraints(); }
  void setPosition(int8_t position) { this->position = position; move(); }

  void setConstraints()
  {
    int8_t rng = abs(range);
    min_pos = constrain(offset - rng, RANGE_MIN, RANGE_MAX);
    max_pos = constrain(offset + rng, RANGE_MIN, RANGE_MAX);
  }

  // main methods to move the servo, or reset its position back to the neutral state
  void move()
  {
    int pos = constrain(position, min_pos, max_pos);
    int angle = map(pos, RANGE_MIN, RANGE_MAX, SERVO_MIN, SERVO_MAX);
    servo.write(angle);
  }

  void reset()
  {
    position = offset;
    move();
  }

  // print variables for debugging purposes
  void print()
  {
    byte len = 80;
    char str[len];
    snprintf(str, len, "pin: %4d, pos: %4d, offset: %4d, range: %4d, freq: %.1f"
             ", mode: %d", pin, position, offset, range, frequency, mode);
    Serial.println(str);
  }

private:
  uint8_t pin;
  int8_t min_pos;
  int8_t max_pos;
};

#endif