#include "Actuator.h"

// initializes variables, attaches servo pins, and moves to initial position
void Actuator::init(byte pin, int offset, int range,
                    float frequency, drive_t mode) {
  this->pin = pin;
  this->position = offset;
  this->offset = offset;
  this->range = range;
  this->frequency = frequency;
  this->mode = mode;
  this->last_position = 0;
  this->start_time = millis();
  this->servo.attach(pin);

  servo.attach(pin);
  setLimits();
  move();
}

// set variable methods
void Actuator::setRange(int range) {
  this->range = range;
  setLimits();
  move();
}

void Actuator::setOffset(int offset) {
  this->offset = offset;
  setLimits();
  move();
}

void Actuator::setFrequency(float frequency) {
  this->frequency = frequency;
  move();
}

void Actuator::setPosition(int position) {
  this->position = position;
  this->frequency = 0;
  move();
}

void Actuator::setTime(unsigned long start_time) {
  this->start_time = start_time;
}

void Actuator::setLimits() {
  int8_t rng = abs(range);
  min_pos = constrain(offset - rng, RANGE_MIN, RANGE_MAX);
  max_pos = constrain(offset + rng, RANGE_MIN, RANGE_MAX);
}

// main methods to move the servo, or reset its position back to the neutral state
void Actuator::move() {
  updatePosition();
  if (position == last_position)
    return;
  
  int angle;
  int servo_min = SERVO_MIN;
  int servo_max = SERVO_MAX;
  if (range < 0)
  {
    servo_min = SERVO_MAX;
    servo_max = SERVO_MIN;
  }

  angle = map(position, RANGE_MIN, RANGE_MAX, servo_min, servo_max);
  servo.write(angle);
  last_position = position;
}

void Actuator::reset() {
  setPosition(offset);
}

// print variables for debugging purposes
void Actuator::print() {
  byte len = 80;
  char str[len];
  snprintf(str, len, "pin: %d, pos: %4d, offset: %4d, range: %4d, freq: %.1f"
                      ", mode: %d",
            pin, position, offset, range, frequency, mode);
  Serial.println(str);
}

void Actuator::printSignal() {
  Serial.print(">");
  Serial.print(mode == STEP ? "STEP" : "LINEAR");
  Serial.print(":");
  Serial.println(position);
}



// update position using square (step) or triangular (linear) reference signals
void Actuator::updatePosition() {
  if (frequency == 0)
    return;

  float period = 1000.0 / abs(frequency);
  float half_period = period / 2;

  unsigned long elapsed_time = millis() - start_time;
  float dt = fmod(elapsed_time, period);
  bool is_first_half = dt < half_period;

  switch (mode)
  {
    case STEP: {
      if (frequency >= 0)
        position = is_first_half ? max_pos : min_pos;
      else
        position = is_first_half ? min_pos : max_pos;
      break;
    }

    case LINEAR: {
      float a, b, c;
      if (frequency >= 0) {
        a = (max_pos - min_pos) / half_period;
        b = 2.0 * max_pos - min_pos;
        c = min_pos;
      } else {
        a = (min_pos - max_pos) / half_period;
        b = 2.0 * min_pos - max_pos;
        c = max_pos;
      }
      position = is_first_half ? a * dt + c : -a * dt + b;
      break;
    }
  }
}