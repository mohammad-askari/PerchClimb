#include "actuator.h"
#include "communication.h"

Actuator::Actuator(const char* name, byte pin, int offset,
                   int range, float frequency, signal_t mode) {
  this->name = name;
  this->pin = pin;
  this->position = offset;
  this->offset = offset;
  this->range = range;
  this->frequency = frequency;
  this->mode = mode;
}

void Actuator::init() {
  this->last_position = offset + 1; // initialize differently from position
  servo.attach(pin);
  setLimits();
  setTime(millis());
  move();
}

// initializes variables, attaches servo pins, and moves to initial position
// void Actuator::init(const char* name, byte pin, int offset, int range,
//                     float frequency, signal_t mode) {
//   this->name = name;
//   this->pin = pin;
//   this->position = offset;
//   this->last_position = offset + 1; // initialize differently from position
//   this->offset = offset;
//   this->range = range;
//   this->frequency = frequency;
//   this->mode = mode;

//   servo.attach(pin);
//   setLimits();
//   setTime(millis());
//   move();
// }

// set variable methods
void Actuator::setOffset(int offset) {
  this->offset = offset;
  setLimits();
  setTime(millis());
  move();
}

void Actuator::setRange(int range) {
  this->range = range;
  setLimits();
  setTime(millis());
  move();
}

void Actuator::setPosition(int position) {
  this->position = position;
  setTime(millis());
  this->frequency = 0;
  move();
}

void Actuator::setFrequency(float frequency) {
  this->frequency = frequency;
  setTime(millis());
}

void Actuator::setMode(signal_t mode) {
  this->mode = mode;
  setTime(millis());
}

void Actuator::setTime(unsigned long start_time) {
  this->start_time = start_time;
}

void Actuator::setLimits() {
  int8_t rng = abs(range);
  min_pos = constrain(offset - rng, RANGE_MIN, RANGE_MAX);
  max_pos = constrain(offset + rng, RANGE_MIN, RANGE_MAX);
}

int Actuator::getPosition() {
  return position;
}

// main methods to move the servo or reset position back to the neutral state
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
  byte len = 75;
  char str[len];
  snprintf(str, len, "%-10s pos: %-+4d  offset: %-+4d  range: %-+4d  freq: "
           "%-+.1f  mode: %d", name, position, offset, range, frequency, mode);
  Serial.println(str);
  sendStringAsStringPacketViaBLE(str + String("\n"));
}

void Actuator::printSignal() {
  Serial.print(">");
  Serial.print(name);
  Serial.print(":");
  Serial.println(position);
  sendStringAsStringPacketViaBLE(String(">") + String(name) + String(":") + String(position) + String("\n"));
}



// update position using square (step) or triangular (linear) reference signals
void Actuator::updatePosition() {
  if (frequency == 0) {
    position = constrain(position, min_pos, max_pos);
    return;
}

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

    case RAMP: {
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