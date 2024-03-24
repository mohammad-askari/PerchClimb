#include "pid.h"

PID::PID(float kp, float ki, float kd, float dt, float min_output, float max_output) :
  kp_(kp), ki_(ki), kd_(kd), dt_(dt), setpoint_(0), integral_(0), prev_error_(0), min_output_(min_output), max_output_(max_output) {}

void PID::setSetpoint(float setpoint) {
  setpoint_ = setpoint;
}

float PID::compute(float measurement) {
  float error = setpoint_ - measurement;
  integral_ += error * dt_;
  
  // Anti-windup - limit integral term to fraction of max_output
  float prop = 0.5;
  if (integral_ > max_output_*prop)
    integral_ = max_output_*prop;
  else if (integral_ < min_output_*prop)
    integral_ = min_output_*prop;

  float derivative = (error - prev_error_) / dt_;
  prev_error_ = error;
  float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // Output saturation
  if (output > max_output_)
    output = max_output_;
  else if (output < min_output_)
    output = min_output_;

  return output;
}