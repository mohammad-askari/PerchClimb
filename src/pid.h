#ifndef PID_H
#define PID_H

class PID {
  public:
    PID(float kp, float ki, float kd, float dt, float min_output, float max_output);
    void setSetpoint(float setpoint);
    float compute(float measurement);
  
  private:
    float kp_;
    float ki_;
    float kd_;
    float dt_;
    float setpoint_;
    float integral_;
    float prev_error_;
    float min_output_;
    float max_output_;
};

#endif