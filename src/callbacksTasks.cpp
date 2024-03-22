#include "callbacksTasks.h"
#include "main.h"
#include "functions.h"

uint16_t ble_conn_handle;

// ——————————————————————— PARSE INCOMING USER INPUTS ——————————————————————— //
/**
 * @brief Processes incoming serial/ble data bytes and parses into the CLI.
 **/
void tsParser() {
  // check BLE UART for user input
  while (bleuart.available()) {
    int ch = bleuart.read(); // read a single byte from the BLE UART
    processCommand(ch, buffer_len, buffer_idx, buffer);
  }

  // check serial for user input
  while (Serial.available()) {
    int ch = Serial.read();  // read a single byte from the serial
    processCommand(ch, buffer_len, buffer_idx, buffer);
  }
};


// ————————————————————————— FILTER SENSOR READINGS ————————————————————————— //
/**
 * @brief Filters the IMU and current sensor readings and stores them in memory.
 **/
void tsSensors() {
  unsigned long n  = ts_sensors.getRunCounter() - 1;
  int num_samples  = filt_freq / log_freq;
  int idx = n % num_samples;
  
  // store current readings at a higher frequency for later filtering
  current_samples[idx] = analogRead(current_pin);

  // apply moving average filter to current and sensor fusion to IMU readings
  if (idx == 0) {
    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();
    float gx = imu.readFloatGyroX();
    float gy = imu.readFloatGyroY();
    float gz = imu.readFloatGyroZ();
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // update the recorded sensor readings in memory
    unsigned long current_sum = 0;
    for (int i = 0; i < num_samples; i++) { current_sum += current_samples[i]; }
    current_average = current_sum / num_samples;
    roll  = filter.getRoll();
    pitch = filter.getPitch();
    yaw   = filter.getYaw();

    // ensure Euler angles are within [-180 +180] degree range
    roll  = clipAngle(roll);
    pitch = clipAngle(pitch);
    yaw   = clipAngle(yaw);

    if (DEBUG) {
      Serial.print(">roll: "); Serial.println(roll);
      Serial.print(">pitch: "); Serial.println(pitch);
      Serial.print(">yaw: "); Serial.println(yaw);
    }
  }
};

// ———————————————————————————— BLE TASK FUNCTION ——————————————————————————— //
void tsBLEConn() {
  // get the reference to current connection and print connected device name
  BLEConnection* conn = Bluefruit.Connection(ble_conn_handle);

  // update and print the current connection parameters
  uint8_t  PHY = conn->getPHY();
  uint16_t DLE = conn->getDataLength();
  uint16_t MTU = conn->getMtu();

  Serial.print("BLE PHY: "); Serial.println(PHY);
  Serial.print("BLE DLE: "); Serial.println(DLE);
  Serial.print("BLE MTU: "); Serial.println(MTU);
};


// ———————————————————————————— ASCENT FUNCTIONS ———————————————————————————— //
void tsPreClimb() {
  // reset the data logger buffer and index to zero
  memset(&exp_data, 0, sizeof(exp_data_t) * data_len);
  data_idx = 0;

  Serial.println("Pre Climb On");

  // if wing opening is set, account for its time
  unsigned long delay_duration = 0;
  if (is_wing_opening) {
    delay_duration  = TASK_SECOND * wing_opening_duration;
    is_wing_opening = false;
    climb_wing_loosening = true;
    clutch.forward();
    clutch.speed(dc_speed);
    Serial.println("Wing Loosening Started");
  }

  ts_climb_on   .restartDelayed(delay_duration);
  ts_data_logger.restart();
};


void tsClimbOn() {
  Serial.println("Climb On");
  ts_climb_off.restartDelayed(TASK_SECOND * exp_duration);
  ts_motor_update.enable();

    // disenage hooks
    body_hook.setPosition(RANGE_MIN);
    tail_hook.setPosition(+30); // FIXME: AVOID HARD-CODED (TREE-CLIMBING)

  // stop wing loosening if activated
  if (climb_wing_loosening) {
    clutch.stop();
    Serial.println("Wing Loosening Completed");
  }

  // synchronize the servo start times
  unsigned long now = millis();
  for(byte i = 0; i < servo_num; i++) actuator[i]->setTime(now);
  esc.speed(esc_speed);
};


void tsClimbOff() {
  if (ts_climb_off.isFirstIteration()) {
    Serial.println("Climb Off Smooth");

    aileron.reset();
    rudder.reset();

    // enage hooks
    body_hook.setPosition(RANGE_MIN);
    tail_hook.setPosition(RANGE_MIN);

    // if wing loosening was activated, revert motor direction to close wings
    if (climb_wing_loosening) {
      is_wing_opening = true;
      climb_wing_loosening = false;
      clutch.reverse();
    }
  }

  // slow down propeller gradually if needed
  if (esc_speed > esc_min){
    esc_speed -= 1;
    esc.speed(esc_speed);
  } // otherwise stop the mission
  else {
    ts_climb_off.disable();
    ts_data_logger.disable();
    ts_motor_update.disable();
  }
};


// ———————————————————————————— DESCENT FUNCTIONS ——————————————————————————— //
void tsPreDescent() {
  // reset the data logger buffer and index to zero
  memset(&exp_data, 0, sizeof(exp_data_t) * data_len);
  data_idx = 0;

  Serial.println("Pre Descent On");
  ts_descent_on .restartDelayed(TASK_SECOND * pre_descent_time);
  ts_data_logger.restart();

  // enage hooks
  body_hook.setPosition(RANGE_MIN);
  tail_hook.setPosition(RANGE_MIN);

  // synchronize the servo start times
  unsigned long now = millis();
  for(byte i = 0; i < servo_num; i++) actuator[i]->setTime(now);
  esc.speed(pre_descent_esc);
};


void tsDescentOn() {
  static bool is_start_of_transition = false;
  unsigned long dt = ts_descent_on.getInterval();
  unsigned long n  = ts_descent_on.getRunCounter() - 1;
  unsigned long elapsed_time = dt * n * TASK_MILLISECOND;

  if (ts_descent_on.isFirstIteration()) {
    Serial.println("Descent On");
    ts_motor_update.enable();
    is_start_of_transition = true;  // enable the flag for later

    // disenage hooks
    body_hook.setPosition(RANGE_MAX);
    tail_hook.setPosition(RANGE_MAX);
  }

  // if during main experiment, do a controlled descent or freefall
  if (elapsed_time < TASK_SECOND * exp_duration)
  {
    if (!is_freefall_mode && descent_freq != 0) {
      float period = 1000.0 / abs(descent_freq);
      float half_period = period / 2;

      float dt = fmod(elapsed_time, period);
      bool is_first_half = dt < half_period;

      esc.speed(is_first_half ? esc_speed : transition_esc);
    }
    else esc.speed(esc_speed);
  }
  // if main experiment is over, do post-descent hover then descent off
  else
  {
    if (is_start_of_transition) {
      is_start_of_transition = false; // reset flag
      ts_motor_update.disable();

      aileron.reset();
      elevator.reset();
      wing_lock.reset();

      // enage hooks
      body_hook.setPosition(RANGE_MIN);
      tail_hook.setPosition(RANGE_MIN);
    }

    // if transition is over, trigger the descent off after post-descent hover
    if (transition_esc <= post_descent_esc) {
      esc_speed = post_descent_esc;
      esc.speed(esc_speed);
      ts_descent_on.disable();
      ts_descent_off.restartDelayed(TASK_SECOND * post_descent_time);
    }

    // slow down propeller gradually
    esc.speed(transition_esc);
    transition_esc -= 1;
  }
};


void tsDescentOff() {
  Serial.println("Descent Off Smooth");
  ts_data_logger.disable();

  while (esc_speed > esc_min){ // slow down propeller gradually
    esc_speed -= 10;
    esc.speed(esc_speed);
    delay(100);
  }
};


// ————————————————————————————— HOVER FUNTIONS ————————————————————————————— //
void tsPreHover() {
  // reset the data logger buffer and index to zero
  memset(&exp_data, 0, sizeof(exp_data_t) * data_len);
  data_idx = 0;
  
  Serial.println("Pre Hover On");
  ts_hover_on   .restartDelayed(TASK_SECOND * pre_hover_time);
  ts_data_logger.restart();
  ts_motor_update.enable();

  // synchronize the servo start times
  unsigned long now = millis();
  for(byte i = 0; i < servo_num; i++) actuator[i]->setTime(now);
  esc.speed(pre_hover_esc);
};


void tsHoverOn() {
  if (ts_hover_on.isFirstIteration()) {
    Serial.println("Hover On");
    if (hover_use_hooks)
    {
      body_hook.setPosition(RANGE_MIN);
      tail_hook.setPosition(RANGE_MIN);
    }
  }

  if (hover_use_hooks && transition_esc > esc_speed){ // slow down propeller gradually
    esc.speed(transition_esc);
    transition_esc -= 10;
  } 
  else {
    esc.speed(esc_speed);
    ts_hover_on.disable();
    ts_hover_off.restartDelayed(TASK_SECOND * exp_duration);
  }
};


void tsHoverOff() {
  Serial.println("Hover Off Smooth");
  ts_data_logger.disable();
  ts_motor_update.disable();
  
  aileron.reset();
  elevator.reset();
  wing_lock.reset();

  while (esc_speed > esc_min){ // slow down propeller gradually
    esc_speed -= 10;
    esc.speed(esc_speed);
    // delay(100);
  }
};


// ———————————————————————————— UNPERCH FUNTIONS ———————————————————————————— //
void tsPreUnperch() {
  // reset the data logger buffer and index to zero
  memset(&exp_data, 0, sizeof(exp_data_t) * data_len);
  data_idx = 0;
  
  Serial.println("Pre Unperch On");
  ts_unperch_on .restartDelayed(TASK_SECOND * pre_unperch_duration);
  ts_data_logger.restart();
  ts_motor_update.enable();

  // punch the tail hook and start hovering
  tail_hook.setPosition(RANGE_MAX);
  esc.speed(pre_unperch_esc);

  // synchronize the servo start times
  unsigned long now = millis();
  for(byte i = 0; i < servo_num; i++) actuator[i]->setTime(now);

  // reset the takeoff parameters for later use in the main unperching task
  is_start_of_takeoff = false;
  is_level_flight = false;
  takeoff_start_time  = 0;
}


void tsUnperchOn() {
  unsigned long dt = ts_unperch_on.getInterval();
  unsigned long n  = ts_unperch_on.getRunCounter() - 1;
  unsigned long elapsed_time = dt * n * TASK_MILLISECOND;

  // initiate the pitch back movement
  if (ts_unperch_on.isFirstIteration()) {
    Serial.println("Unperch On");
    is_start_of_takeoff = true;  // enable the flag for later

    // cut off thrust and disenage the body hook
    esc.speed(tilt_esc);
    wing_lock.setPosition(RANGE_MAX);
    body_hook.setPosition(RANGE_MAX);
  }

  // start takeoff if pitch angle drops below the desired pitch for takeoff
  if (is_start_of_takeoff && fabs(pitch) <= takeoff_pitch) {
    Serial.println("Takeoff Initiated");
    is_start_of_takeoff = false;  // reset flag
    takeoff_start_time  = elapsed_time;
    esc.speed(takeoff_esc);
    elevator.reset();  // reset the elevator
  }

  // after the initial takeoff duration, do wing twist and fly away
  unsigned long since_takeoff = (elapsed_time - takeoff_start_time);
  if (!is_start_of_takeoff && since_takeoff >= TASK_SECOND * takeoff_duration) {
    Serial.println("Fly Away Initiated");
    esc.speed(esc_speed);

    // check if we are out of gimbal lock
    if (!is_level_flight && fabs(pitch) < 45.0){
      is_level_flight = true;
      pid_yaw.setSetpoint(yaw);
    }
    // only do active control if out of gimbal lock
    if (is_level_flight) {
      aileron.setPosition(pid_roll.compute(roll));
      elevator.setPosition(pid_pitch.compute(pitch));
      rudder.setPosition(pid_yaw.compute(yaw));
    } else{
      aileron.setPosition(RANGE_MAX); // otherwise feed forward
    }

    tail_hook.setPosition(RANGE_MIN);  // grasp the tail hook
    ts_unperch_on.disable();
    ts_unperch_off.restartDelayed(TASK_SECOND * exp_duration);
  }
};


void tsUnperchOff() {
  Serial.println("Unperch Off");
  ts_data_logger.disable();
  ts_motor_update.disable();

  esc_speed = esc_min;
  esc.speed(esc_speed);
  tail_hook.setPosition(25); // bring tail hook in - // FIXME: AVOID HARD-CODED
}


// ————————————————————————— MOTOR UPDATE FUNCTIONS ————————————————————————— //
void tsMotorUpdate() {
  for(byte i = 0; i < servo_num; i++) actuator[i]->move();

  static unsigned long start = 0;
  unsigned long dt = ts_motor_update.getInterval();
  unsigned long n  = ts_motor_update.getRunCounter() - 1;
  unsigned long elapsed_time = dt * n * TASK_MILLISECOND;

  if (is_wing_opening) {
    start = elapsed_time;
    is_wing_opening = false;
    clutch.speed(dc_speed);
    Serial.println("Wing Moving Started");
  } 
  
  // HACK: AVOID CLIMB FLAG HERE
  bool time_to_stop = (elapsed_time-start) >= wing_opening_duration*TASK_SECOND;
  if (dc_speed!=0 && !climb_wing_loosening && time_to_stop) {
    dc_speed = 0;
    clutch.stop();
    Serial.println("Wing Moving Completed");
  }
  
  if (DEBUG) {
    for(byte i = 0; i < servo_num; i++) actuator[i]->printSignal();
  }
};


void tsMotorUpdateDisabled() { 
  clutch.stop();
};


// ———————————————————— DATA LOGGING & TRANSFER FUNCTIONS ——————————————————— //
void tsDataLogger() {
  static unsigned long start_time = 0;
  if (ts_data_logger.isFirstIteration()) start_time = millis();

  if (data_idx >= data_len) {
    Serial.println("Data Logger Buffer Full!");
    ts_data_logger.disable();
    return;
  }

  exp_data[data_idx].time      = millis() - start_time;
  exp_data[data_idx].current   = current_average;
  exp_data[data_idx].roll      = round(roll);
  exp_data[data_idx].pitch     = round(pitch);
  exp_data[data_idx].yaw       = round(yaw);
  cmd_data[data_idx].throttle  = esc.getSpeed();
  cmd_data[data_idx].aileron   = aileron  .getPosition();
  cmd_data[data_idx].elevator  = elevator .getPosition();
  cmd_data[data_idx].rudder    = rudder   .getPosition();
  cmd_data[data_idx].clutch    = wing_lock.getPosition();
  cmd_data[data_idx].body_hook = body_hook.getPosition();
  cmd_data[data_idx].tail_hook = tail_hook.getPosition();
  cmd_data[data_idx].wing_open = clutch.getSpeed();
  data_idx++;
};


void tsDataTransfer() {
  Serial.println("Data Transfer Started");
  const byte packet_len  = 60;
  const int python_delay = 250;
  char meta_str[32];

  // transfer only logged sensor data
  if (!transfer_include_commands) {
    sprintf(meta_str, "meta: %d\n", (int)ceil(data_idx / 6) );
    bleuart.write( (uint8_t*) meta_str, strlen(meta_str));
    delay(python_delay);
    
    uint8_t *P;
    for (int i = 0; i < data_idx; i=i+6) {
        P = (uint8_t*) exp_data;
        P += sizeof(exp_data_t) * i;
        bleuart.write(P, sizeof(exp_data_t) * 6);
        delay(python_delay);
  }
  }
  // transfer logged sensor data combined with actuator commands
  else {
    const int sets_per_packet = packet_len / (sizeof(exp_data_t) + sizeof(cmd_data_t));
    sprintf(buffer, "meta: %d\n", (int)ceil(data_idx / 3) );
    bleuart.write( (uint8_t*) buffer, strlen(buffer));
    delay(python_delay);

    uint8_t packet[packet_len];
    int packet_idx = 0;
    for (int i = 0; i < data_idx; i++) {
      memcpy(packet + packet_idx, &exp_data[i], sizeof(exp_data_t));
      packet_idx += sizeof(exp_data_t);
      memcpy(packet + packet_idx, &cmd_data[i], sizeof(cmd_data_t));
      packet_idx += sizeof(cmd_data_t);

      // send the packet once it is full or all data has been copied
      // if ((i + 1) % sets_per_packet == 0 || i == data_idx - 1) {
      if ((i + 1) % sets_per_packet == 0) {
          bleuart.write(packet, packet_len);
          delay(250); // Delay between packets
          packet_idx = 0;
      }
    }
  }

  Serial.println("Data Transfer Complete");
};


// —————————————————————————— KILL SWITCH FUNCTION —————————————————————————— //
void tsKill() {
  esc.speed(esc_min);
  clutch.stop();
  aileron.reset();
  elevator.reset();
  rudder.reset();
  ts_pre_climb.disable();
  ts_climb_on.disable();
  ts_climb_off.disable();
  ts_pre_descent.disable();
  ts_descent_on.disable();
  ts_descent_off.disable();
  ts_pre_hover.disable();
  ts_hover_on.disable();
  ts_hover_off.disable();
  ts_pre_unperch.disable();
  ts_unperch_on.disable();
  ts_unperch_off.disable();
  ts_motor_update.disable();
  ts_data_logger.disable();
};