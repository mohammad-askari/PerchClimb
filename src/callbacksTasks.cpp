#include "callbacksTasks.h"
#include "main.h"
#include "functions.h"
#include "communication.h"

uint16_t ble_conn_handle;

// ——————————————————————— PARSE INCOMING USER INPUTS ——————————————————————— //
/**
 * @brief Processes incoming serial/ble data bytes and parses into the CLI.
 **/
void tsParser() {
  // check serial for user input
  if(Serial.available())
  {
    int ch = Serial.read();  // read a single byte from the serial
    processCommandSerial(ch);
  }
  //else
  //  delay(10);
};

void tsBleParser() {
  uint8_t bleBuffer[MAX_BLUETOOTH_PACKET_LEN];
  uint8_t bleDataLen = 0;
  
  // read BLE data
  if(bleuart.available())
  {
    bleDataLen = bleuart.read(bleBuffer, MAX_BLUETOOTH_PACKET_LEN);
    Serial.print("Read BLE ");
    Serial.print(bleDataLen);
    Serial.println(" bytes");
    if(bleDataLen > 0)
      decodeBytes(bleBuffer, bleDataLen);
  }
  //else
  //{
    //delay(10);
    //Serial.println("No BLE Data");
  //}
};


// ————————————————————————— FILTER SENSOR READINGS ————————————————————————— //
/**
 * @brief Filters the IMU and current sensor readings and stores them in memory.
 **/
void tsSensors() {
  // apply sensor fusion algorithm to filter the IMU readings
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();
  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // update the recorded sensor readings in memory
  current = analogRead(current_pin);
  roll    = filter.getRoll();
  pitch   = filter.getPitch();
  yaw     = filter.getYaw();
  if (DEBUG) {
    Serial.print(">roll: "); Serial.println(roll);
    Serial.print(">pitch: "); Serial.println(pitch);
    Serial.print(">yaw: "); Serial.println(yaw);
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
void tsClimbOn() {
  // reset the data logger buffer and index to zero
  memset(&exp_data, 0, sizeof(exp_data_t) * data_len);
  data_idx = 0;

  Serial.println("Climb On");
  ts_climb_off  .restartDelayed(TASK_SECOND * exp_duration);
  ts_data_logger.restart();
  ts_motor_update.enable();

  // synchronize the servo start times
  unsigned long now = millis();
  start_time = now;
  for(byte i = 0; i < servo_num; i++) {
    actuator[i]->setTime(now);
  }
  esc.speed(esc_speed);
};


void tsClimbOff() {
  Serial.println("Climb Off Smooth");
  ts_data_logger.disable();
  ts_motor_update.disable();

  aileron.reset();
  elevator.reset();
  wing_lock.reset();

  // enage hooks
  body_hook.setPosition(RANGE_MIN);
  tail_hook.setPosition(RANGE_MIN);

  while (esc_speed > esc_min){ // slow down propeller gradually
    esc_speed -= 10;
    esc.speed(esc_speed);
    // delay(100);
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
  start_time = now;
  for(byte i = 0; i < servo_num; i++) {
    actuator[i]->setTime(now);
  }
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
  start_time = now;
  for(byte i = 0; i < servo_num; i++) {
    actuator[i]->setTime(now);
  }
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
  start_time = now;
  for(byte i = 0; i < servo_num; i++) {
    actuator[i]->setTime(now);
  }

  // reset the takeoff parameters for later use in the main unperching task
  is_start_of_takeoff = false;
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
    aileron.setPosition(RANGE_MAX);    // maximum wing twist
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
  tail_hook.setPosition(25); // bring tail hook in - FIXME: AVOID HARD-CODED
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
    analogWrite(enable_pin, dc_speed);
    Serial.println("Wing Opening Started");
  }
  if (dc_speed != 0 && (elapsed_time-start) >= wing_opening_duration*TASK_SECOND)
  {
    dc_speed = 0;
    analogWrite(enable_pin, 0);
    Serial.println("Wing Opening Completed");
  }
  
  if (DEBUG) {
    for(byte i = 0; i < servo_num; i++) { actuator[i]->printSignal(); }
  }
};


void tsMotorUpdateDisabled() { 
  analogWrite(enable_pin,0); 
};


// ———————————————————— DATA LOGGING & TRANSFER FUNCTIONS ——————————————————— //
void tsDataLogger() {
  if (data_idx >= data_len) {
    Serial.println("Data Logger Buffer Full!");
    ts_data_logger.disable();
    return;
  }

  exp_data[data_idx].time     = millis() - start_time;
  exp_data[data_idx].current  = current;
  exp_data[data_idx].roll     = round(roll);
  exp_data[data_idx].pitch    = round(pitch);
  exp_data[data_idx].yaw      = round(yaw);
  data_idx++;
};


void tsDataTransfer() {
  commPacket_t packet;
  pktFileMetadata_t metadata;
  pktFileContent_t fileContent;
  uint8_t *logArrayPointer;
  const int8_t MAX_NUMBER_OF_LOGS_IN_EACH_PACKET = 5; // move this to global.h or something
  uint8_t dataLen;

  Serial.println("Data Transfer Started");
  
  // send metadata packet so that the client would know how many packets it should expect
  metadata.packetCount = (int)ceil(data_idx / MAX_NUMBER_OF_LOGS_IN_EACH_PACKET);
  createFileMetadataPacket(&packet, &metadata);
  sendPacketViaBLE(&packet);

  delay(300);
  
  for (int i = 0; i < data_idx; i = i + MAX_NUMBER_OF_LOGS_IN_EACH_PACKET)
  {                 
      logArrayPointer = (uint8_t*) exp_data;
      logArrayPointer += sizeof(exp_data_t) * i;

      fileContent.packetNo = i;
      // number of log data is not always a coefficient of MAX_NUMBER_OF_LOGS_IN_EACH_PACKET
      dataLen = data_idx - i >= MAX_NUMBER_OF_LOGS_IN_EACH_PACKET ? 
                  sizeof(exp_data_t) * MAX_NUMBER_OF_LOGS_IN_EACH_PACKET : 
                  sizeof(exp_data_t) * (data_idx - i);
      memcpy(&fileContent.data, logArrayPointer, dataLen);
      fileContent.dataLen = dataLen;

      createFileContentPacket(&packet, &fileContent);
      sendPacketViaBLE(&packet);
      
      delay(30);
  }

  // tell client that file send process is finished
  createFileSentPacket(&packet);
  sendPacketViaBLE(&packet);
  
  Serial.println("Data Transfer Complete");
};


// —————————————————————————— KILL SWITCH FUNCTION —————————————————————————— //
void tsKill() {
  esc.speed(esc_min);
  for(byte i = 0; i < servo_num; i++) actuator[i]->reset();
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