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
  while (bleuart.available())
  {
    int ch = bleuart.read(); // read a single byte from the BLE UART
    processCommand(ch, buffer_len, buffer_idx, buffer);
  }

  // check serial for user input
  while (Serial.available())
  {
    int ch = Serial.read();  // read a single byte from the serial
    processCommand(ch, buffer_len, buffer_idx, buffer);
  }
};


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


void tsBLELost() {
  esc.speed(esc_min);
  for(byte i = 0; i < servo_num; i++) actuator[i].reset();
  ts_climb_on.disable();
  ts_climb_off.disable();
  ts_pre_hover.disable();
  ts_hover_on.disable();
  ts_hover_off.disable();
  ts_motor_update.disable();
  ts_data_logger.disable();
};


void tsClimbOn() {
  // reset the data logger buffer and index to zero
  memset(&exp_data, 0, sizeof(exp_data_t) * data_len);
  data_idx = 0;

  ts_climb_off  .restartDelayed(TASK_SECOND * exp_duration);
  ts_data_logger.restart();
  ts_motor_update.enable();
  Serial.println("Climb On");

  // synchronize the servo start times
  unsigned long now = millis();
  start_time = now;
  for(byte i = 0; i < servo_num; i++) {
    actuator[i].setTime(now);
  }
  esc.speed(esc_speed);
};


void tsClimbOff() {
  ts_data_logger.disable();
  ts_motor_update.disable();
  for(byte i = 0; (i < servo_num-2) && (i != 1); i++) actuator[i].reset();
  actuator[4].setPosition(actuator[4].offset - actuator[4].range);
  actuator[5].setPosition(actuator[5].offset - actuator[5].range);

  while (esc_speed > esc_min){ // slow down propeller gradually
    esc_speed -= 10;
    esc.speed(esc_speed);
    delay(100);
    // Serial.println(esc_speed);
  }
  Serial.println("Climb Off Smooth");
};


void tsPreHover() {
  // reset the data logger buffer and index to zero
  memset(&exp_data, 0, sizeof(exp_data_t) * data_len);
  data_idx = 0;

  ts_hover_on   .restartDelayed(TASK_SECOND * pre_hover_time);
  ts_data_logger.restart();
  ts_motor_update.enable();
  Serial.println("Pre Hover On");

  // synchronize the servo start times
  unsigned long now = millis();
  start_time = now;
  for(byte i = 0; i < servo_num; i++) {
    actuator[i].setTime(now);
  }
  esc.speed(pre_hover_esc);
};


void tsHoverOn() {
  if (ts_hover_on.isFirstIteration()) {
    Serial.println("Hover On");
    if (hover_use_hooks)
    {
      actuator[4].setPosition(actuator[4].offset - actuator[4].range);
      actuator[5].setPosition(actuator[5].offset + actuator[5].range);
    }
  }

  if (hover_use_hooks && transition_esc > esc_speed){ // slow down propeller gradually
    transition_esc -= 10;
    esc.speed(transition_esc);
    Serial.println(millis());
  } 
  else {
    esc.speed(esc_speed);
    ts_hover_on.disable();
    ts_hover_off.restartDelayed(TASK_SECOND * exp_duration);
  }
};


void tsHoverOff() {
  ts_data_logger.disable();
  ts_motor_update.disable();
  for(byte i = 0; (i < servo_num-2) && (i != 1); i++) actuator[i].reset();

  while (esc_speed > esc_min){ // slow down propeller gradually
    esc_speed -= 10;
    esc.speed(esc_speed);
    delay(100);
    // Serial.println(esc_speed);
  }
  Serial.println("Hover Off Smooth");
};


void tsMotorUpdate() {
  for(byte i = 0; i < servo_num; i++) actuator[i].move();

  if (DEBUG) {
    for(byte i = 0; i < servo_num; i++) {
      actuator[i].printSignal(i);
    }
  }
};


void tsDataLogger() {
      if (data_idx >= data_len) {
        Serial.println("Data Logger Buffer Full!");
        ts_data_logger.disable();
        return;
      }

      float ax, ay, az;
      float gx, gy, gz;

      ax = imu.readFloatAccelX();
      ay = imu.readFloatAccelY();
      az = imu.readFloatAccelZ();
      gx = imu.readFloatGyroX();
      gy = imu.readFloatGyroY();
      gz = imu.readFloatGyroZ();
      filter.updateIMU(gx, gy, gz, ax, ay, az);

      //!!!!!!!!!!!!!!! WRITE AN ALWAYS RUNNING TASK TO FILTER CURRENT
      exp_data[data_idx].time     = millis() - start_time;
      exp_data[data_idx].current  = analogRead(current_pin);
      exp_data[data_idx].roll     = round(filter.getRoll());
      exp_data[data_idx].pitch    = round(filter.getPitch());
      exp_data[data_idx].yaw      = round(filter.getYaw());

      data_idx++;
};


void tsDataTransfer() {
    Serial.println("Data Transfer Started");
    char buffer[32];
    delay(300);
    sprintf(buffer, "meta: %d\n", (int)ceil(data_idx / 6) );
    bleuart.write( (uint8_t*) buffer, strlen(buffer));
    delay(300);

    uint8_t *P;
    // Serial.println("SENDING DATA NOW");
    for (int i = 0; i < data_idx; i = i+6) {                 
        P = (uint8_t*) exp_data;
        P += sizeof(exp_data_t) * i;
        bleuart.write(P, sizeof(exp_data_t) * 6);
        delay(250);
    }

  Serial.println("Data Transfer Complete");
};