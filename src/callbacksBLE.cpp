#include "callbacksBLE.h"
#include "main.h"
#include "functions.h"
#include "callbacksTasks.h"

// ——————————————————————————— BLE CONNECT ACTIONS —————————————————————————— //
/**
 * @brief Callback invoked when central device connects to the peripheral
 * @param[in] conn_handle connection handle to where this event happens
 **/
void bleConnect(uint16_t conn_handle)
{
  // get the reference to current connection and print connected device name
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  char central_name[32] = {0};
  conn->getPeerName(central_name, sizeof(central_name));
  Serial.print("BLE Connected to "); 
  Serial.println(central_name);
  setLED(led_pin,'G');

  // customization for throughput maximum data transmission speed
  conn->requestPHY();                // change PHY to 2Mbps (BLE v5.0+)
  conn->requestDataLengthUpdate();   // enable data length extension (BLE v4.2+)
  conn->requestMtuExchange(ble_mtu); // change maximum transmission unit

  // schedule a delayed call to update the connection parameters
  ble_conn_handle = conn_handle;
  ts_ble_conn.restartDelayed(TASK_SECOND);
}

// ————————————————————————— BLE DISCONNECT ACTIONS ————————————————————————— //
/**
 * @brief Callback invoked when a connection is dropped, displaying the reason
 * @param[in] conn_handle connection handle to where this event happens
 * @param[in] reason      BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void bleDisconnect(uint16_t conn_handle, byte reason)
{
  Serial.print("BLE Disconnected, reason = 0x");
  Serial.println(reason, HEX);
  setLED(led_pin,'O');
  
  //!!!!!!!!!!!!!!! // TODO: CHECK WHETHER WE NEED THE CONDITION
  // take immediate actions on connection loss (not intended termination)
  // if (reason != BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION)
    ts_kill.restart();
}