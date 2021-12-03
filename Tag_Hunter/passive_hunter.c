/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "sl_simple_button.h"
#include "sl_simple_led_instances.h"
#include "sl_pwm.h"
#include "string.h"

#define CONN_INTERVAL_MIN             80   //100ms
#define CONN_INTERVAL_MAX             80   //100ms
#define CONN_SLAVE_LATENCY            0    //no latency
#define CONN_TIMEOUT                  100  //1000ms
#define CONN_MIN_CE_LENGTH            0
#define CONN_MAX_CE_LENGTH            0xffff

#define SCAN_INTERVAL                 16   //10ms
#define SCAN_WINDOW                   16   //10ms
#define SCAN_PASSIVE                  0
#define SCAN_ACTIVE                   1

//static bool report_button_flag = false;
static int8_t rssi;
typedef enum{
  advertising_on,
  advertising_off
}advertising_cond_t;

typedef enum{
  conn_on,
  conn_off
}conn_state_t;

static advertising_cond_t advertising_cond;
static conn_state_t conn_state;
// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static uint8_t find_name_in_advertisement(uint8_t *data, uint8_t len);
static uint8_t find_channel_in_advertisement(uint8_t *data, uint8_t len);
void sl_button_on_change(const sl_button_t *handle);
static uint8_t connection;
static uint8_t device_name[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
extern sl_pwm_instance_t sl_pwm_pwm0;
static uint16_t duty_cycle;
static bool name_flag = false;
static uint8_t channel = 0x01;
/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert_status(sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);
      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);

      // Create scanning set
      app_log_info("setting scanner passive.\n");
      sc = sl_bt_scanner_set_mode(gap_1m_phy, SCAN_PASSIVE);
      app_assert_status(sc);
      // Set scan interval and scan window
      app_log_info("setting scanner param.\n");
      sc = sl_bt_scanner_set_timing(gap_1m_phy, SCAN_INTERVAL, SCAN_WINDOW);
      app_assert_status(sc);
      // Set the default connection parameters for subsequent connections
      sc = sl_bt_connection_set_default_parameters(CONN_INTERVAL_MIN,
                                                   CONN_INTERVAL_MAX,
                                                   CONN_SLAVE_LATENCY,
                                                   CONN_TIMEOUT,
                                                   CONN_MIN_CE_LENGTH,
                                                   CONN_MAX_CE_LENGTH);
      app_assert_status(sc);
      // Start scanning - looking for led devices
      app_log_info("setting scanner active.\n");
      advertising_cond = advertising_off;
      conn_state = conn_off;
      sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
      app_assert_status_f(sc,
                          "Failed to start discovery #1\n");
      break;


    // -------------------------------
    case sl_bt_evt_scanner_scan_report_id:
          // Parse advertisement packets
      //app_log_info("inside scanner scan info");
      //app_log_info("packet type = %x",evt->data.evt_scanner_scan_report.packet_type);
      if (evt->data.evt_scanner_scan_report.packet_type == 0) {
             // If a tag advertisement is found...
             if (find_name_in_advertisement(&(evt->data.evt_scanner_scan_report.data.data[0]),
                                               evt->data.evt_scanner_scan_report.data.len) != 0) {

                 app_log_info("found name.");
                 name_flag = true;
             }
             else
               name_flag = false;
             if(name_flag){
                 // scan response packet received. packet type = 4
                 if (find_channel_in_advertisement(&(evt->data.evt_scanner_scan_report.data.data[0]),
                                                                evt->data.evt_scanner_scan_report.data.len) != 0){
                          app_log_info("found channel.");
                          rssi = evt->data.evt_scanner_scan_report.rssi;
                          app_log_info("rssi = %d",rssi);
                          if(abs(rssi)<100)
                            duty_cycle = 100 - abs(rssi);
                          else
                            duty_cycle = 0;
                          sl_pwm_set_duty_cycle(&sl_pwm_pwm0, duty_cycle);
                          sl_pwm_start(&sl_pwm_pwm0);
                          app_log_info("pwm duty cycle: %d",sl_pwm_get_duty_cycle(&sl_pwm_pwm0));
                        }
                 else
                   sl_pwm_stop(&sl_pwm_pwm0);
             }
           }

       break;
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      conn_state = conn_on;
      connection = evt->data.evt_connection_opened.connection;
      //stop advertising
      sc = sl_bt_advertiser_stop(advertising_set_handle);
      app_assert_status(sc);
      advertising_cond = advertising_off;
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_bt_evt_gatt_server_user_write_request_id:
          //received user write request
          if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_TAG) {
            // Write user supplied value to TAG name.
              //app_log_info("data length= %x\n",evt->data.evt_gatt_server_attribute_value.value.len);
              uint8_t len = evt->data.evt_gatt_server_attribute_value.value.len;
              for(uint8_t i=0;i<len;i++){
                  device_name[i] = evt->data.evt_gatt_server_attribute_value.value.data[i];
              }
            sl_bt_gatt_server_send_user_write_response(
                evt->data.evt_gatt_server_user_write_request.connection,
                gattdb_TAG, SL_STATUS_OK);
            app_log_info("device name = %s",device_name);
          }
          if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_channel) {
            // Write user supplied value to channel.
              //app_log_info("data length= %x\n",evt->data.evt_gatt_server_attribute_value.value.len);
            sl_bt_gatt_server_send_user_write_response(
                evt->data.evt_gatt_server_user_write_request.connection,
                gattdb_channel, SL_STATUS_OK);
                channel = evt->data.evt_gatt_server_attribute_value.value.data[0];
            app_log_info("new channel: %x",channel);
          }
          break;

    case sl_bt_evt_system_external_signal_id:
      if(advertising_cond == advertising_on){
          sc = sl_bt_advertiser_stop(advertising_set_handle);
          app_assert_status(sc);
          advertising_cond = advertising_off;

          //turn off led
          sl_led_turn_off(&sl_led_led0);

          //start scanning again
          sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
          app_assert_status(sc);
      }

      else if(advertising_cond == advertising_off && conn_state == conn_on){
          sc = sl_bt_connection_close(connection);
          app_assert_status(sc);
          sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
          app_assert_status(sc);
          sl_led_turn_off(&sl_led_led0);
          conn_state = conn_off;
      }
      else if(advertising_cond == advertising_off && conn_state == conn_off){
          // Start general advertising and enable connections.
          sc = sl_bt_scanner_stop();
          app_assert_status(sc);

          sc = sl_bt_advertiser_start(
            advertising_set_handle,
            sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
          app_assert_status(sc);
          sl_led_turn_on(&sl_led_led0);
          advertising_cond = advertising_on;
      }
      break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
static uint8_t find_name_in_advertisement(uint8_t *data, uint8_t len)
{
  uint8_t ad_field_length;
  uint8_t ad_field_type;
  uint8_t i = 0;
  // Parse advertisement packet
  while (i < len) {
    ad_field_length = data[i];
    //app_log_info("packet part %d, field length = %d",i,ad_field_length);
    ad_field_type = data[i + 1];
    //app_log_info("packet part %d, field type = %d",i,ad_field_type);
    if (ad_field_type == 0x09) {
        // field matched with name field value
        // matching name
        for(uint8_t j = 0;j<ad_field_length-1;j++){
            //app_log_info("recieved data = %x, device name = %x",data[i+j+2], device_name[j]);
        }
      if (memcmp(&data[i + 2], device_name, ad_field_length-1) == 0) {
        return 1;
      }
    }
    // advance to the next AD struct
    i = i + ad_field_length + 1;
  }
  return 0;
}
static uint8_t find_channel_in_advertisement(uint8_t *data, uint8_t len)
{
  uint8_t ad_field_length;
  uint8_t ad_field_type;
  uint8_t i = 0;
  // Parse advertisement packet
  while (i < len) {
      ad_field_length = data[i];
      //app_log_info("packet part %d, field length = %d",i,ad_field_length);
      ad_field_type = data[i + 1];
      //app_log_info("packet part %d, field type = %d",i,ad_field_type);
    if (ad_field_type == 0x40) {
        for(uint8_t j = 0;j<ad_field_length-1;j++){
            //app_log_info("recieved data = %x, channel = %x",data[i+j+2], channel);
        }
        // field matched with channel field value
        // matching channels
      if (data[i+2] == channel) {
        return 1;
      }
    }
    // advance to the next AD struct
    i = i + ad_field_length + 1;
  }
  return 0;
}
void sl_button_on_change(const sl_button_t *handle)
{
  sl_simple_button_context_t *ctxt = ((sl_simple_button_context_t*)handle[0].context);
  if(ctxt->state){
      sl_bt_external_signal(1);
  }
}
