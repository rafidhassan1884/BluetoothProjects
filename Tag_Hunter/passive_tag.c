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

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static int16_t tx_power = 0;
static int16_t set_power = 10;
/*
 *  uint8_t len_flags = 0x02;
  uint8_t type_flags = 0x01;
  uint8_t val_flags = 0x06;

  uint8_t len_manuf = 0x05;
  uint8_t type_manuf = 0xff;
  uint8_t company_LO[2] = {0xff, 0x00};
  uint8_t company_HI[2] = {0x02,0x00};
 
  uint8_t device_name_len = 0x07;
  uint8_t type = 0x09;
        uint8_t name[6] = {0x00,0x00,0x00,0x00,0x00,0x00} //(default 0, can change to anything)

  uint8_t channel_len = 0x02;
  uint8_t type = 0x40;
        uint8_t channel = 0x01; //(default 0x01, change to anything)
 */
static uint8_t scan_resp_adv[20] = {0x02,0x01,0x06,0x05,0xff,0xff,0x00,0x02,0x00,0x07,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x40,0x01};
static uint8_t device_name[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
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
void sl_bt_on_event(sl_bt_msg_t* evt)
{
  sl_status_t sc;

  // Handle stack events
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Print boot message.
      app_log_info("Bluetooth stack booted: v%d.%d.%d-b%d\n",
                   evt->data.evt_system_boot.major,
                   evt->data.evt_system_boot.minor,
                   evt->data.evt_system_boot.patch,
                   evt->data.evt_system_boot.build);
      // create advertising set
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
      // reporting scan request as events.
      //sc = sl_bt_advertiser_set_report_scan_request(advertising_set_handle, 1);
      app_assert_status(sc);
      // Start general advertising and enable connections.
      sl_bt_advertiser_set_configuration(advertising_set_handle,1);
      sc = sl_bt_advertiser_set_data(advertising_set_handle,0,sizeof(scan_resp_adv),&scan_resp_adv[0]);
      app_assert_status(sc);
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_user_data,
        advertiser_connectable_scannable);
      app_assert_status(sc);
      app_log_info("Started Advertising\n");
      break;
    // -------------------------------
    // This event is generated when a new connection is established
    case sl_bt_evt_connection_opened_id:
      sc = sl_bt_advertiser_stop(advertising_set_handle);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event is generated when a connection is dropped
    case sl_bt_evt_connection_closed_id:
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_user_data,
        advertiser_connectable_scannable);
      app_assert_status(sc);
      app_log_info("Started Advertising\n");
      break;

    case sl_bt_evt_gatt_server_user_write_request_id:
         app_log_info("inside user write request\n");
         if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_TX) {
           // Write user supplied value to TX
             app_log_info("TX value sent = %x\n",evt->data.evt_gatt_server_attribute_value.value.data[0]);
             if (evt->data.evt_gatt_server_attribute_value.value.data[0]) {
                 tx_power = evt->data.evt_gatt_server_attribute_value.value.data[0];
                 sc = sl_bt_advertiser_set_tx_power(advertising_set_handle,tx_power,&set_power);
                 set_power = set_power;
                 app_assert_status(sc);
             }
             else
               app_log_info("invalid TX power write.\n");
           sc = sl_bt_gatt_server_send_user_write_response(
               evt->data.evt_gatt_server_user_write_request.connection,
               gattdb_TX, SL_STATUS_OK);
             app_assert_status(sc);
         }

         if(evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_device_name){
             uint8_t len = evt->data.evt_gatt_server_attribute_value.value.len;
             //uint8_t device_name[len];
             for(uint8_t i=0;i<len;i++){
                 scan_resp_adv[i+11] = evt->data.evt_gatt_server_attribute_value.value.data[i];
                 device_name[i] = evt->data.evt_gatt_server_attribute_value.value.data[i];
             }
             sc = sl_bt_advertiser_set_data(advertising_set_handle,0,sizeof(scan_resp_adv),&scan_resp_adv[0]);
             app_assert_status(sc);
           sl_bt_gatt_server_send_user_write_response(
               evt->data.evt_gatt_server_user_write_request.connection,
               gattdb_device_name, SL_STATUS_OK);
           app_log_info("device name = %s",device_name);
         }

         // writing the channel value.
         if(evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_channel){
             scan_resp_adv[19] = evt->data.evt_gatt_server_attribute_value.value.data[0];
             sc = sl_bt_advertiser_set_data(advertising_set_handle,0,sizeof(scan_resp_adv),&scan_resp_adv[0]);
             sc = sl_bt_gatt_server_send_user_write_response(
                 evt->data.evt_gatt_server_user_write_request.connection,
                 gattdb_channel, SL_STATUS_OK);
               app_assert_status(sc);
             app_log_info("scan response channel no: %x",scan_resp_adv[19]);
         }
         break;

    case sl_bt_evt_gatt_server_user_read_request_id:
      app_log_info("received read request.\n");
      if(evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_TX){
          uint16_t sent_len;
          uint8_t reduced_set_power = (uint8_t) set_power;
          sc = sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_write_request.connection,
                                                         gattdb_TX, SL_STATUS_OK, sizeof(reduced_set_power), &reduced_set_power, &sent_len);
          app_assert_status(sc);
          app_log_info("sent read response.\n");
      }
      if(evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_device_name){
          uint16_t sent_len;
          //uint8_t reduced_set_power = (uint8_t) set_power;
          sc = sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_write_request.connection,
                                                         gattdb_device_name, SL_STATUS_OK, sizeof(device_name), device_name, &sent_len);
          app_assert_status(sc);
          app_log_info("sent read response.\n");
      }
      break;

      /*
    case sl_bt_evt_advertiser_scan_request_id:
      //app_log_info("receieved scan request.");
      sc = sl_bt_advertiser_stop(advertising_set_handle);
      sc = sl_bt_advertiser_set_data(advertising_set_handle,1,sizeof(scan_resp_adv),&scan_resp_adv[0]);
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_user_data,
        advertiser_connectable_scannable);   //have to use advertiser_user_data for custom data.
      app_assert_status(sc);
      break;
      */


    default:
      break;
  }
}
