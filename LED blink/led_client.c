/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
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
#include "app_log_config.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "sl_simple_button_instances.h"
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#ifdef SL_CATALOG_CLI_PRESENT
#include "sl_cli.h"
#endif


#define CONN_INTERVAL_MIN             80   //100ms
#define CONN_INTERVAL_MAX             80   //100ms
#define CONN_SLAVE_LATENCY            0    //no latency
#define CONN_TIMEOUT                  100  //1000ms
#define CONN_MIN_CE_LENGTH            0
#define CONN_MAX_CE_LENGTH            0xffff

#define SCAN_INTERVAL                 16   //10ms
#define SCAN_WINDOW                   16   //10ms
#define SCAN_PASSIVE                  0

static bool report_button_flag = false;

// Sends notification of the Report Button characteristic.
static const uint8_t led_service[16] = {  0x7b, 0x7c, 0x37, 0x7c, 0x6a, 0xc9, 0x47, 0x95, 0xe5, 0x4d, 0xab, 0xc2, 0x84, 0x5e, 0x76, 0x9c, };
// Custom LED characteristics is defined by following
static const uint8_t led_char[16] = {  0x2e, 0x80, 0x9c, 0x1f, 0xf7, 0x74, 0x60, 0x87, 0x64, 0x41, 0xef, 0xdb, 0x30, 0xe4, 0xc6, 0x58, };

typedef struct {
  uint8_t  connection_handle;
  uint32_t led_service_handle;
  uint16_t led_characteristic_handle;
} conn_properties_t;

typedef enum {
  scanning,
  opening,
  discover_services,
  discover_characteristics,
  enable_indication,
  running
} conn_state_t;

static conn_state_t conn_state;
static conn_properties_t connection;
// Updates the Report Button characteristic.
static sl_status_t update_report_button_characteristic(void);
static uint8_t find_service_in_advertisement(uint8_t *data, uint8_t len);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  // Make sure there will be no button events before the boot event.
  sl_button_disable(SL_SIMPLE_BUTTON_INSTANCE(0));

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)// was void
{
  // Check if there was a report button interaction.
  if (report_button_flag) {
    sl_status_t sc;

    report_button_flag = false; // Reset flag

    sc = update_report_button_characteristic();
    app_log_status_error(sc);
    app_log_info("updated button state");
  }


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
      app_log_info("getting identity address.\n");
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

      // Create an advertising set.
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
      // Start scanning - looking for thermometer devices
      app_log_info("setting scanner active.\n");
      sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
      app_assert_status_f(sc,
                          "Failed to start discovery #1\n");

      // Button events can be received from now on.
      sl_button_enable(SL_SIMPLE_BUTTON_INSTANCE(0));
      break;
      
    case sl_bt_evt_scanner_scan_report_id:
          // Parse advertisement packets
      if (evt->data.evt_scanner_scan_report.packet_type == 0) {
             // If a led advertisement is found...
             if (find_service_in_advertisement(&(evt->data.evt_scanner_scan_report.data.data[0]),
                                               evt->data.evt_scanner_scan_report.data.len) != 0) {
               // then stop scanning for a while
               sc = sl_bt_scanner_stop();
               app_assert_status(sc);
               // and connect to that device
                 sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                            evt->data.evt_scanner_scan_report.address_type,
                                            gap_1m_phy,
                                            NULL);
                 app_assert_status(sc);
                 app_log_info("Opening connection\n");
               }
             }
       break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      connection.connection_handle = evt->data.evt_connection_opened.connection;
      app_log_info("Connection opened.\n");
      sc = sl_bt_gatt_discover_primary_services_by_uuid(evt->data.evt_connection_opened.connection,
                                                        sizeof(led_service),
                                                        (const uint8_t*)led_service);
      app_assert_status(sc);
      conn_state = discover_services;
      app_log_info("Service discovered\n");
      break;

    case sl_bt_evt_gatt_service_id:
      if (evt->data.evt_gatt_service.connection == connection.connection_handle)
        connection.led_service_handle = evt->data.evt_gatt_service.service;
      break;

    case sl_bt_evt_gatt_characteristic_id:
      if(evt->data.evt_gatt_characteristic.connection == connection.connection_handle){
          connection.led_characteristic_handle = evt->data.evt_gatt_characteristic.characteristic;
          app_log_info("led characteristics handle written. value : %d",connection.led_characteristic_handle);
      }
      break;
    // -------------------------------
    // This event indicates that a connection was closed.

    case sl_bt_evt_gatt_procedure_completed_id:
      if (conn_state == discover_services) {
              // Discover thermometer characteristic on the slave device
              sc = sl_bt_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
                                                               connection.led_service_handle,
                                                               sizeof(led_char),
                                                               (const uint8_t*)led_char);
              app_assert_status(sc);
              conn_state = discover_characteristics;
              app_log_info("characteristics discovered\n");
              break;
            }
            // If characteristic discovery finished
            if (conn_state == discover_characteristics) {
              // stop discovering
              sl_bt_scanner_stop();
              app_log_info("scanner stopped\n");
              conn_state = running;
              break;
            }
          break;

    case sl_bt_evt_connection_closed_id:
      app_log_info("Connection closed.\n");

      // Restart sacnning after client has disconnected.
      sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that the value of an attribute in the local GATT
    // database was changed by a remote GATT client.

    // -------------------------------
    // This event occurs when the remote device enabled or disabled the
    // notification.

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/***************************************************************************//**
 * Simple Button
 * Button state changed callback
 * @param[in] handle Button event handle
 ******************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  if (SL_SIMPLE_BUTTON_INSTANCE(0) == handle) {
    report_button_flag = true;
  }
}

/***************************************************************************//**
 * Updates the Report Button characteristic.
 *
 * Checks the current button state and then writes it into the local GATT table.
 ******************************************************************************/
static sl_status_t update_report_button_characteristic(void)
{
  sl_status_t sc;
  uint8_t data_send;

  switch (sl_button_get_state(SL_SIMPLE_BUTTON_INSTANCE(0))) {
    case SL_SIMPLE_BUTTON_PRESSED:
      data_send = (uint8_t)SL_SIMPLE_BUTTON_PRESSED;
      break;

    case SL_SIMPLE_BUTTON_RELEASED:
      data_send = (uint8_t)SL_SIMPLE_BUTTON_RELEASED;
      break;

    default:
      // Invalid button state
      return SL_STATUS_FAIL; // Invalid button state
  }

  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_write_characteristic_value(connection.connection_handle,
                                             connection.led_characteristic_handle,
                                               sizeof(data_send),
                                               &data_send);
  if (sc == SL_STATUS_OK) {
    app_log_info("Attribute written: 0x%02x", (int)data_send);
  }

  return sc;
}
// Parse advertisements looking for advertised LED service
static uint8_t find_service_in_advertisement(uint8_t *data, uint8_t len)
{
  uint8_t ad_field_length;
  uint8_t ad_field_type;
  uint8_t i = 0;
  // Parse advertisement packet
  while (i < len) {
    ad_field_length = data[i];
    ad_field_type = data[i + 1];
    if (ad_field_type == 0x02 || ad_field_type == 0x07) {
      if (memcmp(&data[i + 2], led_service, 16) == 0) {
        return 1;
      }
    }
    // advance to the next AD struct
    i = i + ad_field_length + 1;
  }
  return 0;
}
