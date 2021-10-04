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
#include <stdbool.h>
#include <math.h>
#include "em_common.h"
#include "app_log.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "sl_simple_button_instances.h"
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#ifdef SL_CATALOG_CLI_PRESENT
#include "sl_cli.h"
#endif // SL_CATALOG_CLI_PRESENT
#include "app.h"

// connection parameters
#define CONN_INTERVAL_MIN             80   //100ms
#define CONN_INTERVAL_MAX             80   //100ms
#define CONN_SLAVE_LATENCY            0    //no latency
#define CONN_TIMEOUT                  100  //1000ms
#define CONN_MIN_CE_LENGTH            0
#define CONN_MAX_CE_LENGTH            0xffff

#define SCAN_INTERVAL                 16   //10ms
#define SCAN_WINDOW                   16   //10ms
#define SCAN_PASSIVE                  0

#define CONNECTION_HANDLE_INVALID     ((uint8_t)0xFFu)
#define SERVICE_HANDLE_INVALID        ((uint32_t)0xFFFFFFFFu)
#define CHARACTERISTIC_HANDLE_INVALID ((uint16_t)0xFFFFu)
#define TABLE_INDEX_INVALID           ((uint8_t)0xFFu)

#if SL_BT_CONFIG_MAX_CONNECTIONS < 1
  #error At least 1 connection has to be enabled!
#endif


typedef enum {
  scanning,
  opening,
  discover_services,
  discover_characteristics,
  enable_indication,
  running
} conn_state_t;

// may be all are not needed
typedef struct {
  uint8_t  connection_handle;
  uint16_t server_address;
  uint32_t led_service_handle;
  uint16_t led_characteristic_handle;
  uint8_t  state;
} conn_properties_t;


// Array for holding properties of multiple (parallel) connections
static conn_properties_t conn_properties[SL_BT_CONFIG_MAX_CONNECTIONS];
// Counter of active connections
static uint8_t active_connections_num;
// State of the connection under establishment
static conn_state_t conn_state;
// Custom LED service UUID defined by following.
static const uint8_t led_service[16] = { 0x3c,0xbb,0x77,0xa5,0xc0,0x20,0xba,0x9c,0xb1,0x42,0x84,0xb7,0xca,0x40,0xa3,0xf4 };
// Custom LED characteristics is defined by following
static const uint8_t led_char[16] = { 0xfc, 0xba, 0x18, 0x7c, 0x21, 0x82, 0x90, 0x84, 0xba, 0x4f, 0x35, 0x47, 0x1d, 0xdd, 0xb4, 0xf9 };

static void init_properties(void);
static uint8_t find_service_in_advertisement(uint8_t *data, uint8_t len);
static uint8_t find_index_by_connection_handle(uint8_t connection);
static void add_connection(uint8_t connection, uint16_t address);
// Remove a connection from the connection_properties array
static void remove_connection(uint8_t connection);
static bd_addr *read_and_cache_bluetooth_address(uint8_t *address_type_out);
static void print_bluetooth_address(void);
void sl_button_on_change(const sl_button_t *handle);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  // Initialize connection properties
  init_properties();
  app_log_info("soc_led_client initialised\n");
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
  uint16_t addr_value;
  uint8_t table_index;

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
      // Print bluetooth address.
      print_bluetooth_address();
      // Set passive scanning on 1Mb PHY
      sc = sl_bt_scanner_set_mode(gap_1m_phy, SCAN_PASSIVE);
      app_assert_status(sc);
      // Set scan interval and scan window
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
      sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
      app_assert_status_f(sc,
                          "Failed to start discovery #1\n");
      conn_state = scanning;
      app_log_info("Scanning\n");
      break;

    // -------------------------------
    // This event is generated when an advertisement packet or a scan response
    // is received from a slave
    case sl_bt_evt_scanner_scan_report_id:
      // Parse advertisement packets
      if (evt->data.evt_scanner_scan_report.packet_type == 0) {
        // If a thermometer advertisement is found...
        if (find_service_in_advertisement(&(evt->data.evt_scanner_scan_report.data.data[0]),
                                          evt->data.evt_scanner_scan_report.data.len) != 0) {
          // then stop scanning for a while
          sc = sl_bt_scanner_stop();
          app_assert_status(sc);
          // and connect to that device
          if (active_connections_num < SL_BT_CONFIG_MAX_CONNECTIONS) {
            sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                       evt->data.evt_scanner_scan_report.address_type,
                                       gap_1m_phy,
                                       NULL);
            app_assert_status(sc);
            conn_state = opening;
            app_log_info("Opening connection\n");
          }
        }
      }
      break;

    // -------------------------------
    // This event is generated when a new connection is established
    case sl_bt_evt_connection_opened_id:
      // Get last two bytes of sender address
      addr_value = (uint16_t)(evt->data.evt_connection_opened.address.addr[1] << 8) + evt->data.evt_connection_opened.address.addr[0];
      // Add connection to the connection_properties array
      add_connection(evt->data.evt_connection_opened.connection, addr_value);
      // Discover LED service on the slave device
      sc = sl_bt_gatt_discover_primary_services_by_uuid(evt->data.evt_connection_opened.connection,
                                                        sizeof(led_service),
                                                        (const uint8_t*)led_service);
      app_assert_status(sc);
      conn_state = discover_services;
      app_log_info("Service discovered\n");
      break;

    // -------------------------------
    // This event is generated when a new service is discovered
    case sl_bt_evt_gatt_service_id:
      table_index = find_index_by_connection_handle(evt->data.evt_gatt_service.connection);
      if (table_index != TABLE_INDEX_INVALID) {
        // Save service handle for future reference
        conn_properties[table_index].led_service_handle = evt->data.evt_gatt_service.service;
      }
      break;

    // -------------------------------
    // This event is generated when a new characteristic is discovered
    case sl_bt_evt_gatt_characteristic_id:
      table_index = find_index_by_connection_handle(evt->data.evt_gatt_characteristic.connection);
      if (table_index != TABLE_INDEX_INVALID) {
        // Save characteristic handle for future reference
        conn_properties[table_index].led_characteristic_handle = evt->data.evt_gatt_characteristic.characteristic;
      }
      break;

    // -------------------------------
    // This event is generated for various procedure completions, e.g. when a
    // write procedure is completed, or service discovery is completed
    case sl_bt_evt_gatt_procedure_completed_id:
      table_index = find_index_by_connection_handle(evt->data.evt_gatt_procedure_completed.connection);
      if (table_index == TABLE_INDEX_INVALID) {
        break;
      }
      // If service discovery finished
      if (conn_state == discover_services && conn_properties[table_index].led_service_handle != SERVICE_HANDLE_INVALID) {
        // Discover led characteristic on the slave device
        sc = sl_bt_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
                                                         conn_properties[table_index].led_service_handle,
                                                         sizeof(led_char),
                                                         (const uint8_t*)led_char);
        app_assert_status(sc);
        conn_state = discover_characteristics;
        app_log_info("characteristics discovered\n");
        break;
      }
      // If characteristic discovery finished
      if (conn_state == discover_characteristics && conn_properties[table_index].led_characteristic_handle != CHARACTERISTIC_HANDLE_INVALID) {
        // stop discovering
        sl_bt_scanner_stop();
        app_log_info("scanner stopped\n");
        app_log_info("active connection number: %d", active_connections_num);
        conn_state = running;
        break;
      }
      // If indication enable process finished
      if (conn_state == enable_indication) {
        // and we can connect to more devices
        if (active_connections_num < SL_BT_CONFIG_MAX_CONNECTIONS) {
          // start scanning again to find new devices
          sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
          app_assert_status_f(sc,
                              "Failed to start discovery #2\n");
          conn_state = scanning;
        } else {
          conn_state = running;
        }
        break;
      }
      break;

    // -------------------------------
    // This event is generated when a connection is dropped
    case sl_bt_evt_connection_closed_id:
      // remove connection from active connections
      remove_connection(evt->data.evt_connection_closed.connection);
      app_log_info("connection removed\n");
      app_log_info("active connection number: %d", active_connections_num);
      if (conn_state != scanning) {
        // start scanning again to find new devices
        sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
        app_assert_status_f(sc,
                            "Failed to start discovery #3\n");
        conn_state = scanning;
      }
      break;
    // -------------------------------
      
    case sl_bt_evt_system_external_signal_id:
      //Process external signals
          if (evt->data.evt_system_external_signal.extsignals == 1)  // 1 = BTN0
          {
              app_log_info("got external signal\n");
              // send number of button presses
              uint8_t i = 0;
              for(i = 0;i<active_connections_num;i++){
                  if(conn_properties[i].state == 0x01)
                    conn_properties[i].state = 0x00;
                  else
                    conn_properties[i].state = 0x01;
              sc = sl_bt_gatt_write_characteristic_value(
                      conn_properties[i].connection_handle, conn_properties[i].led_characteristic_handle, sizeof(conn_properties[i].state),
                      &conn_properties[i].state);
              app_assert_status(sc);
              }
          }
      break;

    default:
      break;
  }
}

// Init connection properties
static void init_properties(void)
{
  uint8_t i;
  active_connections_num = 0;

  for (i = 0; i < SL_BT_CONFIG_MAX_CONNECTIONS; i++) {
    conn_properties[i].connection_handle = CONNECTION_HANDLE_INVALID;
    conn_properties[i].led_service_handle = SERVICE_HANDLE_INVALID;
    conn_properties[i].led_characteristic_handle = CHARACTERISTIC_HANDLE_INVALID;
    conn_properties[i].state = (uint8_t) 0x00;
  }
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

// Find the index of a given connection in the connection_properties array
static uint8_t find_index_by_connection_handle(uint8_t connection)
{
  for (uint8_t i = 0; i < active_connections_num; i++) {
    if (conn_properties[i].connection_handle == connection) {
      return i;
    }
  }
  return TABLE_INDEX_INVALID;
}

// Add a new connection to the connection_properties array
static void add_connection(uint8_t connection, uint16_t address)
{
  conn_properties[active_connections_num].connection_handle = connection;
  conn_properties[active_connections_num].server_address    = address;
  active_connections_num++;
}

// Remove a connection from the connection_properties array
static void remove_connection(uint8_t connection)
{
  uint8_t i;
  uint8_t table_index = find_index_by_connection_handle(connection);

  if (active_connections_num > 0) {
    active_connections_num--;
  }
  // Shift entries after the removed connection toward 0 index
  for (i = table_index; i < active_connections_num; i++) {
    conn_properties[i] = conn_properties[i + 1];
  }
  // Clear the slots we've just removed so no junk values appear
  for (i = active_connections_num; i < SL_BT_CONFIG_MAX_CONNECTIONS; i++) {
    conn_properties[i].connection_handle = CONNECTION_HANDLE_INVALID;
    conn_properties[i].led_service_handle = SERVICE_HANDLE_INVALID;
    conn_properties[i].led_characteristic_handle = CHARACTERISTIC_HANDLE_INVALID;
    conn_properties[i].state = TEMP_INVALID;
  }
}

/**************************************************************************//**
 * @brief
 *   Function to Read and Cache Bluetooth Address.
 * @param address_type_out [out]
 *   A pointer to the outgoing address_type. This pointer can be NULL.
 * @return
 *   Pointer to the cached Bluetooth Address
 *****************************************************************************/
static bd_addr *read_and_cache_bluetooth_address(uint8_t *address_type_out)
{
  static bd_addr address;
  static uint8_t address_type;
  static bool cached = false;

  if (!cached) {
    sl_status_t sc = sl_bt_system_get_identity_address(&address, &address_type);
    app_assert_status(sc);
    cached = true;
  }

  if (address_type_out) {
    *address_type_out = address_type;
  }

  return &address;
}

/**************************************************************************//**
 * @brief
 *   Function to Print Bluetooth Address.
 * @return
 *   None
 *****************************************************************************/
static void print_bluetooth_address(void)
{
  uint8_t address_type;
  bd_addr *address = read_and_cache_bluetooth_address(&address_type);

  app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
               address_type ? "static random" : "public device",
               address->addr[5],
               address->addr[4],
               address->addr[3],
               address->addr[2],
               address->addr[1],
               address->addr[0]);
}

// Print parameters to STDOUT. CR used to display results.
void sl_button_on_change(const sl_button_t *handle)
{
  sl_simple_button_context_t *ctxt = ((sl_simple_button_context_t*)handle[0].context);
  if(ctxt->state){
      sl_bt_external_signal(1);
  }
}
