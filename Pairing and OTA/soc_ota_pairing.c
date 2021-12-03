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
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#ifdef SL_CATALOG_CLI_PRESENT
#include "sl_cli.h"
#endif // SL_CATALOG_CLI_PRESENT

#include "btl_interface.h"
#include "btl_interface_storage.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static uint8_t data;
static uint8_t connection;
/***************************************************************************//**
 *
 * Configure security requirements and I/O capabilities of the system.
 * It is named as MITM_PROTECTION in this application.
 *
 * @param[in] flags @parblock
 *   Security requirement bitmask.
 *
 *   Bit 0:
 *     - <b>0:</b> Allow bonding without authentication
 *     - <b>1:</b> Bonding requires authentication (Man-in-the-Middle
 *       protection)
 *
 *   Bit 1:
 *     - <b>0:</b> Allow encryption without bonding
 *     - <b>1:</b> Encryption requires bonding. Note that this setting will also
 *       enable bonding.
 *
 *   Bit 2:
 *     - <b>0:</b> Allow bonding with legacy pairing
 *     - <b>1:</b> Secure connections only
 *
 *   Bit 3:
 *     - <b>0:</b> Bonding request does not need to be confirmed
 *     - <b>1:</b> Bonding requests need to be confirmed. Received bonding
 *       requests are notified by @ref sl_bt_evt_sm_confirm_bonding
 *
 *   Bit 4:
 *     - <b>0:</b> Allow all connections
 *     - <b>1:</b> Allow connections only from bonded devices
 *
 *   Bit 5:
 *     - <b>0:</b> Prefer just works pairing when both options are possible
 *       based on the settings.
 *     - <b>1:</b> Prefer authenticated pairing when both options are possible
 *       based on the settings.
 *
 *   Bit 6 to 7: Reserved
 *
 *   Default value: 0x00
 *   @endparblock
 *   ******************************************************************************/
static uint8_t MITM_PROTECTION = 0x2a;
static uint8_t IO_CAPABILITY = sl_bt_sm_io_capability_noinputnooutput;

/* Flag for indicating DFU Reset must be performed */


//Bootloader Interface
static BootloaderInformation_t bldInfo;
static BootloaderStorageSlot_t slotInfo;

/* OTA variables */
static uint32_t ota_image_position = 0;
static uint8_t ota_in_progress = 0;
static uint8_t ota_image_finished = 0;
static uint16_t ota_time_elapsed = 0;


static int32_t get_slot_info()
{
    int32_t err;

    bootloader_getInfo(&bldInfo);
    app_log_info("Gecko bootloader version: %u.%u\r\n", (bldInfo.version & 0xFF000000) >> 24, (bldInfo.version & 0x00FF0000) >> 16);

    err = bootloader_getStorageSlotInfo(0, &slotInfo);

    if(err == BOOTLOADER_OK)
    {
        app_log_info("Slot 0 starts @ 0x%8.8x, size %u bytes\r\n", slotInfo.address, slotInfo.length);
    }
    else
    {
        app_log_info("Unable to get storage slot info, error %x\r\n", err);
    }

    return(err);
}

static int32_t verify_application()
{
  int32_t err;

  err = bootloader_verifyImage(0, NULL);

  if(err != BOOTLOADER_OK)
  {
      app_log_info("Application Verification Failed. err: %u \r\n", err);
  }
  else
  {
      app_log_info("Application Verified \r\n");
  }

  return err;

}

static void erase_slot_if_needed()
{
    uint32_t offset = 0, num_blocks = 0, i = 0;
    uint8_t buffer[256];
    bool dirty = false;
    int32_t err = BOOTLOADER_OK;

    /* check the download area content by reading it in 256-byte blocks */
    num_blocks = slotInfo.length / 256;

    while((dirty == 0) && (offset < 256*num_blocks) && (err == BOOTLOADER_OK))
    {
        err = bootloader_readStorage(0, offset, buffer, 256);
        if(err == BOOTLOADER_OK)
        {
            i = 0;
            while(i < 256)
            {
                if(buffer[i++] != 0xFF)
                {
                    dirty = true;
                    break;
                }
            }
            offset += 256;
        }
        app_log_info(".");
    }

    if(err != BOOTLOADER_OK)
    {
        app_log_info("error reading flash! %x\r\n", err);
    }
    else if(dirty)
    {
        app_log_info("download area is not empty, erasing...\r\n");
        bootloader_eraseStorageSlot(0);
        app_log_info("done\r\n");
    }
    else
    {
        app_log_info("download area is empty\r\n");
    }

    return;
}

static void print_progress()
{
    // estimate transfer speed in kbps
    int kbps = ota_image_position*8/(1024*ota_time_elapsed);

    app_log_info("pos: %u, time: %u, kbps: %u\r\n", ota_image_position, ota_time_elapsed, kbps);
}

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

      //Soft timer started for progress report
      sl_bt_system_set_soft_timer(32768,  0, 0);

      //need to confirm the hierarchy. deleted bonding and then configured bonding
      //clear previous bonding;
      //sc = sl_bt_sm_delete_bondings();
      //app_assert_status(sc);

      // Configuration according to constants set at compile time
      sc = sl_bt_sm_configure(MITM_PROTECTION, IO_CAPABILITY);
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
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);

      // bootloader init must be called before calling other bootloader_xxx API calls
      bootloader_init();

      // read slot information from bootloader
      if(get_slot_info() == BOOTLOADER_OK){

        // the download area is erased here (if needed), prior to any connections are opened
        erase_slot_if_needed();
      } else {
          app_log_info("Check that you have installed correct type of Gecko bootloader!\r\n");
      }
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      connection = evt->data.evt_connection_opened.connection;
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log_info("Connection closed, reason: 0x%2.2x\r\n", evt->data.evt_connection_closed.reason);

      if(ota_image_finished){
          app_log_info("Verifying new image\r\n");
          int32_t verified = verify_application();
          if(verified == 0){
              app_log_info("Installing new image\r\n");
              bootloader_setImageToBootload(0);
              bootloader_rebootAndInstall();
          }
          else{
              app_log_info("Application not installed\r\n");
              erase_slot_if_needed();
              ota_time_elapsed = 0;
              // Restart advertising after client has disconnected.
                  sc = sl_bt_advertiser_start(
                    advertising_set_handle,            // advertising set handle
                    advertiser_general_discoverable,   // discoverable mode
                    advertiser_connectable_scannable); // connectable mode
                  app_assert_status(sc);
                  app_log_info("Started advertising\n");
          }

      } else{
        // Restart advertising after client has disconnected.
            sc = sl_bt_advertiser_start(
              advertising_set_handle,            // advertising set handle
              advertiser_general_discoverable,   // discoverable mode
              advertiser_connectable_scannable); // connectable mode
            app_assert_status(sc);
            app_log_info("Started advertising\n");
      }
      break;

    case  sl_bt_evt_system_soft_timer_id:
      if(ota_in_progress)
         {
            ota_time_elapsed++;
            print_progress();
           }
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_bt_evt_sm_confirm_bonding_id:
      // 0 to deny, 1 to allow bonding
      sc = sl_bt_sm_bonding_confirm(connection, 1);
      app_assert_status(sc);
      break;

    case sl_bt_evt_gatt_server_user_write_request_id:
             //app_log_info("inside user write request\n");
          {
             uint32_t connection = evt->data.evt_gatt_server_user_write_request.connection;
             uint32_t characteristic = evt->data.evt_gatt_server_user_write_request.characteristic;
             if (characteristic == gattdb_Value) {
               // Write user supplied value
                 if (evt->data.evt_gatt_server_attribute_value.value.data[0]) {
                     data = evt->data.evt_gatt_server_attribute_value.value.data[0];
                     //app_log_info("data = %x",data);
                 }
               sc = sl_bt_gatt_server_send_user_write_response(
                   evt->data.evt_gatt_server_user_write_request.connection,
                   gattdb_Value, SL_STATUS_OK);
                 app_assert_status(sc);
             }
             else if(characteristic == gattdb_ota_control)
             {
                 switch(evt->data.evt_gatt_server_user_write_request.value.data[0])
                 {
                 case 0://Erase and use slot 0
                     // NOTE: download area is NOT erased here, because the long blocking delay would result in supervision timeout
                     //bootloader_eraseStorageSlot(0);
                     ota_image_position=0;
                     ota_in_progress=1;
                     break;
                 case 3://END OTA process
                     //wait for connection close and then reboot
                     ota_in_progress=0;
                     ota_image_finished=1;
                     app_log_info("upload finished. received file size %u bytes\r\n", ota_image_position);

                     break;
                 default:
                     break;
                 }
             } else if(characteristic == gattdb_ota_data)
             {
                 if(ota_in_progress)
                 {
                     bootloader_writeStorage(0,//use slot 0
                             ota_image_position,
                             evt->data.evt_gatt_server_user_write_request.value.data,
                             evt->data.evt_gatt_server_user_write_request.value.len);
                     ota_image_position+=evt->data.evt_gatt_server_user_write_request.value.len;
                 }
             }
             sl_bt_gatt_server_send_user_write_response(connection,characteristic,0);
          }

        break;

    case sl_bt_evt_gatt_server_user_read_request_id:
      //app_log_info("received read request.\n");
      if(evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_Value){
          uint16_t sent_len;
          sc = sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_write_request.connection,
                                                         gattdb_Value, SL_STATUS_OK, sizeof(data), &data, &sent_len);
          app_assert_status(sc);
          //app_log_info("sent read response.\n");
      }
        break;


    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

#ifdef SL_CATALOG_CLI_PRESENT
void hello(sl_cli_command_arg_t *arguments)
{
  (void) arguments;
  bd_addr address;
  uint8_t address_type;
  sl_status_t sc = sl_bt_system_get_identity_address(&address, &address_type);
  sl_app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] Failed to get Bluetooth address\n",
             (int)sc);
  sl_app_log("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
          address_type ? "static random" : "public device",
          address.addr[5],
          address.addr[4],
          address.addr[3],
          address.addr[2],
          address.addr[1],
          address.addr[0]);
}
#endif // SL_CATALOG_CLI_PRESENT
