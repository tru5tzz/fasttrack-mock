/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
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
#include <stdio.h>
#include <string.h>

#include "em_common.h"
#include "app_assert.h"
#include "app_log.h"
#include "sl_status.h"
#include "app.h"

#include "sl_btmesh.h"
#include "sl_bluetooth.h"
#include "sl_btmesh_api.h"
#include "sl_bt_api.h"

#include "sl_btmesh_factory_reset.h"

#include "app_button_press.h"
#include "sl_simple_button.h"
#include "sl_simple_button_instances.h"

#include "sl_sleeptimer.h"

#include "DeviceConfiguration.h"
#include "NetworkConfiguration.h"
#include "DeviceManager.h"


#define BLE_MESH_UUID_LEN_BYTE (16)
#define BLE_ADDR_LEN_BYTE (6)

/// Length of the display name buffer
#define NAME_BUF_LEN                   20

/// Increase step of physical values (lightness, color temperature)
#define INCREASE                       10
/// Decrease step of physical values (lightness, color temperature)
#define DECREASE                       (-10)
/// Used button index
#define BUTTON_PRESS_BUTTON_0          0

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  app_log("Board booting up!\n");

  sl_simple_button_enable(&sl_button_btn0);

  sl_sleeptimer_delay_millisecond(1);

  device_manager_init();

  uint8_t dev_num;
  device_manager_get_device_count(&dev_num);
  app_log("Current devices in table %d\n", dev_num);
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

/***************************************************************************//**
 * Handles button press and does a factory reset
 *
 * @return true if there is no button press
 ******************************************************************************/
bool handle_reset_conditions(void)
{
  // If PB0 is held down then do full factory reset
  if (sl_simple_button_get_state(&sl_button_btn0)
      == SL_SIMPLE_BUTTON_PRESSED) {
    app_log("Board full reset\n");
    // Full factory reset
    sl_btmesh_initiate_full_reset();
    return false;
  }
  return true;
}

/***************************************************************************//**
 * Handling of boot event.
 * If needed it performs factory reset. In other case it sets device name
 * and initialize mesh node.
 ******************************************************************************/
static void handle_boot_event(void)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  // Check reset conditions and continue if not reset.
  if (handle_reset_conditions()) {
    sc = sl_bt_system_get_identity_address(&address, &address_type);
    app_assert_status_f(sc, "Failed to get Bluetooth address\n");
    // Initialize Mesh stack in Node operation mode, wait for initialized event
    sc = sl_btmesh_prov_init();
    if (sc != SL_STATUS_OK) {
      app_log("Initialization failed (0x%x)\r\n", sc);
    }
  }
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(struct sl_bt_msg *evt)
{
  switch (SL_BT_MSG_ID(evt->header)) {
    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_bt_evt_system_boot_id:
      handle_boot_event();
      break;
    case sl_bt_evt_connection_opened_id:
      app_log("Connection opened\r\n");
      break;
    case sl_bt_evt_connection_closed_id:
      app_log("Connection closed\r\n");
      break;
    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/**************************************************************************//**
 * Bluetooth Mesh stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth Mesh stack.
 *****************************************************************************/
void sl_btmesh_on_event(sl_btmesh_msg_t *evt)
{
  uint16_t result = 0;
  uint16_t provisionee_addr;
  uuid_128 provisionee_uuid;
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_btmesh_evt_prov_initialized_id: {
        app_log("sl_btmesh_evt_prov_initialized_id\r\n");
        sc = sl_btmesh_prov_create_network(NETWORK_ID, 0, NULL);
        if(sc != SL_STATUS_OK) {
          /* Something went wrong */
            app_log("sl_btmesh_prov_create_network: failed 0x%.2lx\r\n", sc);
            if (sc == SL_STATUS_BT_MESH_ALREADY_EXISTS) {
              app_log("Key already exists\n");
            }
        } else {
            app_log("Success, netkey id = %x\r\n", NETWORK_ID);
        }

        size_t max_application_key_size = 16;
        size_t application_key_len = 16;
        uint8_t application_key[16];
        sc = sl_btmesh_prov_create_appkey(NETWORK_ID, APPKEY_INDEX, 0, NULL, max_application_key_size, &application_key_len, application_key);

        if(sc != SL_STATUS_OK) {
          /* Something went wrong */
            app_log("sl_btmesh_prov_create_appkey: failed 0x%.2lx\r\n", sc);
        } else {
            app_log("Success, appkey id = %x with value of ", APPKEY_INDEX);
            for (uint8_t i = 0; i < application_key_len; i++) {
              app_log("%x", application_key[i]);
              if (i != application_key_len) {
                app_log(":");
              } else {
                app_log("\n");
              }
            }
        }

        /* Networks  */
        app_log("networks: 0x%x ", evt->data.evt_prov_initialized.networks);

        /* address */
        app_log("address: 0x%x ", evt->data.evt_prov_initialized.address);

        /* ivi  */
        app_log("ivi: 0x%lx", evt->data.evt_prov_initialized.iv_index);
        app_log("\r\n");

        sl_btmesh_generic_client_init();

        result = sl_btmesh_prov_scan_unprov_beacons();
        app_button_press_enable();
      }
      break;
    case sl_btmesh_evt_prov_initialization_failed_id:
      app_log("failed: 0x%x ", evt->data.evt_prov_initialization_failed.result);
      break;
    case sl_btmesh_evt_prov_unprov_beacon_id:
      /* PB-ADV only */
      if(0 == evt->data.evt_prov_unprov_beacon.bearer) {
        uuid_128 temp_id = evt->data.evt_prov_unprov_beacon.uuid;
        bd_addr temp_add = evt->data.evt_prov_unprov_beacon.address;
        uint8_t temp_count;
        device_manager_get_device_count(&temp_count);
        app_log("Before adding new device to the table, it has %d device\n", temp_count);
        /* fill up btmesh device struct */
        if( (sl_btmesh_prov_get_ddb_entry(temp_id, NULL, NULL, NULL, NULL) != 0)) {
          /* Device is not present */
          if (device_manager_add_device(&temp_id, &temp_add) == 0) {
            app_log("Found new device\n");
            app_log("Address: %x:%x:%x:%x:%x:%x\n", 
                                      temp_add.addr[5],
                                      temp_add.addr[4],
                                      temp_add.addr[3],
                                      temp_add.addr[2],
                                      temp_add.addr[1],
                                      temp_add.addr[0]);
            app_log("UUID: ");
            for (uint8_t i = 0; i < BLE_MESH_UUID_LEN_BYTE; i++) {
              app_log("%x", temp_id.data[i]);
            }
            app_log("\n");
            device_manager_get_device_count(&temp_count);
            app_log("After adding 1 dev, now it has %d devices\n", temp_count);
          }
        }
      }
      break;
    /* Provisioning */
    case sl_btmesh_evt_prov_provisioning_failed_id:
      app_log("provisioning failed\r\n");
      break;
    case sl_btmesh_evt_prov_device_provisioned_id:
      provisionee_addr = evt->data.evt_prov_device_provisioned.address;
      provisionee_uuid = evt->data.evt_prov_device_provisioned.uuid;
      app_log("Node successfully provisioned. Address: %4.4x, ", provisionee_addr);

      app_log("uuid 0x");
      for (uint8_t i = 0; i < BLE_MESH_UUID_LEN_BYTE; i++) app_log("%02X", provisionee_uuid.data[i]);
      app_log("\r\n");

      app_log(" sending app key to node ...\r\n");

      sc = sl_btmesh_config_client_add_appkey(NETWORK_ID, provisionee_addr, APPKEY_INDEX, NETWORK_ID, NULL);
      if(sc != SL_STATUS_OK) {
        app_log("sl_btmesh_config_client_add_appkey failed with result 0x%lX (%ld) addr %x\r\n", sc, sc, provisionee_addr);
      }
      break;

    /* Config events */
    case sl_btmesh_evt_config_client_appkey_status_id:
      result = evt->data.evt_config_client_appkey_status.result;
      if(result == SL_STATUS_OK) {
        app_log(" appkey added\r\n");
      }

      break;
    // -------------------------------
    // Default event handler.
    default:
      app_log("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", (unsigned int)SL_BT_MSG_ID(evt->header),
                                                                  (unsigned int)((SL_BT_MSG_ID(evt->header) >> 16) & 0xFF),
                                                                  (unsigned int)((SL_BT_MSG_ID(evt->header) >> 24) & 0xFF) );
      break;
  }
}

void provisionBLEMeshStack_app(eMesh_Prov_Node_t eStrategy)
{
  sl_status_t sc;
  uint8_t current_devices_in_table;
  uuid_128 temp_id;
  bd_addr temp_add;

  device_manager_get_device_count(&current_devices_in_table);

  if(    ( eMESH_PROV_ALL == eStrategy )
      || ( eMESH_PROV_NEXT == eStrategy )
    ) {
    for(uint8_t i = 0; i < current_devices_in_table; i++) {
      if (device_manager_get_next_device(&temp_id, &temp_add) == 0) {
        /* provisioning using ADV bearer (this is the default) */
        sl_btmesh_prov_create_provisioning_session(NETWORK_ID, temp_id, 0);
        sc = sl_btmesh_prov_provision_adv_device(temp_id);
        if (sc == SL_STATUS_OK) {
          app_log("Provisioning request sent\n");
          device_manager_remove_device(&temp_add);
        } else {
          app_log("Provisioning fail %lX: ",sc);
        }
      }
    }
  }
}

sl_sleeptimer_timer_handle_t double_tap_timer;

/* This callback funtion will be used to handle the single button push event
 * While the double push button will be handled by the app_button_press_cb below
 */
void double_tap_timer_on_timeout(sl_sleeptimer_timer_handle_t *timer, void *data) {
  printf("Single push detected\n");
}

void app_button_press_cb(uint8_t button, uint8_t duration) {
  sl_status_t retval;
  bool sleeptimer_running = false;
  app_log("Button %d pressed\n", button);
  switch (duration) {
    case APP_BUTTON_PRESS_DURATION_SHORT:
      if (button == 1) {
        retval = sl_btmesh_prov_list_ddb_entries(NULL);
        app_assert_status_f(retval, "Query node database failed\n");
      }
      break;
    case APP_BUTTON_PRESS_DURATION_VERYLONG:
      sl_sleeptimer_is_timer_running(&double_tap_timer, &sleeptimer_running);
      if (sleeptimer_running) {
        sl_sleeptimer_stop_timer(&double_tap_timer);
        printf("Double tap detected\n");
      } else {
        sl_sleeptimer_start_timer_ms(&double_tap_timer, 200, double_tap_timer_on_timeout, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
      }
      break;
    case APP_BUTTON_PRESS_DURATION_LONG:
      app_log("list unprovisioned devices\n");
      provisionBLEMeshStack_app(eMESH_PROV_NEXT);
      break;
    default:
      break;
  }
}
