/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOSConfig.h"
/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "blehr_sens.h"

static const char *tag = "ESPelotool";

// static xTimerHandle notify_power_timer;

static bool notify_state;

static uint16_t conn_handle;

static const char *device_name = "ESPelotool";

static int blehr_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t blehr_addr_type;


/**
 * Utility function to log an array of bytes.
 */
void
print_bytes(const uint8_t *bytes, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void
print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = addr;
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}


/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void
blehr_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    fields.num_uuids16 = 2;
    
    ble_uuid16_t service_uuids[2] = {BLE_UUID16_INIT(GATT_DEVICE_INFO_UUID), BLE_UUID16_INIT(GATT_CPS_UUID)};

    fields.uuids16 = service_uuids;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(blehr_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, blehr_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

// static void
// notify_power_stop(void)
// {
//     xTimerStop( notify_power_timer, 1000 / portTICK_PERIOD_MS );
// }

/* Reset heart rate measurement */
// static void
// notify_power_reset(void)
// {
//     int rc;

//     if (xTimerReset(notify_power_timer, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
//         rc = 0;
//     } else {
//         rc = 1;
//     }

//     assert(rc == 0);

// }

/* This function simulates power and notifies it to the client */
static void
notify_power(int power_watts /* xTimerHandle ev */)
{
    ESP_LOGI(tag, "notifying power");
    static uint16_t payload[2];
    int rc;
    struct os_mbuf *om;

    if (!notify_state) {
        // notify_power_stop();
        return;
    }

    payload[0] = 0x0000; /* No additional fields present */
    payload[1] = power_watts; /* Power data */

    /* Simulation of power */
    // if (power_watts != 100) {
    //     ESP_LOGI(tag, "setting power to 100");
    //     power_watts = 100;
    // }
    // else {
    //     ESP_LOGI(tag, "setting power to 200");
    //     power_watts = 200;
    // }

    om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
    rc = ble_gattc_notify_custom(conn_handle, hrs_hrm_handle, om);

    assert(rc == 0);

    // notify_power_reset();
}

static int
blehr_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            blehr_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "adv complete\n");
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                    "val_handle=%d\n",
                    event->subscribe.cur_notify, hrs_hrm_handle);
        if (event->subscribe.attr_handle == hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            // notify_power_reset();
        } else if (event->subscribe.attr_handle != hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            // notify_power_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
        break;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;

    }

    return 0;
}

static void
blehr_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &blehr_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(blehr_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");

    /* Begin advertising */
    blehr_advertise();
}

static void
blehr_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void blehr_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void uart_stuff() {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;


    ESP_ERROR_CHECK(uart_driver_install(2 /* port */, 1024 * 2, 256, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(2 /* port */, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(2 /* port */, 14, 13, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(1024);

    uint8_t init[4] = {0xFE, 0x00, 0xFE, 0xF6};
    uint8_t get_power[4] = {0xF5, 0x44, 0x39, 0xF6};

    while (1) {
        int wlen = uart_write_bytes(2, get_power, 4);

        // Read data from the UART
        int len = uart_read_bytes(2, data, 1024, 20 / portTICK_RATE_MS);
        if (len > 0) {
            uint8_t dir = data[0];
            uint8_t metric = data[1];
            uint8_t payload_len = data[2];

            // ESP_LOGI("", "dir: %x, metric: %x, len: %x\n", dir, metric, payload_len);

            if (dir == 0xF1 && metric == 0x44) {
                assert(payload_len == 5);
                assert(len == 10);

                int power_deciwatts = 0;
                int power_10 = 1;
                for (int place = 0; place < 5; place++) {

                    power_deciwatts += power_10 * (data[3 + place] - (int) '0');

                    power_10 *= 10;
                }

                ESP_LOGI("", "power: %d deciwatts\n", power_deciwatts);
                notify_power(power_deciwatts / 10);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // Write data back to the UART
        // ESP_LOGI(tag, "uart: %d\n", len);
    }
}

void app_main(void)
{
    int rc;

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();
    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = blehr_on_sync;
    ble_hs_cfg.reset_cb = blehr_on_reset;

    /* name, period/time,  auto reload, timer ID, callback */
    // notify_power_timer = xTimerCreate("notify_power_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, notify_power);

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(blehr_host_task);

    uart_stuff();
}