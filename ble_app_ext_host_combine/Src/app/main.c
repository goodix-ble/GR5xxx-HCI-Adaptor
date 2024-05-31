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

#include "board_SK.h"

//#include "app_log.h"

//#include <assert.h>
//#include <string.h>
//#include <stdio.h>
//#include <errno.h>


//#include "console/console.h"
//#include "config/config.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h" 
#include "services/gatt/ble_svc_gatt.h"
#include "blecsc_sens.h"

#include "nimble/nimble_npl_os.h"
#include "nimble/ble.h"
#include "nimble/npl_freertos.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"


/* Wheel size for simulation calculations */
#define CSC_SIM_WHEEL_CIRCUMFERENCE_MM            2000
/* Simulated cadence lower limit */
#define CSC_SIM_CRANK_RPM_MIN                     20
/* Simulated cadence upper limit */
#define CSC_SIM_CRANK_RPM_MAX                     100
/* Simulated speed lower limit */
#define CSC_SIM_SPEED_KPH_MIN                     0
/* Simulated speed upper limit */
#define CSC_SIM_SPEED_KPH_MAX                     35

/* Noticication status */
static bool notify_state = false;

/* Connection handle */
static uint16_t conn_handle;

static uint8_t blecsc_addr_type;

/* Advertised device name  */
static const char *device_name = "blecsc_sensor";

/* Measurement and notification timer */
static struct ble_npl_callout blecsc_measure_timer;

/* Variable holds current CSC measurement state */
static struct ble_csc_measurement_state csc_measurement_state;

/* Variable holds simulated speed (kilometers per hour) */
static uint16_t csc_sim_speed_kph = CSC_SIM_SPEED_KPH_MIN;

/* Variable holds simulated cadence (RPM) */
static uint8_t csc_sim_crank_rpm = CSC_SIM_CRANK_RPM_MIN;

extern void ble_store_ram_init(void);

static int blecsc_gap_event(struct ble_gap_event *event, void *arg);


/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void
blecsc_advertise(void)
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

//    /*
//     * Indicate that the TX power level field should be included; have the
//     * stack fill this value automatically.  This is done by assigning the
//     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
//     */
//    fields.tx_pwr_lvl_is_present = 1;
//    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    /*
     * Set appearance.
     */
    fields.appearance = ble_svc_gap_device_appearance();
    fields.appearance_is_present = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        BLE_HS_LOG(ERROR, "error setting advertisement data; rc=%d", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(blecsc_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, blecsc_gap_event, NULL);
    if (rc != 0) {
        BLE_HS_LOG(ERROR, "error enabling advertisement; rc=%d", rc);
        return;
    }
}


/* Update simulated CSC measurements.
 * Each call increments wheel and crank revolution counters by one and
 * computes last event time in order to match simulated candence and speed.
 * Last event time is expressedd in 1/1024th of second units.
 *
 *                 60 * 1024
 * crank_dt =    --------------
 *                cadence[RPM]
 *
 *
 *                circumference[mm] * 1024 * 60 * 60
 * wheel_dt =    -------------------------------------
 *                         10^6 * speed [kph]
 */
static void
blecsc_simulate_speed_and_cadence(void)
{
    uint16_t wheel_rev_period;
    uint16_t crank_rev_period;

    /* Update simulated crank and wheel rotation speed */
    csc_sim_speed_kph++;
    if (csc_sim_speed_kph >= CSC_SIM_SPEED_KPH_MAX) {
         csc_sim_speed_kph = CSC_SIM_SPEED_KPH_MIN;
    }

    csc_sim_crank_rpm++;
    if (csc_sim_crank_rpm >= CSC_SIM_CRANK_RPM_MAX) {
         csc_sim_crank_rpm = CSC_SIM_CRANK_RPM_MIN;
    }

    /* Calculate simulated measurement values */
    if (csc_sim_speed_kph > 0){
        wheel_rev_period = (36*64*CSC_SIM_WHEEL_CIRCUMFERENCE_MM) /
                           (625*csc_sim_speed_kph);
        csc_measurement_state.cumulative_wheel_rev++;
        csc_measurement_state.last_wheel_evt_time += wheel_rev_period;
    }

    if (csc_sim_crank_rpm > 0){
        crank_rev_period = (60*1024) / csc_sim_crank_rpm;
        csc_measurement_state.cumulative_crank_rev++;
        csc_measurement_state.last_crank_evt_time += crank_rev_period;
    }

    BLE_HS_LOG(DEBUG, "CSC simulated values: speed = %d kph, cadence = %d",
                csc_sim_speed_kph, csc_sim_crank_rpm);
}

/* Run CSC measurement simulation and notify it to the client */
static void
blecsc_measurement(struct ble_npl_event *ev)
{
    int rc;

    rc = ble_npl_callout_reset(&blecsc_measure_timer, configTICK_RATE_HZ);
    assert(rc == 0);

    blecsc_simulate_speed_and_cadence();

    if (notify_state) {
        rc = gatt_svr_chr_notify_csc_measurement(conn_handle);
        assert(rc == 0);
    }
}

static int
blecsc_gap_event(struct ble_gap_event *event, void *arg)
{
    int ret;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
         BLE_HS_LOG(INFO, "connection %s; status=%d",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            blecsc_advertise();
            conn_handle = 0;
        }
        else {
          conn_handle = event->connect.conn_handle;
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        BLE_HS_LOG(INFO, "disconnect; reason=%d", event->disconnect.reason);
        ble_npl_callout_stop(&blecsc_measure_timer);
        conn_handle = 0;
        /* Connection terminated; resume advertising */
        blecsc_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        BLE_HS_LOG(INFO, "adv complete");
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        BLE_HS_LOG(INFO, "subscribe event attr_handle=%d",
                    event->subscribe.attr_handle);

        if (event->subscribe.attr_handle == csc_measurement_handle) {
            notify_state = event->subscribe.cur_notify;
            BLE_HS_LOG(INFO, "csc measurement notify state = %d",
                        notify_state);
            if (notify_state) {
                ret = ble_npl_callout_reset(&blecsc_measure_timer, configTICK_RATE_HZ);
                assert(ret == 0);
            } else {
                ble_npl_callout_stop(&blecsc_measure_timer);
            }
        }
        else if (event->subscribe.attr_handle == csc_control_point_handle) {
            gatt_svr_set_cp_indicate(event->subscribe.cur_indicate);
            BLE_HS_LOG(INFO, "csc control point indicate state = %d",
                        event->subscribe.cur_indicate);
        }
        break;

    case BLE_GAP_EVENT_MTU:
        BLE_HS_LOG(INFO, "mtu update event; conn_handle=%d mtu=%d",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;

    }

    return 0;
}

static void
blecsc_on_sync(void)
{
    int rc;

    /* Figure out address to use while advertising (no privacy) */
    rc = ble_hs_id_infer_auto(0, &blecsc_addr_type);
    assert(rc == 0);

    /* Begin advertising */
    blecsc_advertise();
}

void __aeabi_assert(const char *expr, const char *file, int line)
{
    BLE_HS_LOG(ERROR, "<Assert> %s(%s  %d)\r\n", expr, file, line);
}


void blecsc_host_task(void *param)
{
    BLE_HS_LOG(INFO, "BLE Host Task Started\r\n");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();
}



int main(void)
{
    int ret;

    board_init();

    nimble_port_init();

    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = blecsc_on_sync;

    /* Initialize measurement and notification timer */
    ble_npl_callout_init(&blecsc_measure_timer, nimble_port_get_dflt_eventq(), blecsc_measurement, NULL);



    ret = ble_svc_gap_device_name_set(device_name); 
    assert(ret == 0);

    ble_svc_gap_device_appearance_set(BLE_SVC_GAP_APPEARANCE_CYC_SPEED_AND_CADENCE_SENSOR);
    assert(ret == 0);

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ret = gatt_svr_init(&csc_measurement_state);
    assert(ret == 0);
    ble_store_ram_init();

    nimble_port_freertos_init(blecsc_host_task);
    vTaskStartScheduler();
    return 0;
}

