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

#include "gr_includes.h"

#include "scatter_common.h"
#include "flash_scatter_config.h"

#include "app_log.h"

#include <nimble/hci_common.h>
#include <nimble/transport.h>
#include <nimble/transport/hci_h4.h>

STACK_HEAP_INIT(heaps_table);

#define HCI_CACHE_BUF_LEN       4096

#define HCI_CMD_HDR_LEN         (3) /*!< \brief Command packet header length */

extern uint8_t ble_hs_enabled_state;

static struct hci_h4_sm         s_hci_h4sm;

static uint8_t                  s_hci_cache_buffer[HCI_CACHE_BUF_LEN] = {0};
static ble_hci_rx_channel_t     s_hci_ad_rx_channel =
{
    .p_channel      = s_hci_cache_buffer,
    .cache_size   = sizeof(s_hci_cache_buffer),
};

static int ble_hci_frame_cb(uint8_t pkt_type, void *data)
{
    int rc;

    switch (pkt_type)
    {
        case HCI_H4_ACL:
            rc = ble_transport_to_hs_acl(data);
            break;

        case HCI_H4_EVT:
            rc = ble_transport_to_hs_evt(data);
            break;

        default:
            assert(0);
            break;
    }

    return rc;
}

static void ble_hci_host_recv_cb(uint8_t *p_data, uint16_t length)
{
    int rlen;

    if (!ble_hs_enabled_state)
    {
        return;
    }

    rlen = hci_h4_sm_rx(&s_hci_h4sm, p_data, length);
    assert(rlen >= 0);
}


int ble_transport_to_ll_cmd_impl(void *buf)
{
    sdk_err_t ret;
    uint16_t  len;
    uint8_t   cmd = HCI_H4_CMD;

    len = HCI_CMD_HDR_LEN + ((uint8_t *)buf)[2];

//    APP_LOG_DEBUG("HOST  HCI >>>>");
//    APP_LOG_HEX_DUMP(&cmd, 1);
    ret = ble_hci_host_packet_send(&cmd, 1);
    if (!ret)
    {
//        APP_LOG_HEX_DUMP(buf, len);
        ret = ble_hci_host_packet_send(buf, len);
    }

    ble_transport_free(buf);

    return (ret) ? BLE_ERR_MEM_CAPACITY :  0;
}

int ble_transport_to_ll_acl_impl(struct os_mbuf *om)
{
    sdk_err_t ret;
    uint16_t  len = 0;
    uint8_t   data[MYNEWT_VAL(BLE_TRANSPORT_ACL_SIZE) + 1];

    if (OS_MBUF_PKTLEN(om) == 0) {
        os_mbuf_free_chain(om);
        return 0;
    }
    data[0] = HCI_H4_ACL;
    len++;

    os_mbuf_copydata(om, 0, OS_MBUF_PKTLEN(om), &data[1]);
    len += OS_MBUF_PKTLEN(om);

    ret = ble_hci_host_packet_send(data, len);

    os_mbuf_free_chain(om);

    return (ret) ? BLE_ERR_MEM_CAPACITY : 0;
}

void ble_transport_ll_init(void)
{
    hci_h4_sm_init(&s_hci_h4sm, &hci_h4_allocs_from_ll, ble_hci_frame_cb);

    ble_hci_init(&s_hci_ad_rx_channel, ble_hci_host_recv_cb);

    extern void ble_sdk_patch_env_init(void);
    extern void ble_test_evn_init(void);

    ble_sdk_patch_env_init();
    ble_test_evn_init();
    ble_stack_controller_init(&heaps_table);
}

