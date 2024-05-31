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


#include <nimble/hci_common.h>
#include <nimble/transport.h>
#include <nimble/transport/hci_h4.h>

#include "app_uart.h"
#include "app_uart_dma.h"

#define HCI_UART_ID                        APP_UART_ID_1
#define HCI_UART_BAUDRATE                  115200
#define HCI_UART_TX_IO_TYPE                APP_IO_TYPE_AON
#define HCI_UART_RX_IO_TYPE                APP_IO_TYPE_AON
#define HCI_UART_TX_PIN                    APP_IO_PIN_6
#define HCI_UART_RX_PIN                    APP_IO_PIN_5
#define HCI_UART_TX_PINMUX                 APP_IO_MUX_11
#define HCI_UART_RX_PINMUX                 APP_IO_MUX_12

#define HCI_CACHE_BUF_LEN       4096

#define HCI_CMD_HDR_LEN         (3) /*!< \brief Command packet header length */

extern uint8_t ble_hs_enabled_state;

static struct hci_h4_sm         s_hci_h4sm;

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


int ble_transport_to_ll_cmd_impl(void *buf)
{
    sdk_err_t ret;
    uint16_t  len;
    uint8_t   cmd = HCI_H4_CMD;

    len = HCI_CMD_HDR_LEN + ((uint8_t *)buf)[2];

//    APP_LOG_DEBUG("HOST  HCI >>>>");
//    APP_LOG_HEX_DUMP(&cmd, 1);
    ret = app_uart_transmit_async(HCI_UART_ID, &cmd, 1);
    if (!ret)
    {
//        APP_LOG_HEX_DUMP(buf, len);
        ret = app_uart_transmit_async(HCI_UART_ID, buf, len);
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

    ret = app_uart_transmit_async(HCI_UART_ID, data, len);

    os_mbuf_free_chain(om);

    return (ret) ? BLE_ERR_MEM_CAPACITY : 0;
}



static app_uart_params_t s_hci_uart_params = {
    .id      = HCI_UART_ID,
    .pin_cfg = {
           .tx = {
              .type = HCI_UART_TX_IO_TYPE,
              .mux  = HCI_UART_TX_PINMUX,
              .pin  = HCI_UART_TX_PIN,
              .pull = APP_IO_PULLUP,
           },
           .rx = {
              .type = HCI_UART_RX_IO_TYPE,
              .mux  = HCI_UART_RX_PINMUX,
              .pin  = HCI_UART_RX_PIN,
              .pull = APP_IO_PULLUP,
           },
    },
    .dma_cfg = {
        .tx_dma_instance = DMA0,
        .rx_dma_instance = DMA0,
        .tx_dma_channel  = DMA_Channel0,
        .rx_dma_channel  = DMA_Channel1 ,
    },
    .init = {
        .baud_rate = 115200,
        .data_bits = UART_DATABITS_8,
        .stop_bits = UART_STOPBITS_1,
        .parity    = UART_PARITY_NONE,
        .hw_flow_ctrl    = UART_HWCONTROL_NONE,
        .rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE,
    },
};

static uint8_t s_tx_buffer[HCI_CACHE_BUF_LEN] = {0};
static uint8_t s_rx_buffer[HCI_CACHE_BUF_LEN] = {0};

static void hci_uart_callback(app_uart_evt_t *p_evt)
{
    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {
        int rlen;

        if (!ble_hs_enabled_state)
        {
            return;
        }

        rlen = hci_h4_sm_rx(&s_hci_h4sm, s_rx_buffer, p_evt->data.size);
        assert(rlen >= 0);

        uint16_t ret =  app_uart_receive_async(HCI_UART_ID, s_rx_buffer, sizeof(s_rx_buffer));
        assert(ret == 0);
    }
}

void ble_transport_ll_init(void)
{
    hci_h4_sm_init(&s_hci_h4sm, &hci_h4_allocs_from_ll, ble_hci_frame_cb);

    app_uart_tx_buf_t uart_buffer = {0};

    uart_buffer.tx_buf = s_tx_buffer;
    uart_buffer.tx_buf_size = sizeof(s_tx_buffer);

    uint16_t ret = app_uart_init(&s_hci_uart_params, hci_uart_callback, &uart_buffer);
    assert(ret == 0);

    ret = app_uart_dma_init(&s_hci_uart_params);
    assert(ret == 0);

    ret =  app_uart_receive_async(HCI_UART_ID, s_rx_buffer, sizeof(s_rx_buffer));
    assert(ret == 0);
}

