/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "grx_sys.h"
#include "user_app.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
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

#define  HCI_CACHE_BUF_LEN    4096
#define  HCI_UART_TX_BUF_LEN  4096
#define  HCI_UART_RX_BUF_LEN  4096

/*
 * MACRO VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

extern void ble_sdk_patch_env_init(void);
extern void ble_test_evn_init(void);

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

static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x58, 0x17, 0x21, 0x05, 0x24, 0x20};

static uint8_t s_hci_cache_buffer[HCI_CACHE_BUF_LEN] = {0};
static uint8_t s_uart_tx_buffer[HCI_UART_TX_BUF_LEN] = {0};
static uint8_t s_uart_rx_buffer[HCI_UART_RX_BUF_LEN] = {0};

static ble_hci_rx_channel_t  s_hci_ad_rx_channel =
{
    .p_channel      = s_hci_cache_buffer,
    .cache_size   = sizeof(s_hci_cache_buffer),
};

static void hci_uart_callback(app_uart_evt_t *p_evt)
{
    if (APP_UART_EVT_RX_DATA == p_evt->type)
    {
        ble_hci_host_packet_send(s_uart_rx_buffer, p_evt->data.size);

        memset(s_uart_rx_buffer, 0, HCI_UART_RX_BUF_LEN);
        app_uart_receive_async(HCI_UART_ID, s_uart_rx_buffer, sizeof(s_uart_rx_buffer));
    }
}

static void hci_ad_host_recv_cb(uint8_t *p_data, uint16_t length)
{
    app_uart_transmit_async(HCI_UART_ID, p_data, length);
}

int main (void)
{
    SYS_SET_BD_ADDR(s_bd_addr);

    app_uart_tx_buf_t uart_buffer = {0};

    uart_buffer.tx_buf = s_uart_tx_buffer;
    uart_buffer.tx_buf_size = sizeof(s_uart_tx_buffer);

    app_uart_init(&s_hci_uart_params, hci_uart_callback, &uart_buffer);
    app_uart_dma_init(&s_hci_uart_params);
    app_uart_receive_async(HCI_UART_ID, s_uart_rx_buffer, sizeof(s_uart_rx_buffer));

    ble_hci_init(&s_hci_ad_rx_channel, hci_ad_host_recv_cb);
    ble_sdk_patch_env_init();
    ble_test_evn_init();
    ble_stack_controller_init(&heaps_table);

    //loop
    while(1)
    {
        pwr_mgmt_schedule();
    }
}

