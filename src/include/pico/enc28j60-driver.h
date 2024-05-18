// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#pragma once

#include <hardware/pio.h>
#include <stdint.h>

typedef void (*enc28j60_irq_hanler)(void *);

typedef struct {
    PIO pio;
    uint sm;
    uint miso_pin;
    uint mosi_pin;
    uint sclk_pin; //!< pin sclk_pin + 1 will be used as slave select
    uint irq_pin;

    //! Size of the  recevie buffer in bytes; the device has 8kB RAM for
    //! buffers; what is not a receive buffer will be used as a transmit
    //! buffer; the relative sizes depend on the anticipated application
    //! traffic.
    uint16_t rx_size;

    uint8_t mac_addr[6];
    void *irq_data;
    enc28j60_irq_hanler irq_cb;
} enc28j60_config;

typedef struct {
    enc28j60_config *cfg;
    uint16_t wptr;
} enc28j60;

//! Get a default configuration object
enc28j60_config *enc28j60_get_default_config();

//! Configure an ENC28J60
enc28j60 *enc28j60_init(enc28j60_config *cfg);

//! Deinitialize an ENC28J60
void enc28j60_deinit(enc28j60 *eth);

//! Get the revision id of the hardware
uint8_t enc28j60_revision(enc28j60 *eth);

//! Get link status
bool enc28j60_link_status(enc28j60 *eth);

//! Disable the interrupt and get the interrupt flags
uint8_t enc28j60_irq_flags(enc28j60 *eth);

//! Acknowledge and enable the interrupt
void enc28j60_irq_ack(enc28j60 *eth, uint8_t flags);

//! Check if the interrupt is the link interrput
bool enc28j60_irq_is_link(uint8_t flags);

//! Check if the interrupt is the transmit interrupt
bool enc28j60_irq_tx(uint8_t flags);

//! Check if the interrupt is the receive interrupt
bool enc28j60_irq_rx(uint8_t flags);

//! Transmit frame
bool enc28j60_frame_tx_blk(enc28j60 *eth, size_t size, const void *data);

//! Receive frame
bool enc28j60_frame_rx_blk(enc28j60 *eth, void *data);
