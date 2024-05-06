// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#pragma once

#include <hardware/pio.h>
#include <stdint.h>

typedef struct {
    PIO pio;
    uint sm;
    uint miso_pin;
    uint mosi_pin;
    uint sclk_pin; //!< pin sclk_pin + 1 will be used as slave select

    //! Size of the  recevie buffer in bytes; the device has 8kB RAM for
    //! buffers; what is not a receive buffer will be used as a transmit
    //! buffer; the relative sizes depend on the anticipated application
    //! traffic.
    uint16_t rx_size;

    uint8_t mac_addr[6];
} enc28j60_config;

typedef struct {
    enc28j60_config *cfg;
} enc28j60;

//! Get a default configuration object
enc28j60_config *enc28j60_get_default_config();

//! Configure an ENC28J60
enc28j60 *enc28j60_init(enc28j60_config *cfg);

//! Deinitialize an ENC28J60
void enc28j60_deinit(enc28j60 *eth);

//! Get the revision id of the hardware
uint8_t enc28j60_revision(enc28j60 *eth);
