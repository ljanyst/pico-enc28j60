// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#include "pico/enc28j60_driver.h"
#include "enc28j60_cmd.h"
#include "enc28j60_spi.pio.h"

#include <hardware/clocks.h>
#include <hardware/gpio.h>

#include <stdint.h>
#include <stdlib.h>

enc28j60_config *enc28j60_get_default_config()
{
    enc28j60_config *cfg = calloc(1, sizeof(enc28j60_config));
    if (cfg == NULL) {
        panic("Failed to allocate memory for ENC28J60 config");
    }

    cfg->pio = pio0;
    cfg->sm = 0;
    cfg->miso_pin = 16;
    cfg->mosi_pin = 17;
    cfg->sclk_pin = 18;
    return cfg;
}

static void initialize_pio(enc28j60_config *cfg)
{
    // Configure the program
    uint offset = pio_add_program(cfg->pio, &enc28j60_spi_program);
    pio_sm_config c = enc28j60_spi_program_get_default_config(offset);

    sm_config_set_out_pins(&c, cfg->mosi_pin, 1);
    sm_config_set_in_pins(&c, cfg->miso_pin);
    sm_config_set_set_pins(&c, cfg->mosi_pin, 1);
    sm_config_set_sideset_pins(&c, cfg->sclk_pin);
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_in_shift(&c, false, true, 8);

    // the device requires a 20MHz clock or lower, but we'll operate at 60MHz to
    // have the time for various shananigans and copensate with explicit delays
    float div = (float)clock_get_hz(clk_sys) / 120000000;
    sm_config_set_clkdiv(&c, div);

    // Slave select to high, clock, and mosi to low
    pio_sm_set_pins_with_mask(cfg->pio, cfg->sm, (2u << cfg->sclk_pin),
                              (3u << cfg->sclk_pin) | (1u << cfg->mosi_pin));
    // Slave select, clock,  and mosi to output; miso to input
    pio_sm_set_pindirs_with_mask(
        cfg->pio, cfg->sm, (3u << cfg->sclk_pin) | (1u << cfg->mosi_pin),
        (3u << cfg->sclk_pin) | (1u << cfg->mosi_pin) | (1u << cfg->miso_pin));
    pio_gpio_init(cfg->pio, cfg->mosi_pin);
    pio_gpio_init(cfg->pio, cfg->miso_pin);
    pio_gpio_init(cfg->pio, cfg->sclk_pin);
    pio_gpio_init(cfg->pio, cfg->sclk_pin + 1);
    hw_set_bits(&cfg->pio->input_sync_bypass, 1u << cfg->miso_pin);

    pio_sm_init(cfg->pio, cfg->sm, offset, &c);
    pio_sm_set_enabled(cfg->pio, cfg->sm, true);
}

enc28j60 *enc28j60_init(enc28j60_config *cfg)
{
    if (cfg == NULL) {
        panic("Missing ENC28J60 config");
    }

    enc28j60 *eth = calloc(1, sizeof(enc28j60_config));
    if (eth == NULL) {
        panic("Failed to allocate memory for ENC28J60 object");
    }
    eth->cfg = cfg;

    initialize_pio(cfg);

    return eth;
}

uint8_t enc28j60_revision(enc28j60 *eth)
{
    bank_set_blk(eth, 3);
    return reg_read_e_blk(eth, EREVID);
}

void enc28j60_deinit(enc28j60 *eth)
{
}
