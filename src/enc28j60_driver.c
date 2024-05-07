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

    // Use half of the buffer as a receive FIFO
    cfg->rx_size = 0x1000;

    // Locally Administered Addresses are defined by the
    // second-least-significant bit of the first octet. You may choose any MAC
    // addresses you want, as long as it matches one of the following:
    //
    // x2:xx:xx:xx:xx:xx
    // x6:xx:xx:xx:xx:xx
    // xA:xx:xx:xx:xx:xx
    // xE:xx:xx:xx:xx:xx
    cfg->mac_addr[0] = 0xa2;
    cfg->mac_addr[1] = 0x32;
    cfg->mac_addr[2] = 0x15;
    cfg->mac_addr[3] = 0xcc;
    cfg->mac_addr[4] = 0x72;
    cfg->mac_addr[5] = 0x0a;

    return cfg;
}

static void initialize_io(enc28j60_config *cfg)
{
    // Configure the PIO program
    uint offset = pio_add_program(cfg->pio, &enc28j60_spi_program);
    pio_sm_config c = enc28j60_spi_program_get_default_config(offset);

    sm_config_set_out_pins(&c, cfg->mosi_pin, 1);
    sm_config_set_in_pins(&c, cfg->miso_pin);
    sm_config_set_set_pins(&c, cfg->mosi_pin, 1);
    sm_config_set_sideset_pins(&c, cfg->sclk_pin);
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_in_shift(&c, false, true, 8);

    // The device seems to work reliably at 4 MHz clock or lower otherwise
    // MAC and MII accesses are unreliable. We scale the PIO clock to 40 MHz
    // becaue we take 10 PIO cycles for each SPI cycle in the PIO program.
    float div = (float)clock_get_hz(clk_sys) / 40000000;
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

    // Run the program
    pio_sm_init(cfg->pio, cfg->sm, offset, &c);
    pio_sm_set_enabled(cfg->pio, cfg->sm, true);

    // Initialize the interrupt
}

static void initialize_enc28j60(enc28j60 *eth, enc28j60_config *cfg)
{
    // Reset, just in case we power cycled the controller, but not the device
    system_reset(eth);

    // Set up the receving FIFO, section 6.1 of the manual
    bank_set_blk(eth, 0);

    // The receive buffer starts at 0
    reg_write_blk(eth, ERXSTL, 0);
    reg_write_blk(eth, ERXSTH, 0);

    // The receive buffer ends at the place accommodating the configured size;
    // the byte pointed to by the reg is included in the rx buffer
    uint16_t rx_sz = cfg->rx_size - 1;
    reg_write_blk(eth, ERXNDL, rx_sz);
    reg_write_blk(eth, ERXNDH, rx_sz >> 8);

    // The initial value of the receive read pointer must be the same as ERXST
    reg_write_blk(eth, ERXRDPTL, 0);
    reg_write_blk(eth, ERXRDPTH, 0);

    // Accept only valid unicast, multicast, and broadcast packets;
    // sections 6.3 and 8.0
    bank_set_blk(eth, 1);
    reg_write_blk(eth, ERXFCON, UCEN | CRCEN | MCEN | BCEN);

    // Wait for the oscilator to stabilize before accessing MAC or PHY
    while (!(reg_read_e_blk(eth, ESTAT) & CLKRDY))
        ;

    // The device does not support automatic duplex negotiation, if plugged to a
    // switch with automatic duplex negotiation it will appear a as 10M
    // half-duplex. For full duplex, the switch needs to be reconfigured
    // manually. We want this to be plug-and-play, so we won't bother with full
    // duplex.

    // MAC initialization; chapter 6.5
    bank_set_blk(eth, 2);

    // Enable the MAC to receive packets
    reg_write_blk(eth, MACON1, MARXEN);

    // Enable automatic padding of short frames and CRC generation
    reg_write_blk(eth, MACON3, PADCRC);

    // When the medium is occupied, the MAC will wait indefinitely for it to
    // become free when attempting to transmit; IEEE 802.3 compliance
    reg_write_blk(eth, MACON4, DEFER);

    // Ethernet normally supports frames of 1518 bytes or less
    uint16_t max_frame_length = 1518;
    reg_write_blk(eth, MAMXFLL, max_frame_length);
    reg_write_blk(eth, MAMXFLH, max_frame_length >> 8);

    // Delay between the end of one transmission and the beginning of the next
    // in a back-to-back sequence. 0x12 represents the minimum IEEE specified
    // Inter-Packet Gap (IPG) of 9.6 us.
    reg_write_blk(eth, MABBIPG, 0x12);

    // Non-Back-to-Back Inter-Packet Gap
    reg_write_blk(eth, MAIPGL, 0x12);
    reg_write_blk(eth, MAIPGH, 0x0c);

    // Program the MAC address
    bank_set_blk(eth, 3);
    reg_write_blk(eth, MAADR1, cfg->mac_addr[0]);
    reg_write_blk(eth, MAADR2, cfg->mac_addr[1]);
    reg_write_blk(eth, MAADR3, cfg->mac_addr[2]);
    reg_write_blk(eth, MAADR4, cfg->mac_addr[3]);
    reg_write_blk(eth, MAADR5, cfg->mac_addr[4]);
    reg_write_blk(eth, MAADR6, cfg->mac_addr[5]);

    // Disable the half-duplex loopback
    reg_write_p_blk(eth, PHCON2, HDLDIS);

    // Continually scan and fetch the link status from PHY to MAC so that we
    // can read it directly without delay if we get a related interrupt
    bank_set_blk(eth, 2);
    reg_write_blk(eth, MIREGADR, PHSTAT2);
    reg_write_blk(eth, MICMD, MIISCAN);

    // Wait for the first scan to complete
    bank_set_blk(eth, 3);
    while (reg_read_m_blk(eth, MISTAT) & NVALID)
        ;
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

    initialize_io(cfg);
    initialize_enc28j60(eth, cfg);

    return eth;
}

uint8_t enc28j60_revision(enc28j60 *eth)
{
    bank_set_blk(eth, 3);
    return reg_read_e_blk(eth, EREVID);
}

uint16_t enc28j60_link_status(enc28j60 *eth)
{
    bank_set_blk(eth, 2);
    uint8_t result = reg_read_m_blk(eth, MIRDH);
    return (result & LSTAT) ? 1 : 0;
}

void enc28j60_deinit(enc28j60 *eth)
{
}
