// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#include "enc28j60_cmd.h"

// Don't keep this function in flash
void __time_critical_func(enc28j60_execute_blocking)(enc28j60 *eth,
                                                     uint32_t cmd, void *dst,
                                                     const void *src)
{
    io_rw_8 *txfifo = (io_rw_8 *)&eth->cfg->pio->txf[eth->cfg->sm];
    io_rw_8 *rxfifo = (io_rw_8 *)&eth->cfg->pio->rxf[eth->cfg->sm];

    uint8_t c = CMD_OP(cmd);
    size_t rx_sz = CMD_RXSZ(cmd);
    size_t tx_sz = CMD_TXSZ(cmd);

    size_t tx_remain = 2 + tx_sz;
    size_t rx_remain = rx_sz;
    uint8_t data[4] = { cmd >> 24, cmd >> 16, cmd >> 8, cmd };
    const uint8_t *snd = data;
    uint8_t *rcv = dst;

    // For WBM, we first transmit the command header and then the data payload
    // from the source buffer skipping the data payload byte from the command
    // itself
    if (c == WBM_OP) {
        for (int i = 0; i < 3;) {
            if (!pio_sm_is_tx_fifo_full(eth->cfg->pio, eth->cfg->sm)) {
                *txfifo = *snd++;
                ++i;
            }
        }
        tx_remain = tx_sz - 1;
        snd = src;
    }

    // Push the data to the TX fifo and read from the RX fifo at the same time
    // as necessary for SPI, although we don't have commands where reading
    // separately would actually block the PIO program
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(eth->cfg->pio, eth->cfg->sm)) {
            *txfifo = *snd++;
            --tx_remain;
        }
        if (rx_remain &&
            !pio_sm_is_rx_fifo_empty(eth->cfg->pio, eth->cfg->sm)) {
            *rcv++ = *rxfifo;
            --rx_remain;
        }
    }
}
