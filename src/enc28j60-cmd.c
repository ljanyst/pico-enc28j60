// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#include "enc28j60-cmd.h"
#include "pico/enc28j60-driver.h"

#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/pio.h>

#include <stdlib.h>

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

typedef struct irq_arg_chain {
    uint irq;
    uint channel;
    void *data;
    void (*cb)(void *);
    struct irq_arg_chain *next;
} irq_arg_chain;

static irq_arg_chain irq_args;

static void __time_critical_func(dma_irq_handler)()
{
    irq_arg_chain *cur = irq_args.next;
    while (cur) {
        if (dma_irqn_get_channel_status(cur->irq, cur->channel)) {
            cur->cb(cur->data);
            dma_irqn_acknowledge_channel(cur->irq, cur->channel);
            break;
        }
        cur = cur->next;
    }
}

static void add_channel_callback(uint irq, uint channel, void (*cb)(void *),
                                 void *data)
{
    irq_arg_chain *arg = calloc(1, sizeof(irq_arg_chain));
    if (arg == NULL) {
        panic("Failed to allocate irq_arg_chain object for dma channels");
    }
    arg->irq = irq;
    arg->channel = channel;
    arg->data = data;
    arg->cb = cb;
    arg->next = irq_args.next;
    irq_args.next = arg;
}

typedef struct {
    uint32_t len;
    const void *data;
} tx_ctrl_t;

#define TX_SIZE 512
#define RX_SIZE 64
#define CTRL_NUM 64

struct enc28j60_cmd_buf {
    uint8_t tx_buf[TX_SIZE];
    uint8_t *tx_buf_ptr;

    uint8_t rx_buf[RX_SIZE];
    uint8_t *rx_buf_ptr;

    tx_ctrl_t tx_ctrl[CTRL_NUM];

    uint16_t tx_bytes;
    uint16_t rx_bytes;

    uint tx_channel;
    uint rx_channel;
    uint tx_ctrl_channel;
};

static void __time_critical_func(tx_irq_handler)(void *data)
{
    enc28j60 *eth = data;
    if (!eth->cmd_buf->rx_bytes) {
        eth->cfg->notify.notify(eth->cfg->notify.data);
    }
}

static void __time_critical_func(rx_irq_handler)(void *data)
{
    enc28j60 *eth = data;
    if (eth->cmd_buf->rx_bytes) {
        eth->cfg->notify.notify(eth->cfg->notify.data);
    }
}

void enc28j60_cmd_buf_init(enc28j60 *eth)
{
    enc28j60_config *cfg = eth->cfg;

    eth->cmd_buf = calloc(1, sizeof(enc28j60_cmd_buf));
    if (eth->cmd_buf == NULL) {
        panic("Failed to allocate memory for ENC28J60 command buffer");
    }

    enc28j60_cmd_buf *b = eth->cmd_buf;
    b->tx_buf_ptr = b->tx_buf;
    b->rx_buf_ptr = b->rx_buf;

    b->tx_channel = dma_claim_unused_channel(true);
    b->tx_ctrl_channel = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(b->tx_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(cfg->pio, cfg->sm, true));

    // Trigger trigger the control channel when tx channel completes
    channel_config_set_chain_to(&c, b->tx_ctrl_channel);

    // Raise the IRQ when 0 is written to a trigger register (end of chain)
    channel_config_set_irq_quiet(&c, true);

    dma_channel_configure(b->tx_channel, &c,
                          (void *)&cfg->pio->txf[cfg->sm], // write address
                          NULL, // read address
                          0, // transfer count
                          false // trigger
    );
    add_channel_callback(cfg->dma_irq, b->tx_channel, tx_irq_handler, eth);
    dma_irqn_set_channel_enabled(cfg->dma_irq, b->tx_channel, true);

    b->rx_channel = dma_claim_unused_channel(true);
    c = dma_channel_get_default_config(b->rx_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(cfg->pio, cfg->sm, false));
    dma_channel_configure(b->rx_channel, &c,
                          NULL, // write address
                          (void *)&cfg->pio->rxf[cfg->sm], // read address
                          0, // transfer count
                          false // trigger
    );
    add_channel_callback(cfg->dma_irq, b->rx_channel, rx_irq_handler, eth);
    dma_irqn_set_channel_enabled(cfg->dma_irq, b->rx_channel, true);

    c = dma_channel_get_default_config(b->tx_ctrl_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);

    // Only 3 lower bits of the write address are allowed to change, so we loop
    // every 8 bytes written
    channel_config_set_ring(&c, true, 3);

    // The DMA's four CSRs are aliased multiple times in memory. Each alias —
    // of which there are four — exposes the same four physical registers, but
    // in a different order.
    // We'll use alias 3. See section 2.5.2.1. of the RP2040 datasheet.
    dma_channel_configure(b->tx_ctrl_channel, &c,
                          &dma_hw->ch[b->tx_channel].al3_transfer_count, NULL,
                          0, false);

    uint irq = cfg->dma_irq ? DMA_IRQ_1 : DMA_IRQ_0;
    irq_add_shared_handler(irq, dma_irq_handler,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(irq, true);
}

bool enc28j60_cmd_buf_encode_cmd(enc28j60 *eth, uint32_t cmd, const void *src)
{
    enc28j60_cmd_buf *b = eth->cmd_buf;

    uint8_t c = CMD_OP(cmd);
    size_t rx_sz = CMD_RXSZ(cmd);
    size_t tx_sz = 2 + CMD_TXSZ(cmd);
    uint8_t data[4] = { cmd >> 24, cmd >> 16, cmd >> 8, cmd };

    if (b->tx_bytes + tx_sz > TX_SIZE) {
        return false;
    }

    if (b->rx_bytes + rx_sz > RX_SIZE) {
        return false;
    }

    for (int i = 0; i < tx_sz; ++i) {
        *b->tx_buf_ptr++ = data[i];
    }

    b->tx_bytes += tx_sz;
    b->rx_bytes += rx_sz;
    return false;
}

bool enc28j60_cmd_buf_decode_rcr(enc28j60 *eth, bool is_eth, uint8_t *val)
{
    enc28j60_cmd_buf *b = eth->cmd_buf;
    size_t rx_sz = is_eth ? 1 : 2;
    if ((uint32_t)b->rx_buf_ptr - (uint32_t)b->rx_buf + rx_sz >= RX_SIZE) {
        return false;
    }

    if (is_eth) {
        *val = *b->rx_buf_ptr++;
    } else {
        ++b->rx_buf_ptr;
        *val = *b->rx_buf_ptr++;
    }

    return true;
}

void enc28j60_cmd_buf_reset(enc28j60 *eth)
{
    enc28j60_cmd_buf *b = eth->cmd_buf;
    b->tx_buf_ptr = b->tx_buf;
    b->rx_buf_ptr = b->rx_buf;
    b->tx_bytes = 0;
    b->rx_bytes = 0;
}

void enc28j60_cmd_buf_execute(enc28j60 *eth)
{
    enc28j60_cmd_buf *b = eth->cmd_buf;
    enc28j60_config *cfg = eth->cfg;

    if (b->rx_bytes) {
        dma_channel_set_write_addr(b->rx_channel, b->rx_buf, false);
        dma_channel_set_trans_count(b->rx_channel, b->rx_bytes, true);
    }

    b->tx_ctrl[0].data = b->tx_buf;
    b->tx_ctrl[0].len = b->tx_bytes;

    dma_channel_set_read_addr(b->tx_ctrl_channel, b->tx_ctrl, false);
    dma_channel_set_trans_count(b->tx_ctrl_channel, 2, true);

    cfg->notify.wait(cfg->notify.data);
}
