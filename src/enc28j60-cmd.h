// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#pragma once

#include "pico/enc28j60-driver.h"

#include <stdint.h>

// Section 4.2 of the manual
#define RCR_OP 0x0
#define RBM_OP 0x1
#define WCR_OP 0x2
#define WBM_OP 0x3
#define BFS_OP 0x4
#define BFC_OP 0x5
#define SRC_OP 0x7
#define MEM_CONST 0x1a

#define SH(VAL, N) (((uint32_t)(VAL & 0xff)) << N)

// The PIO driver can receive up to 32 bytes with one command and send up to
// 31 bytes, the sizes are in bits and must be multiples of 8bits, ie. we only
// transmit th whole bytes
#define CMD(TXSZ, RXSZ, OPCODE, ADDR, PAYLOAD)                          \
    (SH((TXSZ + 8), 24) | SH(RXSZ, 16) | SH(OPCODE, 13) | SH(ADDR, 8) | \
     (PAYLOAD & 0xff))

// Extract opcode from a command
#define CMD_OP(CMD) ((cmd >> 13) & 0x7)

// Extract transmit size in bytes from the command
#define CMD_TXSZ(CMD) (((cmd >> 24) & 0xff) / 8)

// Extract receive size in bytes from the command
#define CMD_RXSZ(CMD) (((cmd >> 16) & 0xff) / 8)

// Read Control Register - ETH registers - receive one byte
#define RCR_E_CMD(ADDR) CMD(0, 8, RCR_OP, ADDR, 0)

// Read Control Register - MAC and MII registers - receive two bytes with the
// lower one being garbage
#define RCR_M_CMD(ADDR) CMD(0, 16, RCR_OP, ADDR, 0)

// Read Buffer Memory - size in bytes, up to 32
#define RBM_CMD(SIZE) CMD(0, (SIZE * 8), RBM_OP, MEM_CONST, 0)

// Write Control Register
#define WCR_CMD(ADDR, DATA) CMD(8, 0, WCR_OP, ADDR, DATA)

// Write Buffer Memory - size in bytes, up to 31
#define WBM_CMD(SIZE) CMD((SIZE * 8), 0, WBM_OP, MEM_CONST, 0)

// Bit Field Set
#define BFS_CMD(ADDR, DATA) CMD(8, 0, BFS_OP, ADDR, DATA)

// Bit Field Clear
#define BFC_CMD(ADDR, DATA) CMD(8, 0, BFC_OP, ADDR, DATA)

// System Reset Command
#define SRC_CMD() CMD(0, 0, SRC_OP, 0x1f, 0)

// Register addresses
// Chapter 3.1 of the manual
// Bank 0

// Buffer read pointer
#define ERDPTL 0x00
#define ERDPTH 0x01

// Buffer write pointer
#define EWRPTL 0x02
#define EWRPTH 0x03

// Start of the packet to be transmitted
#define ETXSTL 0x04
#define ETXSTH 0x05

// End of the packet to be transmitted
#define ETXNDL 0x06
#define ETXNDH 0x07

// Start of the receive buffer
#define ERXSTL 0x08
#define ERXSTH 0x09

// End of the receive buffer
#define ERXNDL 0x0a
#define ERXNDH 0x0b

// Receive read pointer - position within the RX fifo where the recevie HW is
// forbidden to write to. The recevie hardware will up to but not intluding the
// memory pointed to by this reg. If the FIFO fills up, the new data will be
// discarded. The controller must advance this pointer to receive new data.
#define ERXRDPTL 0x0c
#define ERXRDPTH 0x0d

// Receive write pointer - position within the RX buffer where HW will write
// the incoming bytes
#define ERXWRPTL 0x0e
#define ERXWRPTH 0x0f

#define EDMASTL 0x10
#define EDMASTH 0x11

#define EDMANDL 0x12
#define EDMANDH 0x13

#define EDMADSTL 0x14
#define EDMADSTH 0x15

#define EDMACSL 0x16
#define EDMACSH 0x17

// Bank 1
#define EHT0 0x00
#define EHT1 0x01
#define EHT2 0x02
#define EHT3 0x03
#define EHT4 0x04
#define EHT5 0x05
#define EHT6 0x06
#define EHT7 0x07
#define EPMM0 0x08
#define EPMM1 0x09
#define EPMM2 0x0a
#define EPMM3 0x0b
#define EPMM4 0x0c
#define EPMM5 0x0d
#define EPMM6 0x0e
#define EPMM7 0x0f
#define EPMCSL 0x10
#define EPMCSH 0x11
#define EPMOL 0x14
#define EPMOH 0x15
#define ERXFCON 0x18
#define EPKTCNT 0x19

// Bank 2
#define MACON1 0x00
#define MACON3 0x02
#define MACON4 0x03
#define MABBIPG 0x04
#define MAIPGL 0x06
#define MAIPGH 0x07
#define MACLCON1 0x08
#define MACLCON2 0x09
#define MAMXFLL 0x0a
#define MAMXFLH 0x0b
#define MICMD 0x12
#define MIREGADR 0x14
#define MIWRL 0x16
#define MIWRH 0x17
#define MIRDL 0x18
#define MIRDH 0x19

// Bank 3
#define MAADR5 0x00
#define MAADR6 0x01
#define MAADR3 0x02
#define MAADR4 0x03
#define MAADR1 0x04
#define MAADR2 0x05
#define EBSTSD 0x06
#define EBSTCON 0x07
#define EBSTCSL 0x08
#define EBSTCSH 0x09
#define MISTAT 0x0a
#define EREVID 0x12
#define ECOCON 0x15
#define EFLOCON 0x17
#define EPAUSL 0x18
#define EPAUSH 0x19

// All banks
#define EIE 0x1b
#define EIR 0x1c
#define ESTAT 0x1d
#define ECON2 0x1e
#define ECON1 0x1f

// MAC filters
// ERXFCON: Unicast
#define UCEN 0x80

// ERXFCON: CRC
#define CRCEN 0x20

// ERXFCON: Multicast
#define MCEN 0x2

// ERXFCON: Broadcast
#define BCEN 0x1

// ESTAT: Clock ready
#define CLKRDY 0x1

// EIE: Global interrupt enable
#define INTIE 0x80

// EIE: Receive packet pending enable
#define PKTIE 0x40

// EIE: Link status change enable
#define LINKIE 0x10

// EIE: Transmit enable
#define TXIE 0x8

// EIE: Receive error enable
#define RXERIE 0x1

// EIR: Receive packet pending flag
#define PKTIF 0x40

// EIR: Link status change flag
#define LINKIF 0x10

// EIR: Transmit flag
#define TXIF 0x8

// EIR: Receive error flag
#define RXERIF 0x1

// MACON1: MAC Receive Enable
#define MARXEN 0x1

// MACON3: PADCFG and TXCRCEN
#define PADCRC 0xf0

// MACON4: DEFER
#define DEFER 0x40

// MICMD: PHY reg read command
#define MIIRD 0x1

// MICMD: PHY reg scan command
#define MIISCAN 0x2

// MISTAT: Busy
#define BUSY 0x1

// MISTAT: Not valid
#define NVALID 0x4

// ECON1: Receive enable
#define RXEN 0x4

// ECON1: Transmit request to send
#define TXRTS 0x8

// ECON2: Packet Decrement
#define PKTDEC 0x40

// PHY registers
#define PHCON1 0x0
#define PHSTAT1 0x1
#define PHID1 0x3
#define PHID2 0x3
#define PHCON2 0x10
#define PHSTAT2 0x11
#define PHIE 0x12
#define PHIR 0x13
#define PHLCON 0x14

// PHYCON2: PHY Half-Duplex Loopback Disable
#define HDLDIS 0x0100

// PHSTAT2: Link status; 1 link is up; 0 link is down
#define LSTAT 0x400

// PHIE: Link up/down interrupt
#define PGEIE 0x2
#define PLNKIE 0x10

//! Execute the command
//!
//! @param dst memory for the result of the RCR and RBM commands, not used
//!            otherwise
//! @param src source buffer for the WBM command, not used otherwise
void enc28j60_execute_blocking(enc28j60 *eth, uint32_t cmd, void *dst,
                               const void *src);

//! Write a register
static inline void reg_bits_clear_blk(enc28j60 *eth, uint8_t addr, uint8_t data)
{
    uint32_t cmd = BFC_CMD(addr, data);
    enc28j60_execute_blocking(eth, cmd, NULL, NULL);
}

//! Write a register
static inline void reg_bits_set_blk(enc28j60 *eth, uint8_t addr, uint8_t data)
{
    uint32_t cmd = BFS_CMD(addr, data);
    enc28j60_execute_blocking(eth, cmd, NULL, NULL);
}

//! Set the active register bank
static inline void bank_set_blk(enc28j60 *eth, uint8_t bank)
{
    // Section 3.1 of the manual
    // Clearing BSEL1:BSEL0 in ECON1
    reg_bits_clear_blk(eth, ECON1, 0x3);
    reg_bits_set_blk(eth, ECON1, bank & 0x3);
}

//! Read an ETH register
static inline uint8_t reg_read_e_blk(enc28j60 *eth, uint8_t addr)
{
    uint32_t result = 0;
    uint32_t cmd = RCR_E_CMD(addr);
    enc28j60_execute_blocking(eth, cmd, (uint8_t *)&result, NULL);
    return result;
}

//! Read a MAC or MII register
static inline uint8_t reg_read_m_blk(enc28j60 *eth, uint8_t addr)
{
    // According to the section 4.2.1 of the manual the first received byte
    // for these reads is a dummy.
    uint32_t result = 0;
    uint32_t cmd = RCR_M_CMD(addr);
    enc28j60_execute_blocking(eth, cmd, (uint8_t *)&result, NULL);
    return result >> 8;
}

//! Write a register
static inline void reg_write_blk(enc28j60 *eth, uint8_t addr, uint8_t data)
{
    uint32_t cmd = WCR_CMD(addr, data);
    enc28j60_execute_blocking(eth, cmd, NULL, NULL);
}

//! Read a PHY register; changes the selected bank
static inline uint16_t reg_read_p_blk(enc28j60 *eth, uint8_t addr)
{
    // Initiate the readout operation
    bank_set_blk(eth, 2);
    reg_write_blk(eth, MIREGADR, addr);
    reg_write_blk(eth, MICMD, MIIRD);

    // Wait for MAC to fetch the values from the PHY
    bank_set_blk(eth, 3);
    while (reg_read_m_blk(eth, MISTAT) & BUSY)
        ;

    // Clear the command and read out the data
    bank_set_blk(eth, 2);
    reg_write_blk(eth, MICMD, 0);

    uint16_t result = reg_read_m_blk(eth, MIRDH);
    result <<= 8;
    result |= reg_read_m_blk(eth, MIRDL);
    return result;
}

//! Write a PHY register; changes the selected bank
static inline void reg_write_p_blk(enc28j60 *eth, uint8_t addr, uint16_t data)
{
    // Initiate the readout operation
    bank_set_blk(eth, 2);
    reg_write_blk(eth, MIREGADR, addr);

    // Writing the higher byte automatically triggers the transaction
    reg_write_blk(eth, MIWRL, data);
    reg_write_blk(eth, MIWRH, data >> 8);

    // Wait for MAC to write the values to the PHY
    bank_set_blk(eth, 3);
    while (reg_read_m_blk(eth, MISTAT) & BUSY)
        ;
}

//! Write memory; changes the selected bank
static inline void mem_write_blk(enc28j60 *eth, uint8_t size, const void *data)
{
    uint32_t cmd = WBM_CMD(size);
    enc28j60_execute_blocking(eth, cmd, NULL, data);
}

//! Write memory; changes the selected bank
static inline void mem_read_blk(enc28j60 *eth, uint8_t size, void *data)
{
    uint32_t cmd = RBM_CMD(size);
    enc28j60_execute_blocking(eth, cmd, data, NULL);
}

//! Write memory; changes the selected bank
static inline void mem_write_at_blk(enc28j60 *eth, uint16_t addr, uint8_t size,
                                    const void *data)
{
    bank_set_blk(eth, 0);
    reg_write_blk(eth, EWRPTL, addr);
    reg_write_blk(eth, EWRPTH, addr >> 8);
    mem_write_blk(eth, size, data);
}

//! Write memory; changes the selected bank
static inline void mem_read_at_blk(enc28j60 *eth, uint16_t addr, uint8_t size,
                                   void *data)
{
    bank_set_blk(eth, 0);
    reg_write_blk(eth, ERDPTL, addr);
    reg_write_blk(eth, ERDPTH, addr >> 8);
    mem_read_blk(eth, size, data);
}

//! Reset the device
static inline void system_reset(enc28j60 *eth)
{
    uint32_t cmd = SRC_CMD();
    enc28j60_execute_blocking(eth, cmd, NULL, NULL);
}

#define RBM_MAX 31
#define WBM_MAX 30

//! Initialize the command buffer
void enc28j60_cmd_buf_init(enc28j60 *eth);

//! Encode a command in the transmission buffer.
//!
//! If RBM commands are issued, no other read commands may be issued and the
//! rx_ptr data member of the command buffer must point to the memory where
//! the received data will be stored.
//!
//! @param src source buffer for the WBM command, not used otherwise
bool enc28j60_cmd_buf_encode_cmd(enc28j60 *eth, uint32_t cmd, const void *src);

//! Decode the results of control register reads
bool enc28j60_cmd_buf_decode_rcr(enc28j60 *eth, bool is_eth, uint8_t *val);

//! Reset the state of the buffer
void enc28j60_cmd_buf_reset(enc28j60 *eth);

//! Execute the command buffer
void enc28j60_cmd_buf_execute(enc28j60 *eth);

//! Set pointer to the rx buffer
void enc28j60_cmd_buf_set_rx_ptr(enc28j60 *eth, size_t size, void *rbuf);
