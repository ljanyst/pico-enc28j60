// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#pragma once

#include "pico/enc28j60_driver.h"

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

#define SH(VAL, N) (((uint32_t)(VAL)) << N)

// The PIO driver can receive up to 32 bytes with one command and send up to
// 31 bytes, the sizes are in bits and must be multiples of 8bits, ie. we only
// transmit th whole bytes
#define CMD(TXSZ, RXSZ, OPCODE, ADDR, PAYLOAD) \
    (SH((TXSZ + 8), 24) | SH(RXSZ, 16) | SH(OPCODE, 13) | SH(ADDR, 8) | PAYLOAD)

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
#define ERDPTL 0x00
#define ERDPTH 0x01
#define EWRPTL 0x02
#define EWRPTH 0x03
#define ETXSTL 0x04
#define ETXSTH 0x05
#define ETXNDL 0x06
#define ETXNDH 0x07
#define ERXSTL 0x08
#define ERXSTH 0x09
#define ERXNDL 0x0a
#define ERXNDH 0x0b
#define ERXRDPTL 0x0c
#define ERXRDPTH 0x0d
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

//! Execute the command
//!
//! @param dst memory for the result of the RCR and RBM commands, not used
//!            otherwise
//! @param src source buffer for the WBM command, not used otherwise
void enc28j60_execute_blocking(enc28j60 *eth, uint32_t cmd, uint8_t *dst,
                               uint8_t *src);

//! Set the active register bank
static inline void bank_set_blk(enc28j60 *eth, uint8_t bank)
{
    // Section 3.1 of the manual
    // Clearing BSEL1:BSEL0 in ECON1
    bank &= 0x3;
    uint32_t cmd = BFC_CMD(ECON1, bank);
    enc28j60_execute_blocking(eth, cmd, NULL, NULL);
    // Setting the above to the desired value
    cmd = BFS_CMD(ECON1, bank);
    enc28j60_execute_blocking(eth, cmd, NULL, NULL);
}

//! Read an ETH register
static inline uint8_t reg_read_e_blk(enc28j60 *eth, uint8_t addr)
{
    uint32_t result = 0;
    uint32_t cmd = RCR_E_CMD(EREVID);
    enc28j60_execute_blocking(eth, cmd, (uint8_t *)&result, NULL);
    return result;
}

//! Read a MAC or MII register
static inline uint8_t reg_read_m_blk(enc28j60 *eth, uint8_t addr)
{
    // According to the section 4.2.1 of the manual the first received byte
    // for these reads is a dummy.
    uint32_t result = 0;
    uint32_t cmd = RCR_M_CMD(EREVID);
    enc28j60_execute_blocking(eth, cmd, (uint8_t *)&result, NULL);
    return result >> 8;
}
