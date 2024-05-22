// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#pragma once

#include "pico/enc28j60-driver.h"

#include <FreeRTOS_IP.h>

typedef struct {
    struct xNetworkInterface iface;
    bool initialized;
} enc28j60_freertos_interface;

//! The initialize function expects an enc28j60_config in the pvArgument field
//! of the descriptor
BaseType_t enc28j60_freertos_initialize(struct xNetworkInterface *pxDescriptor);

//! The output function assumes that it's a zero-copy function and gets to
//! release the buffers it's handed
BaseType_t
enc28j60_freertos_output_frame(struct xNetworkInterface *pxDescriptor,
                               NetworkBufferDescriptor_t *const pxNetworkBuffer,
                               BaseType_t xReleaseAfterSend);

//! Returns the cached PHY status because querying the PHY regs is too
//! expensive. The cached status is updated following an interrupt.
BaseType_t
enc28j60_freertos_get_phy_link_status(NetworkInterface_t *pxInterface);
