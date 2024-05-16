// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#include "pico/enc28j60-freertos-driver.h"
#include "projdefs.h"

BaseType_t enc28j60_freertos_initialize(struct xNetworkInterface *pxDescriptor)
{
    return pdFALSE;
}

BaseType_t
enc28j60_freertos_output_frame(struct xNetworkInterface *pxDescriptor,
                               NetworkBufferDescriptor_t *const pxNetworkBuffer,
                               BaseType_t xReleaseAfterSend)
{
    return pdFALSE;
}

BaseType_t
enc28j60_freertos_get_phy_link_status(NetworkInterface_t *pxInterface)
{
    return pdFALSE;
}
