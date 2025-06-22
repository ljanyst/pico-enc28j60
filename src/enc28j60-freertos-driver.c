// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#include "pico/enc28j60-freertos-driver.h"
#include "pico/enc28j60-driver.h"

#include <FreeRTOS.h>
#include <FreeRTOS_IP.h>

#include <stdint.h>
#include <stdlib.h>

#if configTASK_NOTIFICATION_ARRAY_ENTRIES < 2
#error "At least 2 task notifications are required for the driver to work"
#endif

#define TX_QUEUE_LENGTH 10

typedef struct {
    enc28j60 *drv;
    NetworkInterface_t *iface;
    QueueHandle_t tx_queue;
    TaskHandle_t task;
    bool link_up;
    bool initialized;
} driver_data;

#define IRQ_NOTIFICATION 0x1
#define TX_NOTIFICATION 0x2

static void __time_critical_func(irq_handler)(void *data)
{
    driver_data *dd = data;
    xTaskNotifyIndexedFromISR(dd->task, 0, IRQ_NOTIFICATION, eSetBits, NULL);
}

static NetworkBufferDescriptor_t *select_frame(driver_data *dd, bool want_frame)
{
    // Check if we can return a frame right away
    if (want_frame) {
        NetworkBufferDescriptor_t *frame;
        if (xQueueReceive(dd->tx_queue, &frame, 0) == pdPASS) {
            return frame;
        }
    }

    while (true) {
        uint32_t val;
        BaseType_t ret = xTaskNotifyWaitIndexed(0, 0, 0xffffffff, &val, 1000);
        if (ret == pdFAIL) {
            continue;
        }

        if (val & IRQ_NOTIFICATION) {
            return NULL;
        }

        if (want_frame && (val & TX_NOTIFICATION)) {
            NetworkBufferDescriptor_t *frame;
            if (xQueueReceive(dd->tx_queue, &frame, 0) == pdPASS) {
                return frame;
            }
        }
    }
}

static void rx_frame(driver_data *dd)
{
    uint32_t rx_info = enc28j60_frame_rx_info(dd->drv);
    size_t size = ENC28J60_RX_SIZE(rx_info);
    NetworkBufferDescriptor_t *desc;
    desc = pxGetNetworkBufferWithDescriptor(size, 0);

    // No network buffer available, drop the frame
    if (!desc) {
        enc28j60_frame_discard(dd->drv, rx_info);
        return;
    }

    desc->xDataLength = size;
    enc28j60_frame_rx(dd->drv, rx_info, desc->pucEthernetBuffer);

    // The stack is not interested in the frame
    if (eConsiderFrameForProcessing(desc->pucEthernetBuffer) !=
        eProcessBuffer) {
        vReleaseNetworkBufferAndDescriptor(desc);
        return;
    }

    desc->pxEndPoint =
        FreeRTOS_MatchingEndpoint(dd->iface, desc->pucEthernetBuffer);

    // No matching endpoint
    if (desc->pxEndPoint == NULL) {
        vReleaseNetworkBufferAndDescriptor(desc);
        return;
    }

    desc->pxInterface = dd->iface;
    IPStackEvent_t rx_event;
    rx_event.eEventType = eNetworkRxEvent;
    rx_event.pvData = desc;

    // the frame could not be set without delay
    if (xSendEventStructToIPTask(&rx_event, 0) == pdFALSE) {
        vReleaseNetworkBufferAndDescriptor(desc);
        return;
    }
}

static void driver_task(void *params)
{
    driver_data *dd = params;
    NetworkBufferDescriptor_t *tx_frame = NULL;
    uint32_t tx_frame_id = ENC28J60_INVALID_FRAME_ID;
    uint32_t in_progress_frame_id = ENC28J60_INVALID_FRAME_ID;

    while (1) {
        bool rx_ready = false;
        NetworkBufferDescriptor_t *tx_frame_new = NULL;
        tx_frame_new = select_frame(dd, tx_frame == NULL && dd->link_up);

        // If we didn't get a frame here, we were woken up because of an IRQ
        if (!tx_frame_new) {
            uint8_t flags = enc28j60_irq_flags(dd->drv);
            if (enc28j60_irq_is_link(flags)) {
                // These events are exceptional, so we don't care if we block
                // the CPU
                dd->link_up = enc28j60_link_status_blk(dd->drv);

                // We trigger this on any link status change because there is no
                // way to signal that link is up. Re-initialization begins
                // immediately after a link down event.
                FreeRTOS_NetworkDown(dd->iface);
            }

            if (enc28j60_irq_is_tx(flags)) {
                enc28j60_frame_tx_confirm(dd->drv, in_progress_frame_id);
                in_progress_frame_id = ENC28J60_INVALID_FRAME_ID;
            }

            if (enc28j60_irq_is_rx(flags)) {
                rx_frame(dd);
            }

            // The receive buffer is clogged, drain it
            if (enc28j60_irq_is_rx_err(flags)) {
                uint8_t count = enc28j60_rx_count(dd->drv);
                for (int i = 0; i < count; ++i) {
                    rx_frame(dd);
                }
            }

            enc28j60_irq_ack(dd->drv, flags);
        }

        // We have a frame a new frame
        if (tx_frame_new) {
            tx_frame = tx_frame_new;
        }

        // We have a frame to upload
        if (tx_frame && tx_frame_id == ENC28J60_INVALID_FRAME_ID) {
            uint32_t id = enc28j60_frame_upload(dd->drv, tx_frame->xDataLength,
                                                tx_frame->pucEthernetBuffer);
            if (id != ENC28J60_INVALID_FRAME_ID) {
                tx_frame_id = id;
            }
        }

        // We have a frame uploaded waiting to be transmitted, and there is no
        // transimssion going on
        if (tx_frame_id != ENC28J60_INVALID_FRAME_ID &&
            in_progress_frame_id == ENC28J60_INVALID_FRAME_ID) {
            enc28j60_frame_tx(dd->drv, tx_frame_id);
            vReleaseNetworkBufferAndDescriptor(tx_frame);
            tx_frame = NULL;
            in_progress_frame_id = tx_frame_id;
            tx_frame_id = ENC28J60_INVALID_FRAME_ID;
        }
    }
}

static void task_wait(void *arg)
{
    while (xTaskNotifyWaitIndexed(1, 0, 0, NULL, 10) == pdFALSE)
        ;
}

static void task_notify(void *arg)
{
    xTaskNotifyIndexedFromISR(arg, 1, 0, eNoAction, NULL);
}

BaseType_t enc28j60_freertos_initialize(struct xNetworkInterface *pxDescriptor)
{
    // Initialization is called again after the link down signal, so we need
    // this hack to avoid it.
    enc28j60_freertos_interface *iface =
        (enc28j60_freertos_interface *)pxDescriptor;
    if (iface->initialized) {
        return pdTRUE;
    }
    iface->initialized = true;

    driver_data *dd = calloc(1, sizeof(driver_data));
    if (dd == NULL) {
        panic("Failed to allocate memory for ENC28J60 FreeRTOS driver data");
    }

    dd->tx_queue =
        xQueueCreate(TX_QUEUE_LENGTH, sizeof(NetworkBufferDescriptor_t *));
    if (dd->tx_queue == NULL) {
        panic("Failed to allocate memory for ENC28J60 FreeRTOS tx queue");
    }

    char *task_name = malloc(32);
    snprintf(task_name, 32, "ENC28J60 task for %s", pxDescriptor->pcName);
    xTaskCreate(driver_task, task_name, 1024, dd, 4, &dd->task);

    enc28j60_config *eth_cfg = pxDescriptor->pvArgument;
    eth_cfg->irq_cb = irq_handler;
    eth_cfg->irq_data = dd;

    eth_cfg->notify.data = dd->task;
    eth_cfg->notify.notify = task_notify;
    eth_cfg->notify.wait = task_wait;

    dd->drv = enc28j60_init(eth_cfg);
    pxDescriptor->pvArgument = dd;
    dd->iface = pxDescriptor;

    return pdTRUE;
}

BaseType_t
enc28j60_freertos_output_frame(struct xNetworkInterface *pxDescriptor,
                               NetworkBufferDescriptor_t *const pxNetworkBuffer,
                               BaseType_t xReleaseAfterSend)
{
    // The return value of this function is always ignored
    driver_data *dd = pxDescriptor->pvArgument;

    if (xQueueSend(dd->tx_queue, &pxNetworkBuffer, 0) != pdPASS) {
        vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
        return pdTRUE;
    }

    xTaskNotifyIndexed(dd->task, 0, TX_NOTIFICATION, eSetBits);
    return pdTRUE;
}

BaseType_t
enc28j60_freertos_get_phy_link_status(NetworkInterface_t *pxInterface)
{
    driver_data *dd = pxInterface->pvArgument;
    return dd->link_up;
}
