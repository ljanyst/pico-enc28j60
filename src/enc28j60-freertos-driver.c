// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#include "pico/enc28j60-freertos-driver.h"
#include "pico/enc28j60-driver.h"

#include <FreeRTOS.h>
#include <FreeRTOS_IP.h>

#include <stdint.h>
#include <stdlib.h>

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

static NetworkBufferDescriptor_t *select(driver_data *dd, bool want_frame)
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

static void driver_task(void *params)
{
    driver_data *dd = params;
    bool rx_ready = false;
    NetworkBufferDescriptor_t *tx_frame = NULL;
    uint32_t tx_frame_id = INVALID_FRAME_ID;
    uint32_t in_progress_frame_id = INVALID_FRAME_ID;

    while (1) {
        NetworkBufferDescriptor_t *tx_frame_new = NULL;
        tx_frame_new = select(dd, tx_frame == NULL && dd->link_up);

        // If we didn't get a frame here, we were woken up because of an IRQ
        if (!tx_frame_new) {
            uint8_t flags = enc28j60_irq_flags(dd->drv);
            printf("---- irq flags; %x\n", flags);
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
                printf("---- got tx irq;\n");
                enc28j60_frame_tx_confirm(dd->drv, in_progress_frame_id);
                in_progress_frame_id = INVALID_FRAME_ID;
            }

            if (enc28j60_irq_is_rx(flags)) {
                printf("---- got rx irq;\n");
                rx_ready = true;
            }

            enc28j60_irq_ack(dd->drv, flags);
        }

        // We have a frame a new frame
        if (tx_frame_new) {
            tx_frame = tx_frame_new;
        }

        // We have a frame to upload
        if (tx_frame && tx_frame_id == INVALID_FRAME_ID) {
            uint32_t id = enc28j60_frame_upload(dd->drv, tx_frame->xDataLength,
                                                tx_frame->pucEthernetBuffer);
            if (id != INVALID_FRAME_ID) {
                tx_frame_id = id;
            }
        }

        // We have a frame uploaded waiting to be transmitted, and there is no
        // transimssion going on
        if (tx_frame_id != INVALID_FRAME_ID &&
            in_progress_frame_id == INVALID_FRAME_ID) {
            enc28j60_frame_tx(dd->drv, tx_frame_id);
            vReleaseNetworkBufferAndDescriptor(tx_frame);
            tx_frame = NULL;
            in_progress_frame_id = tx_frame_id;
            tx_frame_id = INVALID_FRAME_ID;
        }

        // We have a frame to receive
        if (rx_ready) {
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

    if (!dd->link_up ||
        xQueueSend(dd->tx_queue, &pxNetworkBuffer, 10) != pdPASS) {
        vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
        return pdTRUE;
    }

    xTaskNotifyIndexed(dd->task, 0, TX_NOTIFICATION, eSetBits);
    return pdTRUE;
}

BaseType_t
enc28j60_freertos_get_phy_link_status(NetworkInterface_t *pxInterface)
{
    // This function is not supposed to be called
    panic("enc28j60_freertos_get_phy_link_status: Not implemented");
    return pdFALSE;
}
