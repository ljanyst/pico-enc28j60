// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#include "pico/enc28j60-freertos-driver.h"

#include <FreeRTOS.h>
#include <FreeRTOS_IP.h>
#include <pico/enc28j60-driver.h>

#include <stdint.h>
#include <stdlib.h>

#define TX_QUEUE_LENGTH 10

typedef struct {
    enc28j60 *drv;
    NetworkInterface_t *iface;
    QueueHandle_t tx_queue;
    QueueHandle_t irq_sem;
    QueueSetHandle_t queue_set;
    TaskHandle_t task;
    bool link_up;
    bool initialized;
} driver_data;

static void __time_critical_func(irq_handler)(void *data)
{
    driver_data *dd = data;
    xSemaphoreGiveFromISR(dd->irq_sem, NULL);
}

static bool want_frame(driver_data *dd)
{
    return false;
}

static NetworkBufferDescriptor_t *select(driver_data *dd)
{
    if (!want_frame(dd)) {
        while (xSemaphoreTake(dd->irq_sem, 10) == pdFALSE) {
            continue;
        }
        return NULL;
    }

    while (true) {
        QueueSetMemberHandle_t activated;
        activated = xQueueSelectFromSet(dd->queue_set, 10);

        if (activated == dd->tx_queue) {
            NetworkBufferDescriptor_t *frame;
            xQueueReceive(dd->tx_queue, &frame, 0);
            return frame;
        } else if (activated == dd->irq_sem) {
            xSemaphoreTake(dd->irq_sem, 0);
            return NULL;
        }
        // timeout
    }
}

static void handle_frame(driver_data *dd, NetworkBufferDescriptor_t *frame)
{
}

static void handle_irq(driver_data *dd)
{
    printf("---- got irq;\n");
    uint8_t flags = enc28j60_irq_flags(dd->drv);
    printf("---- flagx; %x\n", flags);
    if (enc28j60_irq_is_link(flags)) {
        dd->link_up = enc28j60_link_status_blk(dd->drv);
        // We trigger this on any link status change because there is no way to
        // signal that link is up. Re-initialization begins immediately after
        // a link down event.
        FreeRTOS_NetworkDown(dd->iface);
    }

    if (enc28j60_irq_is_tx(flags)) {
        printf("---- got tx irq;\n");
    }

    if (enc28j60_irq_is_rx(flags)) {
        printf("---- got rx irq;\n");
    }

    enc28j60_irq_ack(dd->drv, flags);
}

static void driver_task(void *params)
{
    driver_data *dd = params;
    while (1) {
        NetworkBufferDescriptor_t *frame = select(dd);
        if (frame) {
            handle_frame(dd, frame);
            continue;
        }
        handle_irq(dd);
    }
}

static void task_wait(void *arg)
{
    while (xTaskNotifyWait(0, 0, NULL, 10) == pdFALSE)
        ;
}

static void task_notify(void *arg)
{
    xTaskNotify(*(TaskHandle_t *)arg, 0, eNoAction);
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

    dd->irq_sem = xSemaphoreCreateBinary();
    if (dd->tx_queue == NULL) {
        panic("Failed to allocate memory for ENC28J60 FreeRTOS irq semaphore");
    }

    // +1 because there's going to be the queue and the semaphore in the set
    dd->queue_set = xQueueCreateSet(TX_QUEUE_LENGTH + 1);
    if (dd->tx_queue == NULL) {
        panic("Failed to allocate memory for ENC28J60 FreeRTOS queue set");
    }

    xQueueAddToSet(dd->queue_set, dd->irq_sem);
    xQueueAddToSet(dd->queue_set, dd->tx_queue);

    char *task_name = malloc(32);
    snprintf(task_name, 32, "ENC28J60 task for %s", pxDescriptor->pcName);
    xTaskCreate(driver_task, task_name, 1024, dd, 4, &dd->task);

    enc28j60_config *eth_cfg = pxDescriptor->pvArgument;
    eth_cfg->irq_cb = irq_handler;
    eth_cfg->irq_data = dd;

    eth_cfg->notify.data = &dd->task;
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
    assert(xReleaseAfterSend); // we use the zero copy mode
    driver_data *dd = pxDescriptor->pvArgument;

    if (!dd->link_up ||
        xQueueSend(dd->tx_queue, pxNetworkBuffer, 10) != pdPASS) {
        vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
    }

    // The return value of this function is always ignored
    return pdTRUE;
}

BaseType_t
enc28j60_freertos_get_phy_link_status(NetworkInterface_t *pxInterface)
{
    // This function is not supposed to be called
    panic("enc28j60_freertos_get_phy_link_status: Not implemented");
    return pdFALSE;
}
