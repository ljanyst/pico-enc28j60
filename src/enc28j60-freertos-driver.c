// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Licensed under the MIT license, see the LICENSE file for details.

#include "pico/enc28j60-freertos-driver.h"

#include <FreeRTOS.h>
#include <pico/enc28j60-driver.h>

#include <stdint.h>
#include <stdlib.h>

typedef struct {
    enc28j60 *drv;
    QueueHandle_t tx_queue;
    QueueHandle_t irq_sem;
    TaskHandle_t task;
} driver_data;

static void __time_critical_func(irq_handler)(void *data)
{
    driver_data *dd = data;
    xSemaphoreGiveFromISR(dd->irq_sem, NULL);
}

static void driver_task(void *params)
{
    driver_data *dd = params;
    while (1) {
        uint8_t dev_status;
        if (xSemaphoreTake(dd->irq_sem, 10) == pdFALSE) {
            continue;
        }

        printf("---- got irq;\n");
        uint8_t flags = enc28j60_irq_flags(dd->drv);
        printf("---- flagx; %x\n", flags);
        if (enc28j60_irq_is_link(flags)) {
            printf("---- got link irq;\n");
        }
        if (enc28j60_irq_is_tx(flags)) {
            printf("---- got tx irq;\n");
        }
        if (enc28j60_irq_is_rx(flags)) {
            printf("---- got rx irq;\n");
        }
        enc28j60_irq_ack(dd->drv, flags);
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
    driver_data *dd = calloc(1, sizeof(driver_data));
    if (dd == NULL) {
        panic("Failed to allocate memory for ENC28J60 FreeRTOS driver data");
    }

    dd->tx_queue = xQueueCreate(5, sizeof(NetworkBufferDescriptor_t *));
    if (dd->tx_queue == NULL) {
        panic("Failed to allocate memory for ENC28J60 FreeRTOS tx queue");
    }

    dd->irq_sem = xSemaphoreCreateBinary();
    if (dd->tx_queue == NULL) {
        panic("Failed to allocate memory for ENC28J60 FreeRTOS irq semaphore");
    }

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

    return pdTRUE;
}

BaseType_t
enc28j60_freertos_output_frame(struct xNetworkInterface *pxDescriptor,
                               NetworkBufferDescriptor_t *const pxNetworkBuffer,
                               BaseType_t xReleaseAfterSend)
{
    printf("---- output frame\n");
    return pdFALSE;
}

BaseType_t
enc28j60_freertos_get_phy_link_status(NetworkInterface_t *pxInterface)
{
    // This function is not supposed to be called
    panic("enc28j60_freertos_get_phy_link_status: Not implemented");
    return pdFALSE;
}
