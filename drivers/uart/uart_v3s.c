/*
 * Copyright (c) 2020-2021 Huawei Device Co., Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 *    conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 *    of conditions and the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "los_event.h"
#include "device_resource_if.h"
#include "hdf_base.h"
#include "hdf_log.h"
#include "osal_io.h"
#include "osal_mem.h"
#include "osal_time.h"
#include "uart_core.h"
#include "uart_dev.h"
#include "uart_if.h"
#include "uart_dw.h"

#define HDF_LOG_TAG uart_v3s

static int32_t V3sRead(struct UartHost *host, uint8_t *data, uint32_t size)
{
    int32_t ret;
    struct uart_driver_data *udd = NULL;

    if (host == NULL || host->priv == NULL) {
        HDF_LOGE("%s: invalid parameter\n", __func__);
        return HDF_ERR_INVALID_PARAM;
    }
    udd = (struct uart_driver_data *)host->priv;

    if (udd == NULL) {
        uart_error("uart_driver_data is NULL\n");
        return -EINVAL;
    }
    if (UART_STATE_USEABLE != udd->state) {
        return -EFAULT;
    }

    if ((udd->flags & UART_FLG_RD_BLOCK) &&
        (uart_rx_buf_empty(udd))) {
        (void)LOS_EventRead(&udd->wait.stEvent,
                            0x1, LOS_WAITMODE_OR, LOS_WAIT_FOREVER);
    }

    ret = uart_dev_read(udd, (char *)data, size);

    if ((udd->flags & UART_FLG_RD_BLOCK) &&
        (uart_rx_buf_empty(udd))) {
        (void)LOS_EventClear(&udd->wait.stEvent, ~(0x1));
    }
    return ret;
}

static int32_t V3sWrite(struct UartHost *host, uint8_t *data, uint32_t size)
{
    int32_t ret;
    struct uart_driver_data *udd = NULL;

    if (host == NULL || host->priv == NULL) {
        HDF_LOGE("%s: invalid parameter\n", __func__);
        return HDF_ERR_INVALID_PARAM;
    }
    udd = (struct uart_driver_data *)host->priv;
    if (udd->state != UART_STATE_USEABLE) {
        return HDF_FAILURE;
    }
    if (udd->ops->start_tx != NULL) {
        ret = udd->ops->start_tx(udd, (char *)data, size);
    } else {
        ret = HDF_ERR_NOT_SUPPORT;
        HDF_LOGE("%s: not support\n", __func__);
    }
    return ret;
}

static int32_t V3sGetBaud(struct UartHost *host, uint32_t *baudRate)
{
    struct uart_driver_data *udd = NULL;

    if (host == NULL || host->priv == NULL || baudRate == NULL) {
        HDF_LOGE("%s: invalid parameter\n", __func__);
        return HDF_ERR_INVALID_PARAM;
    }

    udd = (struct uart_driver_data *)host->priv;
    if (udd->state != UART_STATE_USEABLE) {
        return HDF_FAILURE;
    }
    *baudRate = udd->baudrate;
    return HDF_SUCCESS;
}

static int32_t V3sSetBaud(struct UartHost *host, uint32_t baudRate)
{
    struct uart_driver_data *udd = NULL;

    if (host == NULL || host->priv == NULL) {
        HDF_LOGE("%s: invalid parameter\n", __func__);
        return HDF_ERR_INVALID_PARAM;
    }

    udd = (struct uart_driver_data *)host->priv;
    if (udd->state != UART_STATE_USEABLE) {
        return HDF_FAILURE;
    }
    if ((baudRate > 0) && (baudRate <= CONFIG_MAX_BAUDRATE)) {
        udd->baudrate = baudRate;
        if (udd->ops->config == NULL) {
            HDF_LOGE("%s: not support\n", __func__);
            return HDF_ERR_NOT_SUPPORT;
        }
        if (udd->ops->config(udd) != HDF_SUCCESS) {
            HDF_LOGE("%s: config baudrate %d failed\n", __func__, baudRate);
            return HDF_FAILURE;
        }
    } else {
        HDF_LOGE("%s: invalid baudrate, which is:%d\n", __func__, baudRate);
        return HDF_FAILURE;
    }
    return HDF_SUCCESS;
}

static int32_t V3sGetAttribute(struct UartHost *host, struct UartAttribute *attribute)
{
    struct uart_driver_data *udd = NULL;

    if (host == NULL || host->priv == NULL || attribute == NULL) {
        HDF_LOGE("%s: invalid parameter\n", __func__);
        return HDF_ERR_INVALID_PARAM;
    }
    udd = (struct uart_driver_data *)host->priv;
    if (udd->state != UART_STATE_USEABLE) {
        return HDF_FAILURE;
    }
    attribute->cts = udd->attr.cts;
    attribute->dataBits = udd->attr.data_bits;
    attribute->fifoRxEn = udd->attr.fifo_rx_en;
    attribute->fifoTxEn = udd->attr.fifo_tx_en;
    attribute->parity = udd->attr.parity;
    attribute->rts = udd->attr.rts;
    attribute->stopBits = udd->attr.stop_bits;
    return HDF_SUCCESS;
}

static int32_t V3sSetAttribute(struct UartHost *host, struct UartAttribute *attribute)
{
    struct uart_driver_data *udd = NULL;

    if (host == NULL || host->priv == NULL || attribute == NULL) {
        HDF_LOGE("%s: invalid parameter\n", __func__);
        return HDF_ERR_INVALID_PARAM;
    }
    udd = (struct uart_driver_data *)host->priv;
    if (udd->state != UART_STATE_USEABLE) {
        return HDF_FAILURE;
    }
    udd->attr.cts = attribute->cts;
    udd->attr.data_bits = attribute->dataBits;
    udd->attr.fifo_rx_en = attribute->fifoRxEn;
    udd->attr.fifo_tx_en = attribute->fifoTxEn;
    udd->attr.parity = attribute->parity;
    udd->attr.rts = attribute->rts;
    udd->attr.stop_bits = attribute->stopBits;
    if (udd->ops->config == NULL) {
        HDF_LOGE("%s: not support\n", __func__);
        return HDF_ERR_NOT_SUPPORT;
    }
    if (udd->ops->config(udd) != HDF_SUCCESS) {
        HDF_LOGE("%s: config failed\n", __func__);
        return HDF_FAILURE;
    }
    return HDF_SUCCESS;
}

static int32_t V3sSetTransMode(struct UartHost *host, enum UartTransMode mode)
{
    struct uart_driver_data *udd = NULL;

    if (host == NULL || host->priv == NULL) {
        HDF_LOGE("%s: invalid parameter\n", __func__);
        return HDF_ERR_INVALID_PARAM;
    }
    udd = (struct uart_driver_data *)host->priv;
    if (udd->state != UART_STATE_USEABLE) {
        return HDF_FAILURE;
    }
    if (mode == UART_MODE_RD_BLOCK) {
        udd->flags |= UART_FLG_RD_BLOCK;
    } else if (mode == UART_MODE_RD_NONBLOCK) {
        udd->flags &= ~UART_FLG_RD_BLOCK;
        (void)LOS_EventWrite(&udd->wait.stEvent, 0x1);
    }
    return HDF_SUCCESS;
}

static int32_t V3sInit(struct UartHost *host)
{
    int32_t ret = 0;
    struct uart_driver_data *udd = NULL;
    struct wait_queue_head *wait = NULL;
    if (host == NULL || host->priv == NULL) {
        HDF_LOGE("%s: invalid parameter\n", __func__);
        return HDF_ERR_INVALID_PARAM;
    }

    udd = (struct uart_driver_data *)host->priv;
    wait = &udd->wait;
    if (udd->state == UART_STATE_NOT_OPENED) {
        udd->state = UART_STATE_OPENING;
        (void)LOS_EventInit(&wait->stEvent);
        spin_lock_init(&wait->lock);
        LOS_ListInit(&wait->poll_queue);
        udd->rx_transfer = (struct uart_ioc_transfer *)OsalMemCalloc(sizeof(struct uart_ioc_transfer));
        if (udd->rx_transfer == NULL) {
            HDF_LOGE("%s: alloc transfer failed\n", __func__);
            return HDF_ERR_MALLOC_FAIL;
        }
        if (udd->ops->startup == NULL) {
            HDF_LOGE("%s: not support\n", __func__);
            ret = HDF_ERR_NOT_SUPPORT;
            goto FREE_TRANSFER;
        }
        if (udd->ops->startup(udd) != HDF_SUCCESS) {
            HDF_LOGE("%s: startup failed\n", __func__);
            ret = HDF_FAILURE;
            goto FREE_TRANSFER;
        }
    }
    udd->state = UART_STATE_USEABLE;
    udd->count++;
    return HDF_SUCCESS;

FREE_TRANSFER:
    (void)OsalMemFree(udd->rx_transfer);
    udd->rx_transfer = NULL;
    return ret;
}

static int32_t V3sDeinit(struct UartHost *host)
{
    struct wait_queue_head *wait = NULL;
    struct uart_driver_data *udd = NULL;
    if (host == NULL || host->priv == NULL) {
        HDF_LOGE("%s: invalid parameter\n", __func__);
        return HDF_ERR_INVALID_PARAM;
    }

    udd = (struct uart_driver_data *)host->priv;
    if ((--udd->count) != 0) {
        return HDF_SUCCESS;
    }
    wait = &udd->wait;
    if (udd->flags & UART_FLG_DMA_RX) {
        if (udd->ops->dma_shutdown != NULL) {
            udd->ops->dma_shutdown(udd, UART_DMA_DIR_RX);
        }
    }
    if (udd->flags & UART_FLG_DMA_TX) {
        if (udd->ops->dma_shutdown != NULL) {
            udd->ops->dma_shutdown(udd, UART_DMA_DIR_TX);
        }
    }
    LOS_ListDelete(&wait->poll_queue);
    LOS_EventDestroy(&wait->stEvent);
    if (udd->ops->shutdown != NULL) {
        udd->ops->shutdown(udd);
    }
    if (udd->rx_transfer != NULL) {
        (void)OsalMemFree(udd->rx_transfer);
        udd->rx_transfer = NULL;
    }
    udd->state = UART_STATE_NOT_OPENED;
    return HDF_SUCCESS;
}

struct UartHostMethod g_uartHostMethod = {
    .Init = V3sInit,
    .Deinit = V3sDeinit,
    .Read = V3sRead,
    .Write = V3sWrite,
    .SetBaud = V3sSetBaud,
    .GetBaud = V3sGetBaud,
    .SetAttribute = V3sSetAttribute,
    .GetAttribute = V3sGetAttribute,
    .SetTransMode = V3sSetTransMode,
};

static int32_t UartGetConfigFromHcs(struct dw_port *port, const struct DeviceResourceNode *node)
{
    uint32_t tmp, regPbase, iomemCount;
    struct uart_driver_data *udd = port->udd;
    struct DeviceResourceIface *iface = DeviceResourceGetIfaceInstance(HDF_CONFIG_SOURCE);

    if (iface == NULL || iface->GetUint32 == NULL) {
        HDF_LOGE("%s: face is invalid\n", __func__);
        return HDF_FAILURE;
    }
    if (iface->GetUint32(node, "num", &udd->num, 0) != HDF_SUCCESS) {
        HDF_LOGE("%s: read busNum fail\n", __func__);
        return HDF_FAILURE;
    }
    if (iface->GetUint32(node, "baudrate", &udd->baudrate, 0) != HDF_SUCCESS) {
        HDF_LOGE("%s: read numCs fail\n", __func__);
        return HDF_FAILURE;
    }
    if (iface->GetUint32(node, "fifoRxEn", &tmp, 0) != HDF_SUCCESS) {
        HDF_LOGE("%s: read speed fail\n", __func__);
        return HDF_FAILURE;
    }
    udd->attr.fifo_rx_en = tmp;
    if (iface->GetUint32(node, "fifoTxEn", &tmp, 0) != HDF_SUCCESS) {
        HDF_LOGE("%s: read fifoSize fail\n", __func__);
        return HDF_FAILURE;
    }
    udd->attr.fifo_tx_en = tmp;
    if (iface->GetUint32(node, "flags", &udd->flags, 0) != HDF_SUCCESS) {
        HDF_LOGE("%s: read clkRate fail\n", __func__);
        return HDF_FAILURE;
    }
    if (iface->GetUint32(node, "regPbase", &regPbase, 0) != HDF_SUCCESS) {
        HDF_LOGE("%s: read mode fail\n", __func__);
        return HDF_FAILURE;
    }
    if (iface->GetUint32(node, "iomemCount", &iomemCount, 0) != HDF_SUCCESS) {
        HDF_LOGE("%s: read bitsPerWord fail\n", __func__);
        return HDF_FAILURE;
    }
    port->phys_base = (unsigned long)OsalIoRemap(regPbase, iomemCount);
    if (iface->GetUint32(node, "interrupt", &port->irq_num, 0) != HDF_SUCCESS) {
        HDF_LOGE("%s: read comMode fail\n", __func__);
        return HDF_FAILURE;
    }
    return 0;
}

static int32_t V3sAttach(struct UartHost *host, struct HdfDeviceObject *device)
{
    int32_t ret;
    struct uart_driver_data *udd = NULL;
    struct dw_port *port = NULL;

    if (device->property == NULL) {
        HDF_LOGE("%s: property is null\n", __func__);
        return HDF_FAILURE;
    }
    udd = (struct uart_driver_data *)OsalMemCalloc(sizeof(*udd));
    if (udd == NULL) {
        HDF_LOGE("%s: OsalMemCalloc udd error\n", __func__);
        return HDF_ERR_MALLOC_FAIL;
    }
    port = (struct dw_port *)OsalMemCalloc(sizeof(struct dw_port));
    if (port == NULL) {
        HDF_LOGE("%s: OsalMemCalloc port error\n", __func__);
        (void)OsalMemFree(udd);
        return HDF_ERR_MALLOC_FAIL;
    }
    udd->ops = dw_get_ops();
    udd->recv = uart_recv_notify;
    udd->count = 0;
    port->udd = udd;
    ret = UartGetConfigFromHcs(port, device->property);
    if (ret != 0 || port->phys_base == 0) {
        (void)OsalMemFree(port);
        (void)OsalMemFree(udd);
        return HDF_FAILURE;
    }
    udd->private = port;
    host->priv = udd;
    UartAddDev(host);
    return HDF_SUCCESS;
}

static void V3sDetach(struct UartHost *host)
{
    struct uart_driver_data *udd = NULL;
    struct dw_port *port = NULL;

    if (host->priv == NULL) {
        HDF_LOGE("%s: invalid parameter", __func__);
        return;
    }
    udd = host->priv;
    if (udd->state != UART_STATE_NOT_OPENED) {
        HDF_LOGE("%s: uart driver data state invalid", __func__);
        return;
    }
    UartRemoveDev(host);
    port = udd->private;
    if (port != NULL) {
        if (port->phys_base != 0) {
            OsalIoUnmap((void *)port->phys_base);
        }
        (void)OsalMemFree(port);
        udd->private = NULL;
    }
    (void)OsalMemFree(udd);
    host->priv = NULL;
}

static int32_t HdfUartDeviceBind(struct HdfDeviceObject *device)
{
    if (device == NULL) {
        return HDF_ERR_INVALID_OBJECT;
    }
    return (UartHostCreate(device) == NULL) ? HDF_FAILURE : HDF_SUCCESS;
}

int32_t HdfUartDeviceInit(struct HdfDeviceObject *device)
{
    int32_t ret;
    struct UartHost *host = NULL;

    if (device == NULL) {
        HDF_LOGE("%s: device is null\n", __func__);
        return HDF_ERR_INVALID_OBJECT;
    }
    host = UartHostFromDevice(device);
    if (host == NULL) {
        HDF_LOGE("%s: host is null\n", __func__);
        return HDF_FAILURE;
    }
    ret = V3sAttach(host, device);
    if (ret != HDF_SUCCESS) {
        HDF_LOGE("%s: attach error\n", __func__);
        return HDF_FAILURE;
    }
    host->method = &g_uartHostMethod;
    return ret;
}

void HdfUartDeviceRelease(struct HdfDeviceObject *device)
{
    struct UartHost *host = NULL;

    if (device == NULL) {
        HDF_LOGE("%s: device is null\n", __func__);
        return;
    }
    host = UartHostFromDevice(device);
    if (host == NULL) {
        HDF_LOGE("%s: host is null\n", __func__);
        return;
    }
    if (host->priv != NULL) {
        V3sDetach(host);
    }
    UartHostDestroy(host);
}

struct HdfDriverEntry g_hdfUartDevice = {
    .moduleVersion = 1,
    .moduleName = "HDF_PLATFORM_UART",
    .Bind = HdfUartDeviceBind,
    .Init = HdfUartDeviceInit,
    .Release = HdfUartDeviceRelease,
};

HDF_INIT(g_hdfUartDevice);
