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

#include "los_base.h"
#include "los_config.h"
#include "sys/mount.h"
#include "mtd_partition.h"
#include "console.h"
#include "hal_timer.h"
#ifdef LOSCFG_DRIVERS_HDF
#include "devmgr_service_start.h"
#endif

static void sun8i_board_init(void)
{
    uint32_t val;
    READ_UINT32(val, IO_DEVICE_ADDR(0x01c202c0));
    val |= BIT(17) | BIT(20);
    WRITE_UINT32(val, IO_DEVICE_ADDR(0x01c202c0));
    READ_UINT32(val, IO_DEVICE_ADDR(0x01c20060));
    val |= BIT(17) | BIT(20);
    WRITE_UINT32(val, IO_DEVICE_ADDR(0x01c20060));

    READ_UINT32(val, IO_DEVICE_ADDR(0x01c202d8));
    val |= BIT(0);
    WRITE_UINT32(val, IO_DEVICE_ADDR(0x01c202d8));

    READ_UINT32(val, IO_DEVICE_ADDR(0x01c2006c)); 
    val |= BIT(0);
    WRITE_UINT32(val, IO_DEVICE_ADDR(0x01c2006c));
    WRITE_UINT32(0x80000000, IO_DEVICE_ADDR(0x01c200a0));

    READ_UINT32(val, IO_DEVICE_ADDR(0x01c20824));
    val &= ~(7 << 24);
    val &= ~(7 << 28);
    val |= (2 <<  24) | (2 << 28);
    WRITE_UINT32(val, IO_DEVICE_ADDR(0x01c20824));
}

void SystemInit(void)
{
    sun8i_board_init();

#ifdef LOSCFG_DRIVERS_HDF
    if (DeviceManagerStart()) {
        PRINT_ERR("No drivers need load by hdf manager!");
    }
#endif

    if (virtual_serial_init("/dev/uartdev-0") != 0)
    {
        PRINT_ERR("virtual_serial_init failed");
    }

    if (system_console_init(SERIAL) != 0)
    {
        PRINT_ERR("system_console_init failed\n");
    }
}

