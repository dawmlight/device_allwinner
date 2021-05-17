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

#include "linux/delay.h"
#include "asm/io.h"

#include "los_typedef.h"
#include "los_task.h"
#include "los_base.h"
#include "los_event.h"
#include "errno.h"

#include "linux/interrupt.h"
#include "linux/kernel.h"
#include "linux/spinlock.h"
#include "uart_dw.h"
#include <sys/bus.h>

#include "uart_dev.h"
#include "string.h"

#include "los_magickey.h"

#define UART_DR                     0x0
#define UART_IIR                    0x08
#define UART_LSR                    0x14
#define UART_USR                    0x7c
#define UART_LCR                    0x0c
#define UART_RBR                    0x00
#define UART_MCR                    0x10
#define UART_IER                    0x04

#define UART_LSR_DR                 BIT(0)
#define UART_LSR_OE                 BIT(1)
#define UART_LSR_PE                 BIT(2)
#define UART_LSR_FE                 BIT(3)
#define UART_LSR_BI                 BIT(4)
#define UART_LSR_THRE               BIT(5)
#define UART_LSR_TEMT               BIT(6)
#define UART_LSR_RXFIFOE            BIT(7)
#define UART_LSR_BRK_ERROR_BITS     0x1e

#ifdef LOSCFG_QUICK_START
__attribute__ ((section(".data"))) UINT32 g_uart_fputc_en = 0;
#else
__attribute__ ((section(".data"))) UINT32 g_uart_fputc_en = 1;
#endif

LITE_OS_SEC_BSS STATIC SPIN_LOCK_INIT(g_uartOutputSpin);

char uart_putc (char c)
{
    /* Wait until THRE is empyt */
    while (!(GET_UINT32(UART_REG_BASE + UART_USR) & 0x02)); /*lint !e40*/
    /* send one char */
    WRITE_UINT8(c, UART_REG_BASE + UART_DR);
    return c;
}

UINTPTR uart_to_ptr(UINTPTR n)
{
    (VOID)n;
    return UART_REG_BASE;
}

VOID UartPutStr(UINTPTR base, const CHAR *s, UINT32 len)
{
    UINT32 i;

    for (i = 0; i < len; i++) {
        if (*(s + i) == '\n') {
            uart_putc(*"\r");
        }
        uart_putc(*(s + i));
    }
}

UINT32 UartPutsReg(UINTPTR base, const CHAR *s, UINT32 len, BOOL isLock)
{
    UINT32 intSave;

    if (isLock) {
        LOS_SpinLockSave(&g_uartOutputSpin, &intSave);
        UartPutStr(base, s, len);
        LOS_SpinUnlockRestore(&g_uartOutputSpin, intSave);
    } else {
        UartPutStr(base, s, len);
    }

    return len;
}

VOID UartPuts(const CHAR *s, UINT32 len, BOOL isLock)
{
    UINTPTR base = uart_to_ptr(0);
    UartPutsReg(base, s, len, isLock);
}

#define FIFO_SIZE    128

static irqreturn_t dw_irq(int irq, void *data)
{
    char buf[FIFO_SIZE];
    unsigned int count = 0;
    struct dw_port *port = NULL;
    struct uart_driver_data *udd = (struct uart_driver_data *)data;
    unsigned int iir, lsr;
    int max_count = 256;
    unsigned char ch = 0;

    if (udd == NULL) {
        uart_error("udd is null!\n");
        return IRQ_HANDLED;
    }
    port = (struct dw_port *)udd->private;

    iir = readl(port->phys_base + UART_IIR);
    lsr = readl(port->phys_base + UART_LSR);

    if(iir == BIT(7)){
        readl(port->phys_base + UART_USR);
    } else {
        if(lsr & (UART_LSR_DR | UART_LSR_BI)){
            do{
                if(lsr & UART_LSR_DR){
                    ch = readb(port->phys_base + UART_RBR);
                }

                if(lsr & 0x1){
                    buf[count++] = (char)ch;
                }
                lsr = readl(port->phys_base + UART_LSR);
            }while(lsr & (UART_LSR_DR | UART_LSR_BI) && (max_count-- > 0));
            udd->recv(udd, buf, count);
        }
    }
    return IRQ_HANDLED;
}


static int dw_config_in(struct uart_driver_data *udd)
{
    return 0;
}

static int dw_startup(struct uart_driver_data *udd) 
{
    int ret = 0;
    struct dw_port *port = NULL;

    if (udd == NULL) {
        uart_error("udd is null!\n");
        return -EFAULT;
    }

    port = (struct dw_port *)udd->private;
    if (!port) {
        uart_error("port is null!");
        return -EFAULT;
    }
    /* enable the clock */
    LOS_TaskLock();
    //uart_clk_cfg(udd->num, true); //use for hi3518
    LOS_TaskUnlock();


    writel(BIT(0) | BIT(2), port->phys_base + UART_IER);

    ret = request_irq(port->irq_num, (irq_handler_t)dw_irq,
                              0, "uart_dw", udd);

    dw_config_in(udd);

    return ret;
}

static int dw_shutdown(struct uart_driver_data *udd)
{
    return 0;
}

int dw_start_tx(struct uart_driver_data *udd, const char *buf, size_t count)
{
    unsigned int tx_len = count;
    struct dw_port *port = NULL;
    char value;
    unsigned int i;
    int ret = 0;

    if (udd == NULL) {
        uart_error("udd is null!\n");
        return -EFAULT;
    }
    port = (struct dw_port *)udd->private;
    if (!port) {
        uart_error("port is null!");
        return -EFAULT;
    }
    /* UART_WITH_LOCK: there is a spinlock in the function to write reg in order. */
    for (i = 0; i < tx_len; i++ ){
        ret = LOS_CopyToKernel((void *)&value, sizeof(char),(void *)(buf++), sizeof(char));
        if (ret) {
            return i;
        }
        (void)UartPutsReg(port->phys_base, &value, 1, UART_WITH_LOCK);
    }
    return count;
}

static int dw_config(struct uart_driver_data *udd)
{
    return dw_config_in(udd);
}

static struct uart_ops dw_uops = {
    .startup        = dw_startup,
    .shutdown       = dw_shutdown,
    .start_tx       = dw_start_tx,
    .config         = dw_config,
};

#define MAX_DEV_NAME_SIZE  32
extern const struct file_operations_vfs uartdev_fops;
extern struct uart_driver_data *get_udd_by_unit(int unit);

struct uart_ops *dw_get_ops(void)
{
    return &dw_uops;
}
