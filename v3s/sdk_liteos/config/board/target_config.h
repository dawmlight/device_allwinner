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

#ifndef __TARGET_CONFIG_H__
#define __TARGET_CONFIG_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*=============================================================================
                                        System clock module configuration
=============================================================================*/
#define OS_SYS_CLOCK                    24000000
#define SYS_CTRL_REG_BASE               IO_DEVICE_ADDR(0x12020000)
#define REG_SC_CTRL                     0

/* memory */
#define CACHE_ALIGNED_SIZE              64
/* physical memory base and size */
#define DDR_MEM_ADDR                    0x40000000
#define DDR_MEM_SIZE                    0x03e00000
#define SYS_MEM_BASE                    DDR_MEM_ADDR

/* Peripheral register address base and size */
#define PERIPH_PMM_BASE                 0x01000000
#define PERIPH_PMM_SIZE                 0x01000000
#define SYS_MEM_SIZE_DEFAULT            0x03e00000


/* hwi */
/**
 * Maximum number of supported hardware devices that generate hardware interrupts.
 * The maximum number of hardware devices that generate hardware interrupts is 128.
 */
#define OS_HWI_MAX_NUM                  128

/**
 * Maximum interrupt number.
 */
#define OS_HWI_MAX                      ((OS_HWI_MAX_NUM) - 1)

/**
 * Minimum interrupt number.
 */
#define OS_HWI_MIN                      0
/**
 * Maximum usable interrupt number.
 */
#define OS_USER_HWI_MAX                 OS_HWI_MAX
/**
 * Minimum usable interrupt number.
 */
#define OS_USER_HWI_MIN                 OS_HWI_MIN

#define NUM_HAL_INTERRUPT_CNTPSIRQ      29
#define NUM_HAL_INTERRUPT_CNTPNSIRQ     30
#define OS_TICK_INT_NUM                 NUM_HAL_INTERRUPT_CNTPSIRQ // use secure physical timer for now
#define NUM_HAL_INTERRUPT_TIMER0        37
#define NUM_HAL_INTERRUPT_TIMER3        38
#define NUM_HAL_INTERRUPT_UART0         32

/* gic config */
#ifndef GIC_BASE_ADDR
#define GIC_BASE_ADDR                   IO_DEVICE_ADDR(0x01C80000)
#endif
#define GICD_OFFSET                     0x1000     /* interrupt distributor offset */
#define GICC_OFFSET                     0x2000     /* CPU interface register offset */

/* timer config */
#define BIT(n)                          (1U << (n))
#define TIMER0_ENABLE                   BIT(16)
#define TIMER1_ENABLE                   BIT(17)
#define TIMER2_ENABLE                   BIT(18)
#define TIMER3_ENABLE                   BIT(19)

#define TIMER0_REG_BASE                 IO_DEVICE_ADDR(0x12000000)
#define TIMER1_REG_BASE                 IO_DEVICE_ADDR(0x12000020)
#define TIMER2_REG_BASE                 IO_DEVICE_ADDR(0x12001000)
#define TIMER3_REG_BASE                 IO_DEVICE_ADDR(0x12001020)

#define TIMER_TICK_REG_BASE             TIMER0_REG_BASE   /* timer for tick */
#define TIMER_TICK_ENABLE               TIMER0_ENABLE
#define TIMER_TIME_REG_BASE             TIMER1_REG_BASE   /* timer for time */
#define TIMER_TIME_ENABLE               TIMER1_ENABLE

#define LOSCFG_BASE_CORE_TICK_HW_TIME   NO
#define NUM_HAL_INTERRUPT_TIMER         NUM_HAL_INTERRUPT_TIMER0
#define NUM_HAL_INTERRUPT_HRTIMER       NUM_HAL_INTERRUPT_TIMER3

#define TIMER_LOAD                      0x0
#define TIMER_VALUE                     0x4
#define TIMER_CONTROL                   0x8
#define TIMER_INT_CLR                   0xc
#define TIMER_RIS                       0x10
#define TIMER_MIS                       0x14
#define TIMER_BGLOAD                    0x18

/* uart config */
#define UART0_REG_BASE                  IO_DEVICE_ADDR(0x01C28000)
#define TTY_DEVICE                      "/dev/uartdev-0"
#define UART_REG_BASE                   UART0_REG_BASE
#define NUM_HAL_INTERRUPT_UART          32

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif
