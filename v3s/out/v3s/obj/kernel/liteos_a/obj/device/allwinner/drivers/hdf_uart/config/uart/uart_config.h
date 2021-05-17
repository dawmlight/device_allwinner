/*
 * It's HDF config auto-gen file, do not modify it manually
 */

#ifndef HCS_CONFIG_UART_CONFIG_HEADER_H
#define HCS_CONFIG_UART_CONFIG_HEADER_H

#include <stdint.h>

struct HdfConfigV3sUartUartController {
    const char* match_attr;
    uint8_t num;
    uint32_t baudrate;
    uint8_t fifoRxEn;
    uint8_t fifoTxEn;
    uint8_t flags;
    uint32_t regPbase;
    uint8_t interrupt;
    uint8_t iomemCount;
};

struct HdfConfigV3sUartPlatform {
    const struct HdfConfigV3sUartUartController* uartController;
    uint16_t uartControllerSize;
};

struct HdfConfigV3sUartRoot {
    const char* module;
    struct HdfConfigV3sUartPlatform platform;
};

const struct HdfConfigV3sUartRoot* HdfGetV3sUartModuleConfigRoot(void);

#endif // HCS_CONFIG_UART_CONFIG_HEADER_H