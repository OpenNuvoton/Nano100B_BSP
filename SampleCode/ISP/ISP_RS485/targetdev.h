/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "Nano100Series.h"
#include "uart_transfer.h"
#include "ISP_USER.h"

/* rename for uart_transfer.c */
#define UART_N                      UART0
#define UART_N_IRQHandler           UART0_IRQHandler
#define UART_N_IRQn                 UART0_IRQn

/* 0: Diable, 1: Enable */
#define USE_ISP_HID                 (1)
#define DetectPin                   0

#define USE_ISP_UART                (1)

