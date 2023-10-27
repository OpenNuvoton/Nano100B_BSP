/**************************************************************************//**
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB keyboard device.
 *           It supports to use GPIO to simulate key input.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "hid_kb.h"

/*--------------------------------------------------------------------------*/
uint8_t volatile g_u8EP2Ready = 0;

extern uint8_t Led_Status[8];
uint32_t LED_SATUS = 0;

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12MHz HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk);
    CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_HXT, 96000000);
    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_HXT_STB_Msk | CLK_CLKSTATUS_PLL_STB_Msk);

    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_HCLK_CLK_DIVIDER(3));

    /* Select IP clock source */
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_USB_CLK_DIVIDER(2));
    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX|SYS_PA_H_MFP_PA14_MFP_UART0_RX);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    UART0->BAUD = 0x67;              /* Baud Rate:115200  OSC:12MHz */
    UART0->TLCTL = 0x03;             /* Character len is 8 bits */
}


void HID_UpdateKbData(void)
{
    int32_t i;
    uint8_t *buf;
    uint32_t key = 0xF;
    static uint32_t preKey;

    if(g_u8EP2Ready)
    {
        buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));

        /* If GPB15 = 0, just report it is key 'a' */
        key = (PB->PIN & (1 << 15)) ? 0 : 1;

        if(key == 0)
        {
            for(i = 0; i < 8; i++)
            {
                buf[i] = 0;
            }

            if(key != preKey)
            {
                /* Trigger to note key release */
                USBD_SET_PAYLOAD_LEN(EP2, 8);
            }
        }
        else
        {
            preKey = key;
            buf[2] = 0x04; /* Key A */
            USBD_SET_PAYLOAD_LEN(EP2, 8);
        }
    }

    if(Led_Status[0] != LED_SATUS)
    {
        if((Led_Status[0] & HID_LED_ALL) != (LED_SATUS & HID_LED_ALL))
        {
            if(Led_Status[0] & HID_LED_NumLock)
                printf("NmLK  ON, ");
            else
                printf("NmLK OFF, ");

            if(Led_Status[0] & HID_LED_CapsLock)
                printf("CapsLock  ON, ");
            else
                printf("CapsLock OFF, ");

            if(Led_Status[0] & HID_LED_ScrollLock)
                printf("ScrollLock  ON, ");
            else
                printf("ScrollLock OFF, ");

            if(Led_Status[0] & HID_LED_Compose)
                printf("Compose  ON, ");
            else
                printf("Compose OFF, ");

            if(Led_Status[0] & HID_LED_Kana)
                printf("Kana  ON\n");
            else
                printf("Kana OFF\n");
        }
        LED_SATUS = Led_Status[0];
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|          NuMicro USB HID Keyboard Sample Code          |\n");
    printf("+--------------------------------------------------------+\n");
    printf("If PB.15 = 0, just report it is key 'a'.\n");

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();

    /* start to IN data */
    g_u8EP2Ready = 1;

    while(1)
    {
        HID_UpdateKbData();
    }
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

