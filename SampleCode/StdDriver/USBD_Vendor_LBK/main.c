/******************************************************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Date: 16/02/02 9:44a $
 * @brief    A USB device vendor class sample program. This sample code needs
 *           to test with NUC472 USBH_VENDOR_LBK(for Nano100B).
 *
 * @note     USBH_VENDOR_LBK(for Nano100B).7z could be found at NANO100BV3_BSP\SampleCode\StdDriver\USBD_Vendor_LBK\
 *           Please copy USBH_VENDOR_LBK(for Nano100B).7z to NUC472BSP\SampleCode\StdDriver\
 *           and unzip it to test this sample.
 *
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "vendor_lbk.h"


#define PLL_CLOCK           96000000


/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12MHz HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk);
    CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_HXT, PLL_CLOCK);
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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    SYS_Init();
    UART0_Init();

    printf("NuMicro USB Vendor\n");

    /* Endpoint configuration */
    LBK_Init();
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();

    USBD_Open(&gsInfo, NULL, NULL);

    USBD_SetVendorRequest(Vendor_ClassRequest);

    /* Endpoint configuration */
    LBK_Init();

    /* Enable USBD interrupt */
    NVIC_EnableIRQ(USBD_IRQn);

    /* Start transaction */
    USBD_Start();

    while(1)
    {
        LBK_IntOut();
        LBK_IntInData();

        LBK_IsoInPushData(&g_IsoLbkBuff[0], 64);

        LBK_BulkOut(&g_BulkLbkBuff[0], 512);
        LBK_BulkInPushData(&g_BulkLbkBuff[0], 512);
    }
}
/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
