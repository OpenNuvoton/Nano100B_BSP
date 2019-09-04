/**************************************************************************//**
 * @file     main.c
 * @brief
 *           Use SD card as storage to implement a USB Mass-Storage device.
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "massstorage.h"
#include "SDCARD.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

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
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);

    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1_S_HCLK, 0);
    CLK_EnableModuleClock(SPI1_MODULE);


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PA_H_MFP &= ~(SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX | SYS_PA_H_MFP_PA14_MFP_UART0_RX);

    /* Set SPI1 multi-function pins */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB2_MFP_GPB2 | SYS_PB_L_MFP_PB1_MFP_GPB1 | SYS_PB_L_MFP_PB0_MFP_GPB0);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB2_MFP_SPI1_SCLK | SYS_PB_L_MFP_PB1_MFP_SPI1_MISO0 | SYS_PB_L_MFP_PB0_MFP_SPI1_MOSI0);

    GPIO_SetMode(PC, BIT5, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PC, BIT4, GPIO_PMD_OUTPUT);

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

    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB MassStorage Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    SDCARD_Open();
    USBD_Open(&gsInfo, MSC_ClassRequest, NULL);

    USBD_SetConfigCallback(MSC_SetConfig);

    /* Endpoint configuration */
    MSC_Init();
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);

    while (1)
    {
        if (g_usbd_UsbConfig)
            MSC_ProcessCmd();
    }
}



/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

