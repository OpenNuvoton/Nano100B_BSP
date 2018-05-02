/**************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to implement a composite device.(HID Transfer and Mass storage)
             Transfer data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with a USB device.
 * @version  2.0.0
 * @date     16, Mar, 2015
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "hid_transfer_and_MSC.h"
#include "massstorage.h"

#define CONFIG_BASE      0x00300000
#define DATA_FLASH_BASE  0x00010000

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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    uint32_t au32Config[2];

    SYS_Init();
    UART0_Init();

    printf("NuMicro USB composite device Sample.(HID Transfer and Mass storage)\n");

    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    /* Check if Data Flash Size is 64K. If not, to re-define Data Flash size and to enable Data Flash function */
    if (FMC_ReadConfig(au32Config, 2) < 0)
        return -1;

    if (((au32Config[0] & 0x01) == 1) || (au32Config[1] != DATA_FLASH_BASE) )
    {
        FMC_ENABLE_CFG_UPDATE();
        au32Config[0] &= ~0x1;
        au32Config[1] = DATA_FLASH_BASE;
        if (FMC_WriteConfig(au32Config, 2) < 0)
            return -1;

        FMC_ReadConfig(au32Config, 2);
        if (((au32Config[0] & 0x01) == 1) || (au32Config[1] != DATA_FLASH_BASE))
        {
            printf("Error: Program Config Failed!\n");
            /* Disable FMC ISP function */
            FMC_Close();
            SYS_LockReg();
            return -1;
        }

        /* Reset Chip to reload new CONFIG value */
        SYS->IPRST_CTL1 = SYS_IPRST_CTL1_CHIP_RST_Msk;
    }
    SYS_LockReg();

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();

    while(1)
    {
        if (g_usbd_UsbConfig)
            MSC_ProcessCmd();
    }
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

