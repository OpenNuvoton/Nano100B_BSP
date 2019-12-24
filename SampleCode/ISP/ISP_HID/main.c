/******************************************************************************
 * @file     main.c
 * @brief    Demonstrate how to transfer data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with a USB device.
 * @version  2.0.0
 * @date     12, Sep, 2014
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "hid_transfer.h"
#include "targetdev.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Enable external 12MHz HXT */
    CLK->PWRCTL |= (CLK_PWRCTL_HXT_EN_Msk | CLK_PWRCTL_HIRC_EN_Msk);

    /* Switch to HIRC for Safe. Avoid HCLK too high when applying new divider. */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;

    /* 12MHz HIRC ==> 96MHz Pll Colck Output */
    CLK->PLLCTL = CLK_PLLCTL_96MHz_HIRC;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk);

    /* 96MHz / (2 + 1) = 32MHz */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLK_N_Msk) | CLK_HCLK_CLK_DIVIDER(3);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | CLK_CLKSEL0_HCLK_S_PLL;
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
//    SystemCoreClockUpdate();
    SystemCoreClock = 32000000;     // HCLK
    CyclesPerUs     = 32;
    /* Select IP clock source */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USB_N_Msk) | CLK_USB_CLK_DIVIDER(2);
    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_USBD_EN;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void USBD_IRQHandler(void);
int32_t main(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();
    WDT->CTL &= ~(WDT_CTL_WTE_Msk);
    WDT->CTL |= (WDT_TIMEOUT_2POW18 | WDT_CTL_WTR_Msk);
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    if (DetectPin == 0)
    {
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);
        /* Endpoint configuration */
        HID_Init();
        //NVIC_EnableIRQ(USBD_IRQn);
        USBD_Start();

        while (DetectPin == 0)
        {
            // polling USBD interrupt flag
            USBD_IRQHandler();
            if (bUsbDataReady == TRUE)
            {
                WDT->CTL &= ~(WDT_CTL_WTE_Msk);
                WDT->CTL |= (WDT_TIMEOUT_2POW18 | WDT_CTL_WTR_Msk);
                ParseCmd((uint8_t *)usb_rcvbuf, EP3_MAX_PKT_SIZE);
                EP2_Handler();
                bUsbDataReady = FALSE;
            }
        }
    }
    else
    {
        SysTick->LOAD = 300000 * CyclesPerUs;
        SysTick->VAL  = (0x00);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        /* Waiting for down-count to zero */
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    }

    outpw(&SYS->RST_SRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}

