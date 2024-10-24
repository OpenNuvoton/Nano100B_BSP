/***************************************************************************//**
 * @file     main.c
 * @brief
 *           Demonstrate how to upgrade firmware between USB device and PC through USB DFU(Device Firmware Upgrade) class.
 *           A Windows tool is also included in this sample code to connect with USB device.
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "dfu_transfer.h"

uint32_t g_apromSize;

#define V6M_AIRCR_VECTKEY_DATA    0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ     0x00000004UL

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
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));

    /* 96MHz / (2 + 1) = 32MHz */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLK_N_Msk) | CLK_HCLK_CLK_DIVIDER(3);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | CLK_CLKSEL0_HCLK_S_PLL;
    /* Update System Core Clock */
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
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_APUEN_Msk | FMC_ISPCON_ISPFF_Msk;
    g_apromSize = GetApromSize();

    /* Open USB controller */
    USBD_Open(&gsInfo, DFU_ClassRequest, NULL);

    /*Init Endpoint configuration for DFU */
    DFU_Init();

    /* Start USB device */
    USBD_Start();

    /* polling USBD interrupt flag */
    while (DetectPin == 0)
    {
        USBD_IRQHandler();
    }

    outpw(&SYS->RST_SRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Empty functions for reduce code size to fit into LDROM & solve the functions are not be defined.       */
/*---------------------------------------------------------------------------------------------------------*/
void ProcessHardFault()
{}

void SH_Return()
{}
