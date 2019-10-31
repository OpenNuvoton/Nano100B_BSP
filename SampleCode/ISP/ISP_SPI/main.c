/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  2.0.0
 *
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"

uint32_t Pclk0;
uint32_t Pclk1;

#define TEST_COUNT 16

uint32_t u32DataCount;
uint32_t *_response_buff;
uint32_t spi_rcvbuf[TEST_COUNT];

void SYS_Init(void)
{
    SYS_UnlockReg();

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_HXT_STB_Msk);

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Enable SPI1 peripheral clock */
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Setup SPI1 multi-function pins */
    SYS->PB_L_MFP = (SYS_PB_L_MFP_PB3_MFP_SPI1_SS0 | SYS_PB_L_MFP_PB2_MFP_SPI1_SCLK | SYS_PB_L_MFP_PB1_MFP_SPI1_MISO0 | SYS_PB_L_MFP_PB0_MFP_SPI1_MOSI0);
}

void SPI_Init(void)
{
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI1 as a low level active device. */
    /* Default setting: slave selection signal is low level active. */
    SPI1->SSR = SPI_SS0_ACTIVE_LOW | SPI_SSR_SS_LTRIG_Msk;
    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    SPI1->CTL = SPI_SLAVE | ((32 & 0x1F) << SPI_CTL_TX_BIT_LEN_Pos) | (SPI_MODE_0);
    /* Set DIVIDER = 0 */
    SPI1->CLKDIV = 0U;
}

int main(void)
{
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= (FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_APUEN_Msk);
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    SPI_Init();

    /* Get APROM size, data flash size and address */
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

_ISP:
    u32DataCount = 0;

    SysTick->CTRL = 0UL;

    /* Check data count */
    while(u32DataCount < TEST_COUNT)
    {
        /* Write to TX register */
        SPI_WRITE_TX0(SPI1, _response_buff[u32DataCount]);
        /* Trigger SPI data transfer */
        SPI_TRIGGER(SPI1);
        /* Check SPI1 busy status */
        while(SPI_IS_BUSY(SPI1))
        {
            if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                goto _ISP;
            }
        }

        /* Read RX register */
        spi_rcvbuf[u32DataCount] = SPI_READ_RX0(SPI1);
        u32DataCount++;

        SysTick->LOAD = 1000 * CyclesPerUs;
        SysTick->VAL  = (0x00);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    }

    /* Disable SysTick counter */
    SysTick->CTRL = 0UL;

    if((u32DataCount == TEST_COUNT) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900))
    {
        spi_rcvbuf[0] &= 0x000000FF;
        ParseCmd((unsigned char *)spi_rcvbuf, 64);
    }

    goto _ISP;

}
