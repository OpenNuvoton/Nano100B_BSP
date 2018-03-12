/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/12 5:01p $
 * @brief    Configure EBI interface to access SRAM connected to EBI interface.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "Nano100Series.h"


uint32_t u32DataArrary[4] = {0x1e20b23c, 0xf96ac369, 0x8c1352f2, 0x18a29ca1};
//uint32_t u32DataArrary[4] = {0xffffffff, 0x0, 0x5a5a5a5a, 0xa5a5a5a5};

int SRAM_Test(uint32_t u32EBIsize);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HXT_EN_Pos); // HXT Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Enable IP clock */
    CLK->AHBCLK |= CLK_AHBCLK_EBI_EN_Msk;
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    //SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    //SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_TX | SYS_PB_L_MFP_PB1_MFP_UART0_RX);
    SYS->PB_L_MFP = ((SYS->PB_L_MFP & 0xFFFFFF00) | 0x11);

    // Enable EBI_EN and EBI_MCLK_EN
    SYS->PC_H_MFP = (SYS->PC_H_MFP & 0xFFFFFFF0) | 0x2;

    // Enable nRD/nWR/ALE/nCS for EBI
    SYS->PA_H_MFP = (SYS->PA_H_MFP & 0xFFFF00FF) | 0x00002200;
    SYS->PB_L_MFP = (SYS->PB_L_MFP & 0x00FFFFFF) | 0x22000000;

    // Enable EBI AD Low-byte, bit 7~0
    SYS->PA_L_MFP = (SYS->PA_L_MFP & 0x00FFFFFF) | 0x22000000;
    SYS->PB_H_MFP = (SYS->PB_H_MFP & 0xFF00FFFF) | 0x00220000;
    SYS->PC_H_MFP = (SYS->PC_H_MFP & 0x00FFFFFF) | 0x22000000;
    SYS->PC_L_MFP = (SYS->PC_L_MFP & 0x00FFFFFF) | 0x22000000;

    // Enable nWRH & nWRL for support Byte-Write in 16bit Data Width Device(SARM)
    SYS->PB_L_MFP = (SYS->PB_L_MFP & 0xFFFF00FF) | 0x00002200;

    // Enable EBI AD High-byte, bit 15~8
    SYS->PA_L_MFP = (SYS->PA_L_MFP & 0xFF00000F) | 0x00222220;
    SYS->PA_H_MFP = (SYS->PA_H_MFP & 0xF000FFFF) | 0x02220000;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}


int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("+-------------------------------------------------+\n");
    printf("|              EBI SRAM Sample Code               |\n");
    printf("+-------------------------------------------------+\n");
    printf("\n");

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    EBI_Open(0, EBI_BUSWIDTH_8BIT, EBI_TIMING_VERYFAST, 0, 0);

    if (SRAM_Test(EBI_MAX_SIZE/2) < 0)
    {
        printf("EBI SRAM 8-bit write test failed!\n");
        while (1);
    }

    EBI_Close(0);

    printf("\nAll test pased.\n");

    while (1);
}


int SRAM_Test(uint32_t u32EBIsize)
{
    uint32_t    u32WriteData;
    uint32_t    u32Idx, u32DataIdx;

    /*------------------------------------------------*/
    /*  EBI SRAM byte write test                      */
    /*------------------------------------------------*/
    printf("[ Byte Write test ]\n");
    for (u32DataIdx = 0; u32DataIdx < 4; u32DataIdx++)
    {
        /*
         *  Write to SARM
         */
        u32WriteData = u32DataArrary[u32DataIdx] & 0xff;
        printf("    All 0x%02X Access ...", (uint8_t)u32WriteData);

        for (u32Idx = 0; u32Idx < u32EBIsize; u32Idx++)
        {
            EBI_WRITE_DATA8(u32Idx, u32WriteData);
        }

        /*
         *  Read to compare
         */
        for (u32Idx = 0; u32Idx < u32EBIsize; u32Idx++)
        {
            if (EBI_READ_DATA8(u32Idx) != u32WriteData)
            {
                printf("\n    Data compare failed at offset 0x%x, expect:0x%x, read:0x%x!\n", u32Idx, u32WriteData, EBI_READ_DATA8(u32Idx));
                return -1;
            }
        }
        printf("[OK]\n");
    }

    /*------------------------------------------------*/
    /*  EBI SRAM half-word write test                 */
    /*------------------------------------------------*/
    printf("[ Half-word Write test ]\n");
    for (u32DataIdx = 0; u32DataIdx < 4; u32DataIdx++)
    {
        /*
         *  Write to SARM
         */
        u32WriteData = u32DataArrary[u32DataIdx] & 0xffff;
        printf("    All 0x%04X Access ... ", u32WriteData);

        for (u32Idx = 0; u32Idx < u32EBIsize; u32Idx+=2)
        {
            EBI_WRITE_DATA16(u32Idx, u32WriteData);
        }

        /*
         *  Read to compare
         */
        for (u32Idx = 0; u32Idx < u32EBIsize; u32Idx+=2)
        {
            if (EBI_READ_DATA16(u32Idx) != u32WriteData)
            {
                printf("\n    Data compare failed at offset 0x%x, expect:0x%x, read:0x%x!\n", u32Idx, u32WriteData, EBI_READ_DATA16(u32Idx));
                return -1;
            }
        }
        printf("[OK]\n");
    }

    /*------------------------------------------------*/
    /*  EBI SRAM word write test                      */
    /*------------------------------------------------*/
    printf("[ Word Write test ]\n");
    for (u32DataIdx = 0; u32DataIdx < 4; u32DataIdx++)
    {
        /*
         *  Write to SARM
         */
        u32WriteData = u32DataArrary[u32DataIdx];
        printf("    All 0x%08X Access ... ", u32WriteData);

        for (u32Idx = 0; u32Idx < u32EBIsize; u32Idx+=4)
        {
            EBI_WRITE_DATA32(u32Idx, u32WriteData);
        }

        /*
         *  Read to compare
         */
        for (u32Idx = 0; u32Idx < u32EBIsize; u32Idx+=4)
        {
            if (EBI_READ_DATA32(u32Idx) != u32WriteData)
            {
                printf("\n    Data compare failed at offset 0x%x, expect:0x%x, read:0x%x!\n", u32Idx, u32WriteData, EBI_READ_DATA32(u32Idx));
                return -1;
            }
        }
        printf("[OK]\n");
    }
    return 0;
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
