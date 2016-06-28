/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/11 5:35p $
 * @brief    Demonstrate SPI 2 bit mode loop back transfer.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

#define TEST_COUNT 64

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData0[TEST_COUNT], g_au32DestinationData1[TEST_COUNT];
volatile uint32_t SPI1_INT_Flag;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /* Enable external 12MHz HXT, 32KHz LXT and HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk | CLK_PWRCTL_LXT_EN_Msk | CLK_PWRCTL_HIRC_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_HXT_STB_Msk | CLK_CLKSTATUS_LXT_STB_Msk | CLK_CLKSTATUS_HIRC_STB_Msk);

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0_S_HCLK, 0);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1_S_HCLK, 0);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD  */
    SYS->PA_H_MFP = (SYS_PA_H_MFP_PA14_MFP_UART0_RX | SYS_PA_H_MFP_PA15_MFP_UART0_TX);

    /* Set multi function pin for SPI0 */
    SYS->PC_L_MFP = (SYS_PC_L_MFP_PC0_MFP_SPI0_SS0 | SYS_PC_L_MFP_PC1_MFP_SPI0_SCLK | SYS_PC_L_MFP_PC2_MFP_SPI0_MISO0 | SYS_PC_L_MFP_PC3_MFP_SPI0_MOSI0
                     | SYS_PC_L_MFP_PC4_MFP_SPI0_MISO1 | SYS_PC_L_MFP_PC5_MFP_SPI0_MOSI1);

    /* Set multi function pin for SPI1 */
    SYS->PC_H_MFP = (SYS_PC_H_MFP_PC8_MFP_SPI1_SS0 | SYS_PC_H_MFP_PC9_MFP_SPI1_SCLK | SYS_PC_H_MFP_PC10_MFP_SPI1_MISO0 | SYS_PC_H_MFP_PC11_MFP_SPI1_MOSI0
                     | SYS_PC_H_MFP_PC12_MFP_SPI1_MISO1 | SYS_PC_H_MFP_PC13_MFP_SPI1_MOSI1);

    /* Lock protected registers */
    SYS_LockReg();
}

void SPI1_IRQHandler(void)
{
    if( SPI_GET_STATUS(SPI1) & SPI_STATUS_INTSTS_Msk ) { /* Check the unit transfer interrupt flag */
        SPI_CLR_UNIT_TRANS_INT_FLAG(SPI1);   /* write '1' to clear SPI1 uint transfer interrupt flag */
        SPI1_INT_Flag = 1;
    }
}

int main(void)
{
    uint32_t u32DataCount, u32TestCount, u32Err;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI0 as a master, MSB first, 32-bit transaction, SPI Mode-0 timing, clock is 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SPI0_SS0 pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS0, SPI_SS0_ACTIVE_LOW);

    /* Enable SPI0 two bit mode */
    SPI_ENABLE_2BIT_MODE(SPI0);

    /* Configure SPI1 as a slave, MSB first, 32-bit transaction, SPI Mode-0 timing, clock is 4Mhz */
    SPI_Open(SPI1, SPI_SLAVE, SPI_MODE_0, 32, 4000000);

    /* Configure SPI1 as a low level active device. */
    SPI_SET_SS0_LOW(SPI1);

    /* Enable SPI1 two bit mode */
    SPI_ENABLE_2BIT_MODE(SPI1);

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|              Nano100 Series SPI Driver Sample Code                   |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");

    printf("Configure SPI0 as a master and SPI1 as a slave.\n");
    printf("The I/O connection for SPI0/SPI1 loopback:\n");
    printf("    SPI0_SS0  (PC.0) <-->  SPI1_SS0  (PC.8)\n    SPI0_CLK  (PC.1) <-->  SPI1_CLK  (PC.9)\n");
    printf("    SPI0_MISO0(PC.2) <-->  SPI1_MISO0(PC.10)\n    SPI0_MOSI0(PC.3) <-->  SPI1_MOSI0(PC.11)\n");
    printf("    SPI0_MISO1(PC.4) <-->  SPI1_MISO1(PC.12)\n    SPI0_MOSI1(PC.5) <-->  SPI1_MOSI1(PC.13)\n");
    printf("Please connect SPI0 with SPI1, and press any key to start transmission ...");
    getchar();
    printf("\n");

    printf("\nSPI0/1 2-bit Mode Loopback Test ");

    /* Enable the SPI1 unit transfer interrupt. */
    SPI_EnableInt(SPI1, SPI_IE_MASK);
    NVIC_EnableIRQ(SPI1_IRQn);
    SPI_TRIGGER(SPI1);

    u32Err = 0;
    for(u32TestCount=0; u32TestCount<10000; u32TestCount++) {
        /* set the source data and clear the destination buffer */
        for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++) {
            g_au32SourceData[u32DataCount] = u32DataCount;
            g_au32DestinationData0[u32DataCount] = 0;
            g_au32DestinationData1[u32DataCount] = 0;
        }

        u32DataCount=0;
        SPI1_INT_Flag = 0;

        if((u32TestCount&0x1FF) == 0) {
            putchar('.');
        }

        /* write the first data of source buffer to Tx register of SPI0. And start transmission. */
        SPI_WRITE_TX0(SPI0, g_au32SourceData[0]);
        SPI_WRITE_TX1(SPI0, g_au32SourceData[0]);
        SPI_TRIGGER(SPI0);

        while(1) {
            if(SPI1_INT_Flag==1) {
                SPI1_INT_Flag = 0;

                if(u32DataCount<(TEST_COUNT-1)) {
                    /* Read the previous retrieved data and trigger next transfer. */
                    g_au32DestinationData0[u32DataCount] = SPI_READ_RX0(SPI1);
                    g_au32DestinationData1[u32DataCount] = SPI_READ_RX1(SPI1);
                    u32DataCount++;
                    SPI_TRIGGER(SPI1);

                    /* Write data to SPI0 Tx buffer and trigger the transfer */
                    SPI_WRITE_TX0(SPI0, g_au32SourceData[u32DataCount]);
                    SPI_WRITE_TX1(SPI0, g_au32SourceData[u32DataCount]);
                    SPI_TRIGGER(SPI0);
                } else {
                    /* Just read the previous retrieved data but trigger next transfer, because this is the last transfer. */
                    g_au32DestinationData0[u32DataCount] = SPI_READ_RX0(SPI1);
                    g_au32DestinationData1[u32DataCount] = SPI_READ_RX1(SPI1);
                    SPI_TRIGGER(SPI1);
                    break;
                }
            }
        }

        /*  Check the received data */
        for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++) {
            if((g_au32DestinationData0[u32DataCount]!=g_au32SourceData[u32DataCount]) ||
                    (g_au32DestinationData1[u32DataCount]!=g_au32SourceData[u32DataCount]))
                u32Err = 1;
        }

        if(u32Err)
            break;
    }
    /* Disable the SPI1 unit transfer interrupt. */
    SPI_DisableInt(SPI1, SPI_IE_MASK);
    NVIC_DisableIRQ(SPI1_IRQn);

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");


    printf("\n\nExit SPI driver sample code.\n");

    while(1);
}
