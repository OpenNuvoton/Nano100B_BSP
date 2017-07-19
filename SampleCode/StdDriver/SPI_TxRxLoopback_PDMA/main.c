/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/09/09 8:57a $
 * @brief    Demonstrate the usage of PDMA transfer. One SPI interface
 *           is enabled in loopback mode. Two PDMA channels are used
 *           in this sample, one for transmit, the other for receive.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

#define PDMA_TEST_COUNT 16

uint32_t g_au32SrcData[PDMA_TEST_COUNT];
uint32_t g_au32DstData[PDMA_TEST_COUNT];
uint32_t volatile u32IsTestOver = 0;

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if (status & 0x2) { /* done */
        PDMA_CLR_CH_INT_FLAG(1, PDMA_ISR_TD_IS_Msk);
    } else if (status & 0x4) { /* done */
        if (PDMA_GET_CH_INT_STS(2) & 0x2)
            u32IsTestOver = 1;
        PDMA_CLR_CH_INT_FLAG(2, PDMA_ISR_TD_IS_Msk);
    } else
        printf("unknown interrupt, status=0x%x !!\n", status);
}

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

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);
    CLK_EnableModuleClock(DMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD  */
    SYS->PA_H_MFP = (SYS_PA_H_MFP_PA14_MFP_UART0_RX | SYS_PA_H_MFP_PA15_MFP_UART0_TX);

    /* Set multi function pin for SPI0 */
    SYS->PC_L_MFP = (SYS_PC_L_MFP_PC0_MFP_SPI0_SS0 | SYS_PC_L_MFP_PC1_MFP_SPI0_SCLK | SYS_PC_L_MFP_PC2_MFP_SPI0_MISO0 | SYS_PC_L_MFP_PC3_MFP_SPI0_MOSI0);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32Err=0;
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI0 as a master, MSB first, 32-bit transaction, SPI Mode-0 timing, clock is 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SPI0_SS0 pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS0, SPI_SS0_ACTIVE_LOW);

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                     SPI with PDMA Sample Code                        |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");

    printf("The I/O connection for SPI0 loopback:\n");
    printf("    SPI0_MISO(PC.2) <--> SPI0_MOSI(PC.3)\n\n\n");
    printf("Please connect SPI0 MISO and MOSI pin, and press any key to start transmission ...");
    getchar();
    printf("\n");

    for(i=0; i<PDMA_TEST_COUNT; i++)
        g_au32SrcData[i] = 0x55550000 + i;

    /* Open Channel 1 for SPI0 TX, channel 2 for SPI0 RX */
    PDMA_Open(3 << 1);

    /* Configure channel 1 */
    PDMA_SetTransferCnt(1, PDMA_WIDTH_32, PDMA_TEST_COUNT);
    PDMA_SetTransferAddr(1, (uint32_t)g_au32SrcData, PDMA_SAR_INC, (uint32_t)&SPI0->TX0, PDMA_DAR_FIX);
    PDMA_SetTimeOut(1, 0, 0x5555);
    PDMA_EnableInt(1, PDMA_IER_TD_IE_Msk);

    /* Configure channel 2 */
    PDMA_SetTransferCnt(2, PDMA_WIDTH_32, PDMA_TEST_COUNT);
    PDMA_SetTransferAddr(2, (uint32_t)&SPI0->RX0, PDMA_SAR_FIX, (uint32_t)g_au32DstData, PDMA_DAR_INC);
    PDMA_SetTimeOut(2, 0, 0x5555);
    PDMA_EnableInt(2, PDMA_IER_TD_IE_Msk);

    /* Set Channel 1 for SPI0 TX, channel 2 for SPI0 RX, and then start timeout counting */
    PDMA_SetTransferMode(1, PDMA_SPI0_TX, 0, 0);
    PDMA_SetTransferMode(2, PDMA_SPI0_RX, 0, 0);

    PDMA_Trigger(1);
    PDMA_Trigger(2);

    /* Enable PDMA IRQ */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Trigger PDMA */
    SPI_TRIGGER_RX_PDMA(SPI0);
    SPI_TRIGGER_TX_PDMA(SPI0);

    /* Wait for PDMA operation finish */
    while(u32IsTestOver == 0);

    /* Check PDMA status */
    if(u32IsTestOver != 1)
        printf("PDMA error !\n");

    /* Check Rx Data */
    for(i=0; i<PDMA_TEST_COUNT; i++) {
        if(g_au32SrcData[i] != g_au32DstData[i]) {
            u32Err ++;
        }
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
