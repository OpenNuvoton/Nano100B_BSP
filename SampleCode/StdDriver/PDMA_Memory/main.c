/**************************************************************************//**
 * @file     main.c
 * @version  V2.10
 * $Date: 15/06/30 11:40a $
 * @brief    Use PDMA channel 4 to demonstrate memory to memory transfer.
 *
 * @note
 * Copyright (C) 2012 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

#define TEST_CH     4

uint32_t PDMA_TEST_LENGTH = 64;
uint8_t SrcArray[256];
uint8_t DestArray[256];
uint32_t volatile u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_nano100series.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if (status & 0x2)   /* CH1 */
    {
        if (PDMA_GET_CH_INT_STS(1) & 0x2)
            u32IsTestOver = 1;
        PDMA_CLR_CH_INT_FLAG(1, PDMA_ISR_TD_IS_Msk);
    }
    else if (status & 0x4)     /* CH2 */
    {
        if (PDMA_GET_CH_INT_STS(2) & 0x2)
            u32IsTestOver = 2;
        PDMA_CLR_CH_INT_FLAG(2, PDMA_ISR_TD_IS_Msk);
    }
    else if (status & 0x8)     /* CH3 */
    {
        if (PDMA_GET_CH_INT_STS(3) & 0x2)
            u32IsTestOver = 3;
        PDMA_CLR_CH_INT_FLAG(3, PDMA_ISR_TD_IS_Msk);
    }
    else if (status & 0x10)     /* CH4 */
    {
        if (PDMA_GET_CH_INT_STS(4) & 0x2)
            u32IsTestOver = 4;
        PDMA_CLR_CH_INT_FLAG(4, PDMA_ISR_TD_IS_Msk);
    }
    else if (status & 0x20)     /* CH5 */
    {
        if (PDMA_GET_CH_INT_STS(5) & 0x2)
            u32IsTestOver = 5;
        PDMA_CLR_CH_INT_FLAG(5, PDMA_ISR_TD_IS_Msk);
    }
    else if (status & 0x40)     /* CH6 */
    {
        if (PDMA_GET_CH_INT_STS(6) & 0x2)
            u32IsTestOver = 6;
        PDMA_CLR_CH_INT_FLAG(6, PDMA_ISR_TD_IS_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

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

    CLK->AHBCLK |= CLK_AHBCLK_DMA_EN_Msk;
    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
//    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
//    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX|SYS_PA_H_MFP_PA14_MFP_UART0_RX);
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB1_MFP_UART0_TX|SYS_PB_L_MFP_PB0_MFP_UART0_RX);

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
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz <%d>\n", SystemCoreClock, TEST_CH);

    printf("+--------------------------------------+ \n");
    printf("|    NANO100 PDMA Driver Sample Code   | \n");
    printf("+--------------------------------------+ \n");

    /* Open Channel 2 */
    PDMA_Open(1 << TEST_CH);
    PDMA_SetTransferCnt(TEST_CH, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    PDMA_SetTransferAddr(TEST_CH, (uint32_t)SrcArray, PDMA_SAR_INC, (uint32_t)DestArray, PDMA_DAR_INC);
    PDMA_SetTransferMode(TEST_CH, PDMA_MEM, 0, 0);
    PDMA_SetTimeOut(TEST_CH, 0, 0x5555);
    PDMA_EnableInt(TEST_CH, PDMA_IER_TD_IE_Msk);
    NVIC_EnableIRQ(PDMA_IRQn);
    u32IsTestOver = 0;
    PDMA_Trigger(TEST_CH);
    while(u32IsTestOver == 0);

    if (u32IsTestOver == TEST_CH)
        printf("test done...\n");

    PDMA_Close();
    while(1);
}


