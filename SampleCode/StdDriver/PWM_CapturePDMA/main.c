
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/09/11 5:46p $
 * @brief    Demonstrate PWM Capture function by using PWM0 channel 2 to capture the output of PWM0 channel 0 and move captured data to SRAM with PDMA.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

#define PDMA_CH 1
#define SAMPLE_CNT 32

uint16_t g_au16RxPDMADestination[SAMPLE_CNT];

volatile uint32_t g_u32PdmaTDoneInt;

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if (status & 0x2) { /* CH1 */
        if (PDMA_GET_CH_INT_STS(1) & 0x2)
            g_u32PdmaTDoneInt = 1;
        PDMA_CLR_CH_INT_FLAG(1, PDMA_ISR_TD_IS_Msk);
    } else if (status & 0x4) { /* CH2 */
        if (PDMA_GET_CH_INT_STS(2) & 0x2)
            g_u32PdmaTDoneInt = 2;
        PDMA_CLR_CH_INT_FLAG(2, PDMA_ISR_TD_IS_Msk);
    } else if (status & 0x8) { /* CH3 */
        if (PDMA_GET_CH_INT_STS(3) & 0x2)
            g_u32PdmaTDoneInt = 3;
        PDMA_CLR_CH_INT_FLAG(3, PDMA_ISR_TD_IS_Msk);
    } else if (status & 0x10) { /* CH4 */
        if (PDMA_GET_CH_INT_STS(4) & 0x2)
            g_u32PdmaTDoneInt = 4;
        PDMA_CLR_CH_INT_FLAG(4, PDMA_ISR_TD_IS_Msk);
    } else if (status & 0x20) { /* CH5 */
        if (PDMA_GET_CH_INT_STS(5) & 0x2)
            g_u32PdmaTDoneInt = 5;
        PDMA_CLR_CH_INT_FLAG(5, PDMA_ISR_TD_IS_Msk);
    } else if (status & 0x40) { /* CH6 */
        if (PDMA_GET_CH_INT_STS(6) & 0x2)
            g_u32PdmaTDoneInt = 6;
        PDMA_CLR_CH_INT_FLAG(6, PDMA_ISR_TD_IS_Msk);
    } else
        printf("unknown interrupt !!\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
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

    CLK->AHBCLK |= CLK_AHBCLK_DMA_EN_Msk;

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PWM clock */
    CLK_EnableModuleClock(PWM0_CH01_MODULE);
    CLK_EnableModuleClock(PWM0_CH23_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART_S_HXT,CLK_UART_CLK_DIVIDER(1));

    /* Set HCLK as PWM clock source */
    CLK_SetModuleClock(PWM0_CH01_MODULE, CLK_CLKSEL1_PWM0_CH01_S_HCLK, 0);
    CLK_SetModuleClock(PWM0_CH23_MODULE, CLK_CLKSEL1_PWM0_CH23_S_HCLK, 0);


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX );

    /* Set PB multi-function pins for Clock Output */
    SYS->PB_H_MFP = ( SYS->PB_H_MFP & ~SYS_PB_H_MFP_PB12_MFP_Msk ) |  SYS_PB_H_MFP_PB12_MFP_CKO;

    /* Set PA.12 and PA.14 multi-function pins for PWM channel 0 and 2 */
    SYS->PA_H_MFP = (SYS->PA_H_MFP & ~SYS_PA_H_MFP_PA12_MFP_Msk) | SYS_PA_H_MFP_PA12_MFP_PWM0_CH0;
    SYS->PA_H_MFP = (SYS->PA_H_MFP & ~SYS_PA_H_MFP_PA14_MFP_Msk) | SYS_PA_H_MFP_PA14_MFP_PWM0_CH2;

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

static void PDMA_INIT(void)
{
    uint32_t u32SrcAddr, u32DstAddr;

    /* Configure PDMA channel 1 to trigger PWM */
    PDMA_Open(1 << PDMA_CH);
    PDMA_SetTransferCnt(PDMA_CH, PDMA_WIDTH_16, SAMPLE_CNT);
    u32SrcAddr = (uint32_t)&PWM0->PDMACH2;
    u32DstAddr = (uint32_t)g_au16RxPDMADestination;
    PDMA_SetTransferAddr(PDMA_CH, u32SrcAddr, PDMA_SAR_FIX, u32DstAddr, PDMA_DAR_INC);
    PDMA_SetTimeOut(PDMA_CH, 0, 0x5555);
    PDMA_EnableInt(PDMA_CH, PDMA_IER_TD_IE_Msk);
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Set PDMA Channel 1 for PWM0 channel 2, and start timeout counting */
    PDMA_SetTransferMode(PDMA_CH, PDMA_PWM0_CH2, 0, 0);

    PDMA_Trigger(PDMA_CH);
}

int32_t main (void)
{
    uint8_t i;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\nThis sample code demonstrates PWM captured data transferred by PDMA.\n");
    printf("PWM0 channel 2 will capture the output of PWM0 channel 0\n");
    printf("So, please connect GPIO port A12 with A14.\n");
    // PWM0 frequency is 25000Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM0, 0, 25000, 30);

    // PWM2
    PWM_ConfigCaptureChannel(PWM0,2,50,0);

    // Enable output of channel 0
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    // Enable capture of channel 2
    PWM_EnableCapture(PWM0, PWM_CH_2_MASK);

    //Enable PWM channel 2 capture PDMA
    PWM_EnablePDMA(PWM0, 2, PWM_CAP_PDMA_RFORDER_R, PWM_RISING_FALLING_LATCH_PDMA_ENABLE);

    /* Configure PDMA channel 1 */
    PDMA_INIT();

    /* Clear destination buffer */
    for(i = 0; i < SAMPLE_CNT; i++)
        g_au16RxPDMADestination[i] = 0;

    // Start
    PWM_Start(PWM0, (PWM_CH_0_MASK|PWM_CH_2_MASK));

    /* Wait for PDMA transfer down */
    while(g_u32PdmaTDoneInt == 0);

    // Stop
    PWM_Stop (PWM0, (PWM_CH_0_MASK|PWM_CH_2_MASK));

    printf("Captured data is as below.\n");
    printf("(rising : falling)\n");
    for(i = 1; i < (SAMPLE_CNT  >> 1); i+=2) {  // ignore first sampled data. it's wrong
        printf("%d, %d : %d\n", i, g_au16RxPDMADestination[i], g_au16RxPDMADestination[i+1]);
    }

    while(1);


}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


