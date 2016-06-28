/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/16 7:23p $
 * @brief    Configure Timer0 to ADC and move converted data to SRAM using PDMA.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

#define PDMA_CH 1
#define ADC_TEST_COUNT 32

uint32_t g_au32RxPDMADestination[ADC_TEST_COUNT];
uint32_t au32AdcData[ADC_TEST_COUNT];

volatile uint32_t g_u32PdmaTDoneInt;
volatile uint32_t g_u32PdmaTAbortInt;

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

void TMR0_IRQHandler(void)
{
    TIMER_ClearIntFlag(TIMER0);
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
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART_S_HXT,CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX );

    /* Set PB multi-function pins for Clock Output */
    SYS->PB_H_MFP = ( SYS->PB_H_MFP & ~SYS_PB_H_MFP_PB12_MFP_Msk ) |  SYS_PB_H_MFP_PB12_MFP_CKO;

    /* Set PA.0, PA.1, PA.2, PA.3 multi-function pin for ADC channel 0,1,2,3 */
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA0_MFP_Msk) | SYS_PA_L_MFP_PA0_MFP_ADC_CH0;
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA1_MFP_Msk) | SYS_PA_L_MFP_PA1_MFP_ADC_CH1;
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA2_MFP_Msk) | SYS_PA_L_MFP_PA2_MFP_ADC_CH2;
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA3_MFP_Msk) | SYS_PA_L_MFP_PA3_MFP_ADC_CH3;

    /* Disable PA.0, PA.1, PA.2, PA.3 digital input path */
    PA->OFFD |= (0xf << GP_OFFD_OFFD_Pos);

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

void PDMA_INIT(void)
{
    uint32_t u32SrcAddr, u32DstAddr;
    uint32_t u32PDMACnt;

    /* Configure PDMA channel 1 to trigger ADC */
    PDMA_Open(1 << PDMA_CH);
    u32PDMACnt = 9; //Transmit 2 cycles => ((4*2)+1)
    PDMA_SetTransferCnt(PDMA_CH, PDMA_WIDTH_32, u32PDMACnt);
    u32SrcAddr = (uint32_t)&ADC->PDMA;
    u32DstAddr = (uint32_t)g_au32RxPDMADestination;
    PDMA_SetTransferAddr(PDMA_CH, u32SrcAddr, PDMA_SAR_FIX, u32DstAddr, PDMA_DAR_INC);
    PDMA_SetTimeOut(PDMA_CH, 0, 0x5555);
    PDMA_EnableInt(PDMA_CH, PDMA_IER_TD_IE_Msk);
    NVIC_EnableIRQ(PDMA_IRQn);
    
    /* Set PDMA Channel 1 for ADC, and start timeout counting */
    PDMA_SetTransferMode(PDMA_CH, PDMA_ADC, 0, 0);

    PDMA_Trigger(PDMA_CH);
}

int32_t main (void)
{
    uint32_t u32TimerSrc;
    uint32_t u32PDMACnt;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nThis sample code demonstrate Timer trigger ADC function\n");

    // Enable channel 0,1,2,3
    ADC_Open(ADC, ADC_INPUT_MODE_SINGLE_END, ADC_OPERATION_MODE_CONTINUOUS, ADC_CH_0_MASK|ADC_CH_1_MASK|ADC_CH_2_MASK|ADC_CH_3_MASK);

    // Set reference voltage to AVDD
    ADC_SET_REF_VOLTAGE(ADC, ADC_REFSEL_POWER);

    // Enable timer 0 trigger ADC 
    u32TimerSrc = 0;
    u32PDMACnt = 3; //Transfer (3+1) ADC result whenever timer event occurred
    ADC_EnableTimerTrigger(ADC, u32TimerSrc, u32PDMACnt);

    /* Enable ADC PDMA */
    ADC_ENABLE_PDMA(ADC);

    /* Configure PDMA channel 1 */
    PDMA_INIT();

    // Give a dummy target frequency here. Will over write capture resolution with macro
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000);

    // Update prescale to set proper resolution.
    // Timer 0 clock source is 12MHz, to set resolution to 1us, we need to
    // set clock divider to 12. e.g. set prescale to 12 - 1 = 11
    TIMER_SET_PRESCALE_VALUE(TIMER0, 11);

    // Set compare value
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFF);

    // Set Timer 0 time out to trigger ADC
    TIMER_SetTriggerSource(TIMER0, TIMER_TIMEOUT_TRIGGER);
    TIMER_SetTriggerTarget(TIMER0, TIMER_CTL_ADC_TEEN_Msk);

    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    
    // Start Timer 0
    TIMER_Start(TIMER0);

    /* Wait for PDMA transfer down */
    while(g_u32PdmaTDoneInt == 0);

    if((g_au32RxPDMADestination[8] & 0xFFF) != ADC_GET_CONVERSION_DATA(ADC, 0))
        printf("PDMA data error\n");
    else
        printf("PDMA data ok\n");

    while (1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


