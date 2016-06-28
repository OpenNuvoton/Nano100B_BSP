/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/16 7:22p $
 * @brief    Use PDMA channel 1 to move ADC channel 0, 1, 2 converted data to SRAM
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

    /* Enable ADC clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART_S_HXT,CLK_UART_CLK_DIVIDER(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX );

    /* Set PB multi-function pins for Clock Output */
    SYS->PB_H_MFP = ( SYS->PB_H_MFP & ~SYS_PB_H_MFP_PB12_MFP_Msk ) |  SYS_PB_H_MFP_PB12_MFP_CKO;

    /* Set PA.0,PA.1,PA.2,PA.3 multi-function pin for ADC channel 0,1,2,3 */
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA0_MFP_Msk) | SYS_PA_L_MFP_PA0_MFP_ADC_CH0;
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA1_MFP_Msk) | SYS_PA_L_MFP_PA1_MFP_ADC_CH1;
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA2_MFP_Msk) | SYS_PA_L_MFP_PA2_MFP_ADC_CH2;
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA3_MFP_Msk) | SYS_PA_L_MFP_PA3_MFP_ADC_CH3;

    /* Disable PA.0 digital input path */
    PA->OFFD |= ((1 << 0) << GP_OFFD_OFFD_Pos);

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

    /* Configure PDMA channel 1 to trigger ADC */
    PDMA_Open(1 << PDMA_CH);
    PDMA_SetTransferCnt(PDMA_CH, PDMA_WIDTH_32, ADC_TEST_COUNT);
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
    uint32_t u32DataCount;
    uint32_t u32ErrorCount;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nThis sample code demonstrates ADC PDMA function.\n");
    printf("Set ADC operation mode to single cycle scan mode, and enable channel 0,1,2,3\n");
    printf("Enable ADC PDMA function, and trigger ADC conversion.\n");
    printf("Compare the log of ADC conversion data register with the content of PDMA target buffer.\n");
    printf("Finally, print the test result.\n\n");

    // Enable channel 0,1,2,3
    ADC_Open(ADC, ADC_INPUT_MODE_SINGLE_END, ADC_OPERATION_MODE_SINGLE_CYCLE, ADC_CH_0_MASK | ADC_CH_1_MASK | ADC_CH_2_MASK | ADC_CH_3_MASK);

    // Set reference voltage to AVDD
    ADC_SET_REF_VOLTAGE(ADC, ADC_REFSEL_POWER);

    // Power on ADC
    ADC_POWER_ON(ADC);

    /* Enable ADC PDMA */
    ADC_ENABLE_PDMA(ADC);

    /* Configure PDMA channel 1 */
    PDMA_INIT();

    /* Enable PDMA IRQ */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Clear destination buffer */
    for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
        g_au32RxPDMADestination[u32DataCount] = 0;

    u32DataCount = 0;
    u32ErrorCount = 0;

    ADC_START_CONV(ADC);

    while(1) {
        uint32_t u32Ch;
        if(ADC_GET_INT_FLAG(ADC,ADC_ADF_INT) == 1) {
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            for (u32Ch = 0; u32Ch < 4; u32Ch++) {
                au32AdcData[u32DataCount++] = ADC_GET_CONVERSION_DATA(ADC, u32Ch);
                if(u32DataCount >= ADC_TEST_COUNT)
                    break;
            }
            if (u32DataCount < ADC_TEST_COUNT)
                ADC_START_CONV(ADC);
            else
                break;
        }
    }
    
    /* Wait for PDMA transfer down */
    while(g_u32PdmaTDoneInt == 0);

    /* Compare the log of ADC conversion data register with the content of PDMA target buffer */
    for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++) {
        if( au32AdcData[u32DataCount] != (g_au32RxPDMADestination[u32DataCount] & 0xFFF) ) {
            printf("*** Count %d, conversion result: 0x%X, PDMA result: 0x%X.\n",
                   u32DataCount, au32AdcData[u32DataCount], g_au32RxPDMADestination[u32DataCount]);
            u32ErrorCount++;
        }
    }

    if (u32ErrorCount == 0)
        printf("PASS!\n");
    else
        printf("FAIL!\n");

    while (1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


