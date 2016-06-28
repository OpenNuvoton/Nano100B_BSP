/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/11 7:12p $
 * @brief    Demonstrate software trigger DAC convert sine wave outputs.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

#define SINE_ARRAY_SIZE 63
const uint16_t a16Sine[SINE_ARRAY_SIZE] = {2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
                                           4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
                                           3102, 2921, 2732, 2536, 2335, 2132, 1927, 1724, 1523, 1328, 1141,  962,  794,
                                            639,  497,  371,  262,  171,   99,   45,   12,    0,    7,   35,   84,  151,
                                            238,  343,  465,  602,  754,  919, 1095, 1281, 1475, 1674, 1876};


uint32_t index0 = 0;
//180 degrees phase difference
uint32_t index1 = SINE_ARRAY_SIZE / 2;

void DAC_IRQHandler(void)
{
    // clear interrupt flag
    DAC_CLR_INT_FLAG(DAC, 0);

    DAC_WRITE_DATA(DAC, 0, a16Sine[index0]);
    DAC_WRITE_DATA(DAC, 1, a16Sine[index1]); // Write channel 1 will trigger DAC to update both channels in group mode
    index0 = (index0 + 1) % SINE_ARRAY_SIZE;
    index1 = (index1 + 1) % SINE_ARRAY_SIZE;

    return;
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

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable DAC clock */
    CLK->APBCLK |= CLK_APBCLK_DAC_EN_Msk;

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART_S_HXT,CLK_UART_CLK_DIVIDER(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX );

    /* Set PC.6 and PC.7 multi-function pin for DAC */
    SYS->PC_L_MFP &= ~(SYS_PC_L_MFP_PC6_MFP_Msk | SYS_PC_L_MFP_PC7_MFP_Msk);
    SYS->PC_L_MFP |= SYS_PC_L_MFP_PC6_MFP_DA_OUT0 | SYS_PC_L_MFP_PC7_MFP_DA_OUT1;

    /* Disable digital input path for both PC.6 and PC.7*/
    GPIO_DISABLE_DIGITAL_PATH(PC, (1 << 6) | (1 << 7));

    /* Lock protected registers */
    SYS_LockReg();
}



int32_t main (void)
{

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART0 for printf */
    UART_Open(UART0, 115200);

    printf("\nThis sample code demonstrate DAC function.\n");
    printf("Two DAC channels will output sine wave with 180 degrees phase difference in group mode.\n");

    // Enable channel 0 and 1. DAC will convert after software writes new data.
    DAC_Open(DAC, 0, DAC_WRITE_DAT_TRIGGER);
    DAC_Open(DAC, 1, DAC_WRITE_DAT_TRIGGER);

    // Enable DAC group mode
    DAC_ENABLE_GROUP_MODE(DAC);

    // Enable DAC0 interrupt. Enable interrupt for one channel is sufficient in group mode.
    DAC_ENABLE_INT(DAC, 0);
    NVIC_EnableIRQ(DAC_IRQn);

    // Wait 'til both channels are ready
    while(DAC_IS_BUSY(DAC, 0) == 1);
    while(DAC_IS_BUSY(DAC, 1) == 1);

    DAC_WRITE_DATA(DAC, 0, a16Sine[index0]);
    DAC_WRITE_DATA(DAC, 1, a16Sine[index1]); // Write channel 1 will trigger DAC to update both channels in group mode
    index0 = (index0 + 1) % SINE_ARRAY_SIZE;
    index1 = (index1 + 1) % SINE_ARRAY_SIZE;

    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


