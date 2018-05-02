/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/11 7:37p $
 * @brief    Use the timer pin PC.6 to demonstrate timer trigger counting mode
 *           function. And displays the measured input frequency to UART console.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"


void TMR0_IRQHandler(void)
{

    uint32_t t = TIMER_GetCaptureData(TIMER0);

    printf("Input frequency is %dHz\n", 1000000 / t);
    TIMER_ClearCaptureIntFlag(TIMER0);

}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);


    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB1_MFP_UART0_TX | SYS_PB_L_MFP_PB0_MFP_UART0_RX);

    SYS->PC_L_MFP  = (SYS->PC_L_MFP & ~SYS_PC_L_MFP_PC6_MFP_Msk) | SYS_PC_L_MFP_PC6_MFP_TMR0_CAP;
    /* Lock protected registers */
    SYS_LockReg();
}


int main(void)
{
    int volatile i;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\nThis sample code demonstrate timer trigger counting mode.\n");
    printf("Please connect input source with Timer 0 capture pin PC.6, press any key to continue\n");
    getchar();

    // Give a dummy target frequency here. Will over write capture resolution with macro
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000);

    // Update prescale to set proper resolution.
    // Timer 0 clock source is 12MHz, to set resolution to 1us, we need to
    // set clock divider to 12. e.g. set prescale to 12 - 1 = 11
    TIMER_SET_PRESCALE_VALUE(TIMER0, 11);

    // Set compare value as large as possible, so don't need to worry about counter overrun too frequently.
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);

    // Configure Timer 0 trigger counting mode, capture TDR value on rising edge
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_TRIGGER_COUNTING_MODE, TIMER_CAPTURE_RISING_EDGE);

    // Start Timer 0
    TIMER_Start(TIMER0);

    // Enable timer interrupt
    TIMER_EnableCaptureInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


