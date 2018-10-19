/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/06/24 9:53a $
 * @brief    Use timer to wake up system from Power-down mode periodically.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

void TMR0_IRQHandler(void)
{
    // Clear wake up flag
    TIMER_ClearWakeupFlag(TIMER0);
    // Clear interrupt flag
    TIMER_ClearIntFlag(TIMER0);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External LXT (32 kHz) */
    CLK->PWRCTL |= CLK_PWRCTL_LXT_EN_Msk;

    /* Waiting for LXT clock ready */
    while(CLK_WaitClockReady( CLK_CLKSTATUS_LXT_STB_Msk) != 1);
    CLK->APBCLK |= CLK_APBCLK_TMR0_EN_Msk | CLK_APBCLK_UART0_EN_Msk;

    /* Select Timer clock source from LIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0_S_Msk) | CLK_CLKSEL1_TMR0_S_LXT;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB1_MFP_UART0_TX | SYS_PB_L_MFP_PB0_MFP_UART0_RX);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    int i = 0;
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*Initial Timer0 to periodic mode with 1Hz */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);

    /* Enable timer wake up system */
    TIMER_EnableWakeup(TIMER0);
    /* Enable Timer0 interrupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    /* Unlock protected registers */
    SYS_UnlockReg();

    while(1)
    {
        CLK_PowerDown();
        printf("Wake %d\n", i++);

    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


