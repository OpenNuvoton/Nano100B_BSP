/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/09/11 7:11p $
 * @brief    Demonstrate the timer, WDT, and RTC function.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Nano100Series.h"

#define CLS()    do {printf("%c[2J", 0x1B); printf("%c[%d;1H", 0x1B, 0);}while(0)


uint32_t volatile rtc = 0, alarm = 0, tmr = 0, wdt = 0;
uint32_t volatile TimerCounter=0;
char string[15];

// Timer0 interrupt heandler
void TMR0_IRQHandler(void)
{
    TimerCounter += 1;

    tmr = 1;

    // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER0);
}

// RTC interrupt heandler
void RTC_IRQHandler(void)
{
    S_RTC_TIME_DATA_T sCurTime;

    /* tick interrupt */
    if ( (RTC->RIER & RTC_RIER_TIER_Msk) && (RTC->RIIR & RTC_RIIR_TIF_Msk) )
    {
        /* Get the current time */
        RTC_GetDateAndTime(&sCurTime);
        rtc = 1;
        // Clear interrupt
        RTC->RIIR = 0x2;
    }
    /* alarm interrupt */
    if ( (RTC->RIER & RTC_RIER_AIER_Msk) && (RTC->RIIR & RTC_RIIR_AIF_Msk) )
    {
        alarm = 1;
        // Clear interrupt
        RTC->RIIR = 0x1;
    }
}

void WDT_IRQHandler(void)
{
    SYS_UnlockReg();
    // Clear WDT interrupt flag
    WDT_CLEAR_TIMEOUT_INT_FLAG();
    SYS_LockReg();

    wdt = 1;
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
    /* Enable LIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRC_EN_Msk);
    /* Enable LXT */
    CLK_EnableXtalRC(CLK_PWRCTL_LXT_EN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);
    /* Waiting for LIRC clock stable */
    CLK_WaitClockReady( CLK_CLKSTATUS_LIRC_STB_Msk);
    /* Waiting for LXT clock stable */
    CLK_WaitClockReady( CLK_CLKSTATUS_LXT_STB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_HCLK_CLK_DIVIDER(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(RTC_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0);
    CLK_SetModuleClock(WDT_MODULE, 0, 0);
    CLK_SetModuleClock(RTC_MODULE, 0, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPA.14 and GPA.15 multi-function pins for UART0 RXD and TXD */
    SYS->PA_H_MFP &= ~(SYS_PA_H_MFP_PA14_MFP_Msk | SYS_PA_H_MFP_PA15_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA14_MFP_UART0_RX | SYS_PA_H_MFP_PA15_MFP_UART0_TX);

    /* Lock protected registers */
    SYS_LockReg();
}




/*----------------------------------------------------------------------------
  MAIN function
  ----------------------------------------------------------------------------*/
int32_t main (void)
{
    S_RTC_TIME_DATA_T sInitTime;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user wants to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Clear console */
    CLS();

    printf("Nuvoton Timer WDT RTC demo code\n");

    ////        Configure Timer 0       ////
    // Set timer0 frequency to 1Hz
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_Start(TIMER0);
    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    SYS_UnlockReg();

    ////        Configure WDT       ////
    // WDT Timeout about 6.6 sec, disable WDT reset and wake up function
    WDT_Open(WDT_TIMEOUT_2POW16, 0, FALSE, FALSE);

    // Enable WDT interrupt
    WDT_EnableInt();
    NVIC_EnableIRQ(WDT_IRQn);

    SYS_LockReg();

    ////        Configure RTC       ////
    /* Time/Date Setting */
    sInitTime.u32Year       = 2013;
    sInitTime.u32Month      = 10;
    sInitTime.u32Day        = 15;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_TUESDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    RTC_Open(&sInitTime);

    /* RTC alarm time setting, 10 sec from now */
    sInitTime.u32Second = sInitTime.u32Second + 10;
    /* Set the alarm time */
    RTC_SetAlarmDateAndTime(&sInitTime);

    /* Enable RTC alarm and tick interrupt */
    RTC_EnableInt(RTC_RIER_AIER_Msk | RTC_RIER_TIER_Msk);
    NVIC_EnableIRQ(RTC_IRQn);


    while(1)
    {
        if(alarm == 1)
        {
            printf("Alarm!!!\n");
            alarm = 0;
        }
        if(wdt == 1)
        {
            printf("WDT int\n");
            wdt = 0;
        }
        if(tmr == 1)
        {
            printf("Timer:%d\n", TimerCounter);
            tmr = 0;
        }
        if(rtc == 1)
        {
            printf("%s\n", string);
            rtc = 0;
        }

    }
}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


