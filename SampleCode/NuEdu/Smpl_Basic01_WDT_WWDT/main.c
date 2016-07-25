/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/09/18 11:49a $
 * @brief    Demonstrate the WDT and WWDT application via NuEdu-EVB-Nano130 and NuEdu-Basci01 boards.
 *           The buzzer will beep when WDT interrupt or WWDT interrupt occurs.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "NuEdu-Basic01.h"

#define WDT_alarm 1
#define WWDT_alarm 2
#define initial_Buzzer() GPIO_SetMode(PC, BIT13, GPIO_PMD_OUTPUT)
#define RKEY_INPUT PB14
#define BKEY_INPUT PD12
#define Buzzer_ON  PC13=0
#define Buzzer_OFF PC13=1
#define RLED_ON    LED_on(1);
#define GLED_ON    LED_on(4);
#define RLED_OFF   LED_on(0);
#define GLED_OFF   LED_on(0);

uint32_t alarm_time=1000000;    //1s
uint32_t WDT_wait;
uint32_t WWDT_wait=0;
uint32_t Alarm=0;

void WDT_Buzzer(void)
{
    Buzzer_ON;
    RLED_ON;
    CLK_SysTickDelay(alarm_time);
    Buzzer_OFF;
    RLED_OFF;
}
void WWDT_Buzzer(void)
{
    Buzzer_ON;
    GLED_ON;
    CLK_SysTickDelay(alarm_time);
    Buzzer_OFF;
    GLED_OFF;
}
void WDT_IRQHandler(void)
{
    /*WDT Interrupt Status is Set*/
    if(WDT_GET_TIMEOUT_INT_FLAG()) {
        // Reset WDT and clear time out flag
        WDT_CLEAR_TIMEOUT_INT_FLAG();
        Alarm=WDT_alarm;
        printf("WDT interrupt !!!\n");
        WDT_RESET_COUNTER();            //Reset WDT Timer Counter
        WDT_wait=0;
    }

    /*WWDT Interrupt Status is Set*/
    if(WWDT_GET_INT_FLAG()) {
        // Reset WWDT and clear time out flag
        WWDT_CLEAR_INT_FLAG();
        if(WWDT_wait==1) {                  //Check black key status
            Alarm=WWDT_alarm;
            printf("WWDT interrupt !!!\n");
            WWDT_wait=0;
        }
        WWDT_RELOAD_COUNTER();      //Reset WWDT Timer Counter
    }
}

void initial_KEY_INPUT(void)
{
    GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);
    GPIO_SetMode(PD, BIT12, GPIO_PMD_INPUT);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /* Enable external 12MHz HXT, 32KHz LXT and HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk | CLK_PWRCTL_LXT_EN_Msk | CLK_PWRCTL_HIRC_EN_Msk);

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);
    SystemCoreClockUpdate();

    /*Set UART clock*/
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_UART_CLK_DIVIDER(1));

    /*Set WDT clock*/
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_SetModuleClock(WDT_MODULE, 0, 0);

    /* Set PC.10 and PC.11 multi-function pins for UART0 RXD, UART0 TXD */
    SYS->PC_H_MFP &= ~( SYS_PC_H_MFP_PC11_MFP_Msk | SYS_PC_H_MFP_PC10_MFP_Msk);
    SYS->PC_H_MFP |= (SYS_PC_H_MFP_PC11_MFP_UART1_TX | SYS_PC_H_MFP_PC10_MFP_UART1_RX);

    initial_Buzzer();
    initial_led();
    initial_KEY_INPUT();

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    //Initial System
    SYS_Init();
    SYS_UnlockReg();

    UART_Open(UART1, 115200);
    WDT_Open(WDT_TIMEOUT_2POW14,WDT_RESET_DELAY_3CLK,FALSE,FALSE);  //Initial WDT (timeout interval 1.638 s)
    WWDT_Open(WWDT_PRESCALER_768, 0x20, TRUE);      //Initial WWDT (timeout interval 2.4 s)
    WDT_EnableInt();                                                            //Enable WDT (share with WWDT) timeout interrupt
    NVIC_EnableIRQ(WDT_IRQn);

    printf("WDT & WWDT Sample code!!!\n");
    while(1) {
        if(RKEY_INPUT==0) {
            WDT_wait=1;
            while(WDT_wait);            //Wait for WDT interrupt
        }
        if(BKEY_INPUT==0) {
            WWDT_wait=1;                    //Wait for WWDT interrupt
        }

        /*Check WDT or WWDT alarm*/
        if(Alarm==WDT_alarm) {
            WDT_Buzzer();
            Alarm=0;
        } else if(Alarm==WWDT_alarm) {
            WWDT_Buzzer();
            Alarm=0;
        }
        WDT_RESET_COUNTER();            // Reset WDT
    }
}
