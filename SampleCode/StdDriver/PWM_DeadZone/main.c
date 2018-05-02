/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 14/09/11 5:46p $
 * @brief    Demonstrate the dead-zone feature with PWM0.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

void PWM0_IRQHandler(void);

void PWM0_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    // Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times.
    if(++cnt == 100)
    {
        if(out)
            PWM_EnableOutput(PWM0, 0x3F);
        else
            PWM_DisableOutput(PWM0, 0x3F);
        out ^= 1;
        cnt = 0;
    }
    // Clear channel 0 period interrupt flag
    PWM_ClearPeriodIntFlag(PWM0, 0);
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

    /* Set PA 12,13,14,15 multi-function pins for PWM0 channel 0,1,2,3 */
    SYS->PA_H_MFP = (SYS->PA_H_MFP & ~(SYS_PA_H_MFP_PA12_MFP_Msk | SYS_PA_H_MFP_PA13_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk | SYS_PA_H_MFP_PA15_MFP_Msk));
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA12_MFP_PWM0_CH0 | SYS_PA_H_MFP_PA13_MFP_PWM0_CH1 | SYS_PA_H_MFP_PA14_MFP_PWM0_CH2 | SYS_PA_H_MFP_PA15_MFP_PWM0_CH3);

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

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\nThis sample code will output PWM0 channel 0 to with different\n");
    printf("frequency and duty, enable dead zone function of all PWM0 pairs.\n");
    printf("And also enable/disable PWM0 output every 1 second.\n");
    // PWM0 frequency is 100Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM0, 0, 100, 30);
    PWM_EnableDeadZone(PWM0, 0, 400);

    // PWM2 frequency is 300Hz, duty 50%
    PWM_ConfigOutputChannel(PWM0, 2, 300, 50);
    PWM_EnableDeadZone(PWM0, 2, 200);

    // Enable output of all PWM channels
    PWM_EnableOutput(PWM0, 0xF);

    // Enable PWM channel 0 period interrupt, use channel 0 to measure time.
    PWM_EnablePeriodInt(PWM0, 0, 0);
    NVIC_EnableIRQ(PWM0_IRQn);

    // Start
    PWM_Start(PWM0, 0xF);

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


