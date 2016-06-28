
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 14/02/10 1:57p $
 * @brief    PWM sample for NANO100 series MCU
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

#define CLS       do {printf("%c[2J", 0x1B); printf("%c[%d;1H", 0x1B, 0);}while(0)
#define JMP(x)    do {printf("%c[0J", 0x1B); printf("%c[%d;1H", 0x1B, x);}while(0)

static uint32_t CaptureValue[2];

void PWM0_IRQHandler(void);

void PWM0_IRQHandler(void)
{
    uint32_t u32CapIntFlag;

    // Get channel 2 capture interrupt flag
    u32CapIntFlag = PWM_GetCaptureIntFlag(PWM0, 2);

    // Falling latch condition happened
    if ((u32CapIntFlag & PWM_FALLING_LATCH_INT_FLAG)) {
        CaptureValue[0]=(0xFFFF - PWM_GET_CAPTURE_FALLING_DATA(PWM0, 2));//usec
    }

    // Rising latch condition happened
    if ((u32CapIntFlag & PWM_RISING_LATCH_INT_FLAG)) {
        CaptureValue[1]=(0xFFFF - PWM_GET_CAPTURE_RISING_DATA(PWM0, 2));//usec
    }

    // Clear channel 2 capture interrupt flag
    PWM_ClearCaptureIntFlag(PWM0, 2, PWM_RISING_FALLING_LATCH_INT_ENABLE);
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
    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA14_MFP_Msk | SYS_PA_H_MFP_PA15_MFP_Msk);
    SYS->PA_H_MFP = (SYS_PA_H_MFP_PA14_MFP_UART0_RX | SYS_PA_H_MFP_PA15_MFP_UART0_TX);

    /* Set PB multi-function pins for Clock Output */
    SYS->PB_H_MFP = ( SYS->PB_H_MFP & ~SYS_PB_H_MFP_PB12_MFP_Msk ) |  SYS_PB_H_MFP_PB12_MFP_CKO;

    /* Set PC.6 and PA.7 multi-function pins for PWM channel 0 and 2 */
    SYS->PC_L_MFP = (SYS->PC_L_MFP & ~SYS_PC_L_MFP_PC6_MFP_Msk) | SYS_PC_L_MFP_PC6_MFP_PWM0_CH0;
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA7_MFP_Msk) | SYS_PA_L_MFP_PA7_MFP_PWM0_CH2;

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
    char string[18];

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    CLS;
    printf("\nPWM0 channel 2 will capture the output of PWM0 channel 0\n");
    printf("So, please connect GPIO port C6 with A7.\n");
    // PWM0 frequency is 25000Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM0, 0, 25000, 30);

    // PWM2
    PWM_ConfigCaptureChannel(PWM0,2,50,0);

    // Reload CNR2 when channel 2 rising and falling capture event comes
    PWM0->CAPCTL |= (PWM_CAPCTL_CAPRELOADREN2_Msk | PWM_CAPCTL_CAPRELOADREN2_Msk);

    // Enable output of channel 0
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    // Enable capture of channel 2
    PWM_EnableCapture(PWM0, PWM_CH_2_MASK);

    // Enable PWM channel 2 rising and falling edge capture interrupt
    PWM_EnableCaptureInt(PWM0,2,PWM_RISING_FALLING_LATCH_INT_ENABLE);
    NVIC_EnableIRQ(PWM0_IRQn);

    // Start
    PWM_Start(PWM0, (PWM_CH_0_MASK|PWM_CH_2_MASK));

    while(1) {
        JMP(5);

        if(CaptureValue[0]>=1000000) {
            sprintf(string,"%ds",CaptureValue[0]/1000000);
        } else if(CaptureValue[0]>=1000) {
            sprintf(string,"%dms",CaptureValue[0]/1000);
        } else
            sprintf(string,"%dus",CaptureValue[0]);

        printf("High: %s      \n", string);


        if(CaptureValue[1]>=1000000) {
            sprintf(string,"%ds",CaptureValue[1]/1000000);
        } else if(CaptureValue[1]>=1000) {
            sprintf(string,"%dms",CaptureValue[1]/1000);
        } else
            sprintf(string,"%dus",CaptureValue[1]);
        printf("Low:  %s      \n", string);
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


