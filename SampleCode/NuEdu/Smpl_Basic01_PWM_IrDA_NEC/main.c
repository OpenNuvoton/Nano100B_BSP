/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/06/18 9:09a $
 * @brief    Demonstrate remote control function based on NEC IR protocol
 *           and changes LED display via NuEdu-EVB-Nano130 and NuEdu-Basci01 boards
 *           when system receives data of NEC IR.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdarg.h>

#include "Nano100Series.h"
#include "NuEdu-Basic01.h"

uint32_t u32LEDEanble;
void DecodeIRcmd(volatile uint8_t*  IR_CODE1)
{
    if((IR_CODE1[0] == 0x00)& (IR_CODE1[1] == 0xFF))
    {
        if((IR_CODE1[2] == 0x10)& (IR_CODE1[3] == 0xEF))
        {
            LED_on(++u32LEDEanble);
        }
        else if((IR_CODE1[2] == 0x14)& (IR_CODE1[3] == 0xEB))
        {
            LED_on(--u32LEDEanble);
        }
    }
}

void PWM1_IRQHandler(void)
{
    uint32_t TDR1_tmp;
    TDR1_tmp = MaxValue-PWM_GET_CAPTURE_FALLING_DATA(PWM1,3);
    PWM_ClearCaptureIntFlag(PWM1, 3, PWM_FALLING_LATCH_INT_FLAG);
    IrDa_NEC_Rx(TDR1_tmp);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Select IP clock source */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_UART_CLK_DIVIDER(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PC.10, PC.11 and PC.14 multi-function pins for UART1 RXD, UART1 TXD and PWM1_CH3*/
    SYS->PC_H_MFP = (SYS_PC_H_MFP_PC11_MFP_UART1_TX | SYS_PC_H_MFP_PC10_MFP_UART1_RX);

    /* Lock protected registers */
    SYS_LockReg();
}



/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main()
{
    uint32_t u32Key;
    uint8_t au8IR_CODE[4];
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init IrDA, Key and LED GPIO type */
    GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);
    Initial_KEY_INPUT();
    initial_led();
    IrDA_NEC_TxRx_Init(DecodeIRcmd);
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART1, 115200);

    printf("+-----------------------------------------+\n");
    printf("|    Nano100 Series IrDA NEC Sample Code      |\n");
    printf("+-----------------------------------------+\n");

    au8IR_CODE[0] = 0x00;
    au8IR_CODE[1] = ~au8IR_CODE[0];

    while(1)
    {
        /* Detect Key status */
        u32Key = Get_KEY_INPUT();
        if(PB14==0)
        {
            au8IR_CODE[2] = 0x10;
            au8IR_CODE[3] = ~au8IR_CODE[2];
            SendNEC(au8IR_CODE);
            CLK_SysTickDelay(100000);
        }
        if((u32Key&0x01)==0)
        {
            au8IR_CODE[2] = 0x14;
            au8IR_CODE[3] = ~au8IR_CODE[2];
            SendNEC(au8IR_CODE);
            CLK_SysTickDelay(100000);
        }

    }
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
