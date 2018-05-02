/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/18 11:49a $
 * @brief    Demonstrate how to access EEPROM through I2C interface
 *           and print the test results on PC via NUCOM1 port of the NuEdu-Basic01 board.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXT_EN_Msk;

    /* Waiting for 12MHz clock ready */
    while((CLK->CLKSTATUS & CLK_CLKSTATUS_HXT_STB_Msk) == 0) {};

    /* Switch PLL clock source to XTAL */
    CLK->PLLCTL &= CLK_PLLCTL_PLL_SRC_HXT;

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL &= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency 84 MHz */
    CLK->PLLCTL |= 0x0218;

    /* Waiting for clock ready */
    while((CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk) == 0) {};

    /* Set HCLK_N = 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLK_N_Msk) | 0x01;

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();

}


/*---------------------------------------------------------------------------------------------------------*/
/* Init UART1                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{
    /* Set PC.10 and PC.11 multi-function pins for UART1_RXD, UART1_TXD */
    SYS->PC_H_MFP |= (SYS_PC_H_MFP_PC11_MFP_UART1_TX | SYS_PC_H_MFP_PC10_MFP_UART1_RX);

    /* Enable UART1 IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART1_EN_Msk;

    /* Set UART1 IP clock source */
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* Configure UART1 and set UART1 Baudrate */
    UART1->BAUD = UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(FREQ_12MHZ, 115200);
    UART1->TLCTL = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main()
{
    uint32_t u32Data,i;

    /* Initial system */
    SYS_Init();

    /* Initial UART1 to 115200-8n1 for print message */
    UART1_Init();

    printf("Hello World.\n");
    printf("PLL Clock = %d Hz\n", CLK_GetPLLClockFreq());
    printf("Core Clock = %d Hz\n\n", CLK_GetHCLKFreq());
    printf("+-------------------------------------------------------+\n");
    printf("|   Nano100 Series I2C Sample Code with EEPROM 24LC64   |\n");
    printf("+-------------------------------------------------------+\n");

    /* Initial I2C */
    I2C_EEPROM_Init(8);

    /* I2C EEPROM Write/Read test */
    for (i = 0; i < 2; i++)
    {
        printf("\n\nAddress = 0x0010, Write Data = %xh", (i*2+3));
        I2C_EEPROM_Write(0x0010,(i*2+3));

        u32Data = I2C_EEPROM_Read(0x0010);
        printf("\nAddress = 0x0010, Read Data = %xh", u32Data);
        if(u32Data!=(i*2+3))
        {
            printf("I2C Byte Write/Read Failed, Data 0x%x\n", u32Data);
            return -1;
        }
    }

    printf("\n\nI2C Access EEPROM Test OK\n");

    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
