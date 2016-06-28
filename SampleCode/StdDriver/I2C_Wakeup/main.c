/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 14/09/11 5:34p $
 * @brief    Demonstrate how to wake up system form Power-down mode by I2C interrupt.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void PDWU_IRQHandler(void)
{
    /* Clear I2C wake up flag */
    I2C_CLEAR_WAKEUP_FLAG(I2C0);

    /* Clear wake up flag */
    CLK->WK_INTSTS = 1;
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

    /* Enable external 12MHz HXT, 32KHz LXT and HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk | CLK_PWRCTL_LXT_EN_Msk | CLK_PWRCTL_HIRC_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_HXT_STB_Msk | CLK_CLKSTATUS_LXT_STB_Msk | CLK_CLKSTATUS_HIRC_STB_Msk);

    /* Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(I2C0_MODULE, 0, 0);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD  */
    SYS->PA_H_MFP = (SYS_PA_H_MFP_PA14_MFP_UART0_RX | SYS_PA_H_MFP_PA15_MFP_UART0_TX);

    /* Set multi function pin for I2C0 */
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA9_MFP_I2C0_SCL | SYS_PA_H_MFP_PA8_MFP_I2C0_SDA);

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Init Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C0 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x25, I2C_GCMODE_DISABLE);   /* Set Slave Address */

    NVIC_EnableIRQ(PDWU_IRQn);

    /* Enable I2C0 wakeup function */
    I2C_EnableWakeup(I2C0);

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    volatile uint8_t u8RxData[3];
    volatile uint8_t u8ReadWrite;
    volatile uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-------------------------------------------------------+\n");
    printf("|        Nano100 Series I2C Wake Up Sample Code         |\n");
    printf("+-------------------------------------------------------+\n");
    printf("Note: Master need to send the I2C signal likes the following :\n");
    printf("    S | SLA+W | P \n");
    printf(" or S | SLA+R | P \n\n");

    /* Init I2C0 as Slave */
    I2C0_Init();

    printf("Enter power down mode !!!\n");
    // Wait 'til UART FIFO empty to get a cleaner console out
    UART_WAIT_TX_EMPTY(UART0);

    /* Enter power down mode and enable wake up interrupt */
    SYS_UnlockReg();
    CLK->PWRCTL |= CLK_PWRCTL_PD_WK_IE_Msk;
    CLK_PowerDown();

    printf("Wake up !!\n");

    while(1);
}
