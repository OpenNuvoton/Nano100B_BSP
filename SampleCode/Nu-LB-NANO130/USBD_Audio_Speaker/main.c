/******************************************************************************
 * @file     main.c
 * @brief    Demonstrate how to implement a USB audio class device. 
 *           NAU8822 is used in this sample code to play the audio data from Host. 
 *           It also supports to record data from NAU8822 to Host.
 * @version  1.0.0
 * @date     23, December, 2013
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "usbd_audio.h"


/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12MHz HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk);
    CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_HXT, 96000000);
    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_HXT_STB_Msk | CLK_CLKSTATUS_PLL_STB_Msk);

    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_HCLK_CLK_DIVIDER(3));

    /* Select IP clock source */
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_USB_CLK_DIVIDER(2));
    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(I2S_MODULE,CLK_CLKSEL2_I2S_S_HXT, CLK_I2S_CLK_DIVIDER(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2S_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX|SYS_PA_H_MFP_PA14_MFP_UART0_RX);

    /* Set PD/PE multi-function pins for I2S */
    SYS->PD_L_MFP = (SYS_PD_L_MFP_PD2_MFP_I2S_WS | SYS_PD_L_MFP_PD3_MFP_I2S_BCLK | SYS_PD_L_MFP_PD4_MFP_I2S_DIN | SYS_PD_L_MFP_PD5_MFP_I2S_DOUT);
    SYS->PE_L_MFP = SYS_PE_L_MFP_PE0_MFP_I2S_MCLK;

    /* Set PA multi-function pins for I2C0 */
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA8_MFP_I2C0_SDA | SYS_PA_H_MFP_PA9_MFP_I2C0_SCL);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    UART0->BAUD = 0x67;              /* Baud Rate:115200  OSC:12MHz */
    UART0->TLCTL = 0x03;             /* Character len is 8 bits */
}

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
//    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C0 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, I2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, I2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, I2C_GCMODE_DISABLE);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, I2C_GCMODE_DISABLE);   /* Slave Address : 0x75 */
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    SYS_Init();

    /* Init I2C0 to access WAU8822 */
    I2C0_Init();
    I2S_Open(I2S, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_I2S);
    /* select source from HXT(12MHz) */
    CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HXT, 0);
    /* Initialize WAU8822 codec */
    WAU8822_Setup();

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S, 12000000);

    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_SetPriority(TMR0_IRQn, 3);
    NVIC_EnableIRQ(TMR0_IRQn);

    USBD_Open(&gsInfo, UAC_ClassRequest, UAC_SetInterface);
    /* Endpoint configuration */
    UAC_Init();
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();

    while(1);
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

