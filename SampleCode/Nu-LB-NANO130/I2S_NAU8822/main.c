/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 6 $
 * $Date: 15/06/12 4:33p $
 * @brief    An I2S demo using NAU8822 audio codec, used to play back the
 *           input from line-in or MIC interface.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "Nano100Series.h"

#define NAU8822_ADDR    0x1A                /* NAU8822 Device ID */

uint32_t PcmBuff[BUFF_LEN] = {0};
uint32_t volatile u32BuffPos = 0;

void Delay(int count)
{
    volatile uint32_t i;
    for (i = 0; i < count ; i++);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of WAU8822 with I2C0                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data)
{
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, NAU8822_ADDR<<1);
    I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    I2C_WAIT_READY(I2C0);

    I2C_STOP(I2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  WAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void WAU8822_Setup()
{
    printf("\nConfigure WAU8822 ...");

    I2C_WriteWAU8822(0,  0x000);   /* Reset all registers */
    Delay(0x200);

#ifdef INPUT_IS_LIN //input source is LIN
    I2C_WriteWAU8822(1,  0x02F);
    I2C_WriteWAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteWAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteWAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteWAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteWAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteWAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteWAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteWAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteWAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteWAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteWAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteWAU8822(47, 0x050);   /* LLIN connected, and its Gain value */
    I2C_WriteWAU8822(48, 0x050);   /* RLIN connected, and its Gain value */
    I2C_WriteWAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteWAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#else   //input source is MIC
    I2C_WriteWAU8822(1,  0x03F);
    I2C_WriteWAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteWAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteWAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteWAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteWAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteWAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteWAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteWAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteWAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteWAU8822(16, 0x1EF);   /* ADC right digital volume control */
    I2C_WriteWAU8822(44, 0x033);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteWAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteWAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#endif

    printf("[OK]\n");
}

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
    CLK_EnableModuleClock(I2S_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART_S_HIRC,CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(I2S_MODULE,CLK_CLKSEL2_I2S_S_HXT, CLK_I2S_CLK_DIVIDER(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->PA_H_MFP = (SYS_PA_H_MFP_PA14_MFP_UART0_RX | SYS_PA_H_MFP_PA15_MFP_UART0_TX);

    /* Set PD/PE multi-function pins for I2S */
    SYS->PD_L_MFP = (SYS_PD_L_MFP_PD2_MFP_I2S_WS | SYS_PD_L_MFP_PD3_MFP_I2S_BCLK | SYS_PD_L_MFP_PD4_MFP_I2S_DIN | SYS_PD_L_MFP_PD5_MFP_I2S_DOUT);
    SYS->PE_L_MFP = SYS_PE_L_MFP_PE0_MFP_I2S_MCLK;

    /* Set PA multi-function pins for I2C0 */
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA8_MFP_I2C0_SDA | SYS_PA_H_MFP_PA9_MFP_I2C0_SCL);

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

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
    uint32_t u32startFlag = 1;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   I2S Driver Sample Code with WAU8822                  |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with WAU8822.\n");

    /* Init I2C0 to access WAU8822 */
    I2C0_Init();

#ifdef INPUT_IS_LIN
    I2S_Open(I2S, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_I2S);
#else
    I2S_Open(I2S, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, I2S_MONO, I2S_FORMAT_I2S, I2S_I2S);
#endif
    NVIC_EnableIRQ(I2S_IRQn);

    // select source from HXT(12MHz)
    CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HXT, 0);

    /* Initialize WAU8822 codec */
    WAU8822_Setup();

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S, 12000000);

#ifndef INPUT_IS_LIN
    I2S_SET_MONO_RX_CHANNEL(I2S, I2S_MONO_LEFT);       //WAU8822 will store data in left channel
#endif

    /* Enable Rx threshold level interrupt */
    I2S_EnableInt(I2S, I2S_INTEN_RXTHIE_Msk);

    /* Enable I2S Rx function to receive data */
    I2S_ENABLE_RX(I2S);

    while(1) {
        if (u32startFlag) {
            /* Enable I2S Tx function to send data when data in the buffer is more than half of buffer size */
            if (u32BuffPos >= BUFF_LEN/2) {
                I2S_EnableInt(I2S, I2S_INTEN_TXTHIE_Msk);
                I2S_ENABLE_TX(I2S);
                u32startFlag = 0;
            }
        }
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
