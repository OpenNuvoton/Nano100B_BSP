/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 14/09/11 5:32p $
 * @brief    An I2C master/slave demo by connecting I2C0 and I2C1 interface.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

#define SLAVE_ADDRESS 0x16

typedef void (*I2C_FUNC)(uint32_t u32Status);
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr;
uint32_t g_u32SlaveBuffAddr;
uint8_t g_au8MasterTxData[3];
uint8_t g_u8SlvData[256];
uint8_t g_u8MasterRxData;
uint8_t g_au8SlaveRxData[3];
volatile uint8_t g_u8MasterDataLen;
uint8_t g_u8SlaveDataLen;
volatile uint8_t g_u8EndFlag = 0;

volatile I2C_FUNC s_I2C0HandlerFn = NULL;
volatile I2C_FUNC s_I2C1HandlerFn = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    // clear interrupt flag
    I2C0->INTSTS = I2C_INTSTS_INTSTS_Msk;

    u32Status = I2C_GET_STATUS(I2C0);
    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    // clear interrupt flag
    I2C1->INTSTS = I2C_INTSTS_INTSTS_Msk;

    u32Status = I2C_GET_STATUS(I2C1);
    if (I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C1);
    }
    else
    {
        if (s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1)); /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MasterTxData[g_u8MasterDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MasterDataLen != 2)
        {
            I2C_SET_DATA(I2C0, g_au8MasterTxData[g_u8MasterDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_SI);
        }
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) | 0x01);  /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    }
    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    }
    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
    {
        g_u8MasterRxData = I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
        g_u8EndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);  /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MasterTxData[g_u8MasterDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MasterDataLen != 3)
        {
            I2C_SET_DATA(I2C0, g_au8MasterTxData[g_u8MasterDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
            g_u8EndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlaveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_SI | I2C_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8SlaveRxData[g_u8SlaveDataLen] = I2C_GET_DATA(I2C1);
        g_u8SlaveDataLen++;

        if (g_u8SlaveDataLen == 2)
        {
            g_u32SlaveBuffAddr = (g_au8SlaveRxData[0] << 8) + g_au8SlaveRxData[1];
        }
        if (g_u8SlaveDataLen == 3)
        {
            g_u8SlvData[g_u32SlaveBuffAddr] = g_au8SlaveRxData[2];
            g_u8SlaveDataLen = 0;
        }
        I2C_SET_CONTROL_REG(I2C1, I2C_SI | I2C_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {

        I2C_SET_DATA(I2C1, g_u8SlvData[g_u32SlaveBuffAddr]);
        g_u32SlaveBuffAddr++;
        I2C_SET_CONTROL_REG(I2C1, I2C_SI | I2C_AA);
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_SI | I2C_AA);
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlaveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_SI | I2C_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlaveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_SI | I2C_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
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

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(I2C0_MODULE, 0, 0);
    CLK_SetModuleClock(I2C1_MODULE, 0, 0);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD  */
    SYS->PA_H_MFP = (SYS_PA_H_MFP_PA14_MFP_UART0_RX | SYS_PA_H_MFP_PA15_MFP_UART0_TX);

    /* Set multi function pin for I2C0/I2C1 */
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA9_MFP_I2C0_SCL | SYS_PA_H_MFP_PA8_MFP_I2C0_SDA | SYS_PA_H_MFP_PA11_MFP_I2C1_SCL | SYS_PA_H_MFP_PA10_MFP_I2C1_SDA);

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
    /* Open I2C1 and set clock to 100k */
    I2C_Open(I2C1, 400000);

    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", I2C_GetBusClockFreq(I2C1));

    /* Set I2C1 2 Slave Addresses */
    I2C_SetSlaveAddr(I2C1, 0, SLAVE_ADDRESS, I2C_GCMODE_DISABLE);   /* Set Slave Address */

    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_SI | I2C_AA);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-------------------------------------------------------+\n");
    printf("|      Nano100 Series I2C Cross Test Sample Code        |\n");
    printf("+-------------------------------------------------------+\n");
    printf("  I/O Configuration:\n");
    printf("    GPA8 <--> GPA10\n");
    printf("    GPA9 <--> GPA11\n");
    printf("\n");
    printf("..... Press a key to continue ...\n");
    getchar();


    /* Configure I2C0 as master and I2C1 as slave */
    I2C0_Init();
    I2C1_Init();

    for (i = 0; i < 0x100; i++)
    {
        g_u8SlvData[i] = 0;
    }

    /* I2C function to Slave receive/transmit data */
    s_I2C1HandlerFn = (I2C_FUNC)I2C_SlaveTRx;

    g_u8DeviceAddr = SLAVE_ADDRESS;

    printf("Test Loop =>");
    for (i = 0; i < 0x100; i++)
    {
        printf("%d..", i);
        g_au8MasterTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8MasterTxData[1] = (uint8_t)(i & 0x00FF);
        g_au8MasterTxData[2] = (uint8_t)(g_au8MasterTxData[1] + 3);

        g_u8MasterDataLen = 0;
        g_u8EndFlag = 0;

        /* I2C function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA);

        /* Wait I2C Tx Finish */
        while (g_u8EndFlag == 0);
        g_u8EndFlag = 0;

        /* I2C function to read data from slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;

        g_u8MasterDataLen = 0;
        g_u8DeviceAddr = SLAVE_ADDRESS;

        /* I2C as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA);

        /* Wait I2C Rx Finish */
        while (g_u8EndFlag == 0);

        /* Compare Tx and Rx data */
        if (g_u8MasterRxData != g_au8MasterTxData[2])
        {
            printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MasterRxData);
            return -1;
        }
        printf("[OK]\n");
    }

    printf("\nTest Completely !!\n");
    while(1);
}



