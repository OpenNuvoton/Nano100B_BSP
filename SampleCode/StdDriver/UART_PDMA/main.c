/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/26 2:00p $
 * @brief    Demonstrate UART transmit and receive function with PDMA.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Nano100Series.h"

#include "uart.h"

#define TX_CH     1
#define RX_CH     2

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t TX_Buffer[100];
uint8_t RX_Buffer[100];
uint32_t volatile u32IsTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART1_TEST_HANDLE(void);
void UART_PDMATest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HXT_EN_Pos); // HXT Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable
    CLK->APBCLK |= CLK_APBCLK_UART1_EN; // UART1 Clock Enable
    CLK->AHBCLK |= CLK_AHBCLK_DMA_EN;   // DMA Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX);

    /* Set PB multi-function pins for UART1 RXD, TXD, RTS, CTS  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB4_MFP_Msk | SYS_PB_L_MFP_PB5_MFP_Msk |
                       SYS_PB_L_MFP_PB6_MFP_Msk | SYS_PB_L_MFP_PB7_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB4_MFP_UART1_RX | SYS_PB_L_MFP_PB5_MFP_UART1_TX |
                      SYS_PB_L_MFP_PB6_MFP_UART1_RTS  | SYS_PB_L_MFP_PB7_MFP_UART1_CTS);

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

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART1, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART0_Init();

	/* Init UART1 for test */
	UART1_Init();
	
    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------+\n");
    printf("| UART PDMA function test  |\n");
    printf("+--------------------------+\n");

    UART_PDMATest();
	
    while(1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART1_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE()
{
    uint32_t u32IntSts= UART1->ISR;

    if(u32IntSts & UART_ISR_RLS_IS_Msk)
    {
        printf("\n UART1 Receive Line error!!");
        UART1->FSR = (UART_FSR_PE_F_Msk | UART_FSR_FE_F_Msk | UART_FSR_BI_F_Msk);
    }

    if(u32IntSts & UART_ISR_BUF_ERR_IS_Msk)
    {
        printf("\n UART1 Buffer Overflow error");
        UART1->FSR = (UART_FSR_RX_OVER_F_Msk | UART_FSR_TX_OVER_F_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle PDMA interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if (status & 0x2) { /* CH1 */
        if (PDMA_GET_CH_INT_STS(1) & 0x2)
            u32IsTestOver = 1;
        PDMA_CLR_CH_INT_FLAG(1, PDMA_ISR_TD_IS_Msk);
    } else if (status & 0x4) { /* CH2 */
        if (PDMA_GET_CH_INT_STS(2) & 0x2)
            u32IsTestOver = 2;
        PDMA_CLR_CH_INT_FLAG(2, PDMA_ISR_TD_IS_Msk);
    }else
        printf("unknown interrupt !!\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PDMATest()
{
    uint32_t i;
    uint32_t TEST_SIZE = 100;
	
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART PDMA Test                                           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo uart PDMA function.          |\n");
    printf("|    Please connect UART1 Tx, Rx pin.                       |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        connect UART1 Tx, Rx pin.
        UART1 will transfer data from Tx PDMA buffer.
        And will receive data to Rx PDMA buffer.
        This test function will compare Tx and Rx buffer data.
    */

    for(i = 0; i < TEST_SIZE; i++)
    {
        TX_Buffer[i] = (i & 0xff);
        RX_Buffer[i] = 0;
    }

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART1, (UART_IER_RLS_IE_Msk | UART_IER_BUF_ERR_IE_Msk) );
    NVIC_EnableIRQ(UART1_IRQn);

    // Rx PDMA Setting
    PDMA_Open(1 << RX_CH);
    PDMA_SetTransferCnt(RX_CH, PDMA_WIDTH_8, TEST_SIZE);
    PDMA_SetTransferAddr(RX_CH, UART1_BASE, PDMA_SAR_FIX, (uint32_t)RX_Buffer, PDMA_DAR_INC);
    PDMA_SetTransferMode(RX_CH, PDMA_UART1_RX, 0, 0);
    PDMA_SetTimeOut(RX_CH, 0, 0x5555);
    PDMA_EnableInt(RX_CH, PDMA_IER_TD_IE_Msk);
    NVIC_EnableIRQ(PDMA_IRQn);
    u32IsTestOver = 0;
    	
    // Tx PDMA Setting
    PDMA_Open(1 << TX_CH);
    PDMA_SetTransferCnt(TX_CH, PDMA_WIDTH_8, TEST_SIZE);
    PDMA_SetTransferAddr(TX_CH, (uint32_t)TX_Buffer, PDMA_SAR_INC, UART1_BASE, PDMA_DAR_FIX);
    PDMA_SetTransferMode(TX_CH, PDMA_UART1_TX, 0, 0);
    PDMA_SetTimeOut(TX_CH, 0, 0x5555);
    PDMA_EnableInt(TX_CH, PDMA_IER_TD_IE_Msk);
    NVIC_EnableIRQ(PDMA_IRQn);
    u32IsTestOver = 0;

    PDMA_Trigger(RX_CH);
    PDMA_Trigger(TX_CH);

    UART1->CTL |= UART_CTL_DMA_RX_EN_Msk;
    UART1->CTL |= UART_CTL_DMA_TX_EN_Msk;

    while(u32IsTestOver == 0);

    for(i = 0; i < TEST_SIZE; i++)
    {
        if(TX_Buffer[i] != RX_Buffer[i])
        {
            printf("\n Test fail!!");
            while(1);
        }
    }

    printf("\n Tx/Rx data compare pass!!");
    
    /* Disable Interrupt */
    UART_DISABLE_INT(UART1, (UART_IER_RLS_IE_Msk | UART_IER_BUF_ERR_IE_Msk));
    NVIC_DisableIRQ(UART1_IRQn);

    printf("\n UART PDMASample Demo End.\n");

}


