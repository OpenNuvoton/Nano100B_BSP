/**************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to implement a USB dual virtual com port device.
 * @version  2.0.0
 * @date     17, Dec, 2014
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "vcom_serial.h"

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding0 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING gLineCoding1 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal0 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t gCtrlSignal1 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

#define TX_FIFO_SIZE_0        16  /* TX Hardware FIFO size */
#define TX_FIFO_SIZE_1        16  /* TX Hardware FIFO size */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
volatile uint8_t comRbuf0[RXBUFSIZE];
volatile uint16_t comRbytes0 = 0;
volatile uint16_t comRhead0 = 0;
volatile uint16_t comRtail0 = 0;

volatile uint8_t comTbuf0[TXBUFSIZE];
volatile uint16_t comTbytes0 = 0;
volatile uint16_t comThead0 = 0;
volatile uint16_t comTtail0 = 0;

uint8_t gRxBuf0[64] = {0};
uint8_t *gpu8RxBuf0 = 0;
uint32_t gu32RxSize0 = 0;
uint32_t gu32TxSize0 = 0;

volatile int8_t gi8BulkOutReady0 = 0;

/* UART1 */
volatile uint8_t comRbuf1[RXBUFSIZE];
volatile uint16_t comRbytes1 = 0;
volatile uint16_t comRhead1 = 0;
volatile uint16_t comRtail1 = 0;

volatile uint8_t comTbuf1[TXBUFSIZE];
volatile uint16_t comTbytes1 = 0;
volatile uint16_t comThead1 = 0;
volatile uint16_t comTtail1 = 0;

uint8_t gRxBuf1[64] = {0};
uint8_t *gpu8RxBuf1 = 0;
uint32_t gu32RxSize1 = 0;
uint32_t gu32TxSize1 = 0;

volatile int8_t gi8BulkOutReady1 = 0;

/*---------------------------------------------------------------------------------------------------------*/
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
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX|SYS_PA_H_MFP_PA14_MFP_UART0_RX);

    /* Set PB multi-function pins for UART1 RXD, TXD  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB4_MFP_Msk | SYS_PB_L_MFP_PB5_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB4_MFP_UART1_RX | SYS_PB_L_MFP_PB5_MFP_UART1_TX);


    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    UART0->BAUD = 0x67;              /* Baud Rate:115200  OSC:12MHz */
    UART0->TLCTL = 0x03;             /* Character len is 8 bits */

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART0, (UART_IER_RDA_IE_Msk | UART_IER_THRE_IE_Msk | UART_IER_RTO_IE_Msk));
}

void UART1_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART1_RST);
    UART1->BAUD = 0x67;              /* Baud Rate:115200  OSC:12MHz */
    UART1->TLCTL = 0x03;             /* Character len is 8 bits */

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART1, (UART_IER_RDA_IE_Msk | UART_IER_THRE_IE_Msk | UART_IER_RTO_IE_Msk));
}


/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint8_t bInChar;
    int32_t size;
    uint32_t u32IntStatus;

    u32IntStatus = UART0->ISR;

    if((u32IntStatus & UART_ISR_RDA_IS_Msk) || (u32IntStatus & UART_ISR_RTO_IS_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while (!(UART0->FSR & UART_FSR_RX_EMPTY_F_Msk))
        {
            /* Get the character from UART Buffer */
            bInChar = UART0->RBR;

            /* Check if buffer full */
            if(comRbytes0 < RXBUFSIZE)
            {
                /* Enqueue the character */
                comRbuf0[comRtail0++] = bInChar;
                if(comRtail0 >= RXBUFSIZE)
                    comRtail0 = 0;
                comRbytes0++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_ISR_THRE_IS_Msk)
    {

        if(comTbytes0)
        {
            /* Fill the Tx FIFO */
            size = comTbytes0;
            if(size >= TX_FIFO_SIZE_0)
            {
                size = TX_FIFO_SIZE_0;
            }

            while(size)
            {
                bInChar = comTbuf0[comThead0++];
                UART0->THR = bInChar;
                if(comThead0 >= TXBUFSIZE)
                    comThead0 = 0;
                comTbytes0--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART0->IER &= ~UART_IER_THRE_IE_Msk;
        }
    }
}

void UART1_IRQHandler(void)
{
    uint8_t bInChar;
    int32_t size;
    uint32_t u32IntStatus;

    u32IntStatus= UART1->ISR;

    if((u32IntStatus & UART_ISR_RDA_IS_Msk) || (u32IntStatus & UART_ISR_RTO_IS_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while (!(UART1->FSR & UART_FSR_RX_EMPTY_F_Msk))
        {
            /* Get the character from UART Buffer */
            bInChar = UART1->RBR;

            /* Check if buffer full */
            if(comRbytes1 < RXBUFSIZE)
            {
                /* Enqueue the character */
                comRbuf1[comRtail1++] = bInChar;
                if(comRtail1 >= RXBUFSIZE)
                    comRtail1 = 0;
                comRbytes1++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_ISR_THRE_IS_Msk)
    {

        if(comTbytes1)
        {
            /* Fill the Tx FIFO */
            size = comTbytes1;
            if(size >= TX_FIFO_SIZE_1)
            {
                size = TX_FIFO_SIZE_1;
            }

            while(size)
            {
                bInChar = comTbuf1[comThead1++];
                UART1->THR = bInChar;
                if(comThead1 >= TXBUFSIZE)
                    comThead1 = 0;
                comTbytes1--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART1->IER &= ~UART_IER_THRE_IE_Msk;
        }
    }
}


void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check whether USB is ready for next packet or not*/
    if(gu32TxSize0 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(comRbytes0)
        {
            i32Len = comRbytes0;
            if(i32Len > EP2_MAX_PKT_SIZE)
                i32Len = EP2_MAX_PKT_SIZE;

            for(i = 0; i < i32Len; i++)
            {
                gRxBuf0[i] = comRbuf0[comRhead0++];
                if(comRhead0 >= RXBUFSIZE)
                    comRhead0 = 0;
            }

            __set_PRIMASK(1);
            comRbytes0 -= i32Len;
            __set_PRIMASK(0);

            gu32TxSize0 = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)gRxBuf0, i32Len);
            USBD_SET_PAYLOAD_LEN(EP2, i32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP2);
            if(i32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }

    if(gu32TxSize1 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(comRbytes1)
        {
            i32Len = comRbytes1;
            if(i32Len > EP7_MAX_PKT_SIZE)
                i32Len = EP7_MAX_PKT_SIZE;

            for(i = 0; i < i32Len; i++)
            {
                gRxBuf1[i] = comRbuf1[comRhead1++];
                if(comRhead1 >= RXBUFSIZE)
                    comRhead1 = 0;
            }

            __set_PRIMASK(1);
            comRbytes1 -= i32Len;
            __set_PRIMASK(0);

            gu32TxSize1 = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), (uint8_t *)gRxBuf1, i32Len);
            USBD_SET_PAYLOAD_LEN(EP7, i32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP7);
            if(i32Len == EP7_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP7, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if(gi8BulkOutReady0 && (gu32RxSize0 <= TXBUFSIZE - comTbytes0))
    {
        for(i = 0; i < gu32RxSize0; i++)
        {
            comTbuf0[comTtail0++] = gpu8RxBuf0[i];
            if(comTtail0 >= TXBUFSIZE)
                comTtail0 = 0;
        }

        __set_PRIMASK(1);
        comTbytes0 += gu32RxSize0;
        __set_PRIMASK(0);

        gu32RxSize0 = 0;
        gi8BulkOutReady0 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    if(gi8BulkOutReady1 && (gu32RxSize1 <= TXBUFSIZE - comTbytes1))
    {
        for(i = 0; i < gu32RxSize1; i++)
        {
            comTbuf1[comTtail1++] = gpu8RxBuf1[i];
            if(comTtail1>= TXBUFSIZE)
                comTtail1 = 0;
        }

        __set_PRIMASK(1);
        comTbytes1 += gu32RxSize1;
        __set_PRIMASK(0);

        gu32RxSize1 = 0;
        gi8BulkOutReady1 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }

    /* Process the software Tx FIFO */
    if(comTbytes0)
    {
        /* Check if Tx is working */
        if((UART0->IER & UART_IER_THRE_IE_Msk) == 0)
        {
            /* Send one bytes out */
            UART0->THR = comTbuf0[comThead0++];
            if(comThead0 >= TXBUFSIZE)
                comThead0 = 0;

            __set_PRIMASK(1);
            comTbytes0--;
            __set_PRIMASK(0);

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->IER |= UART_IER_THRE_IE_Msk;
        }
    }

    if(comTbytes1)
    {
        /* Check if Tx is working */
        if((UART1->IER & UART_IER_THRE_IE_Msk) == 0)
        {
            /* Send one bytes out */
            UART1->THR = comTbuf1[comThead1++];
            if(comThead1 >= TXBUFSIZE)
                comThead1 = 0;

            __set_PRIMASK(1);
            comTbytes1--;
            __set_PRIMASK(0);

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART1->IER |= UART_IER_THRE_IE_Msk;
        }
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    SYS_Init();
    UART0_Init();
    UART1_Init();

    printf("NuMicro USB CDC Dual Port\n");

    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART1_IRQn);
    USBD_Start();

    while(1)
    {
        VCOM_TransferData();
    }
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

