/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/06/26 2:03p $
 * @brief    Demonstrate how to transmit data in UART RS485 mode.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Nano100Series.h"
#include "uart.h"
#include "sys.h"
#include "clk.h"



#define RXBUFSIZE 3072

/*---LIN Check sum mode-------------------*/
#define MODE_CLASSIC    2
#define MODE_ENHANCED   1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---Using in UART Test -------------------*/
volatile uint8_t comRbuf[1024];
volatile uint16_t comRbytes = 0;        /* Available receiving bytes */
volatile uint16_t comRhead  = 0;
volatile uint16_t comRtail  = 0;
volatile int32_t g_bWait    = TRUE;
uint8_t u8SendData[12] = {0};

/*---Using in RS485 Test -------------------*/
uint8_t u8RecData[RXBUFSIZE]  = {0};
int32_t w_pointer =0;
volatile int32_t r_pointer = 0;
int32_t IsRS485ISR_TX_PORT = FALSE;
int32_t IsRS485ISR_RX_PORT = FALSE;

/*---Using in LIN Test -------------------*/
uint8_t testPattern[] = {0x00,0x55,0xAA,0xFF,0x00,0x55,0xFF,0xAA};

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_INT_HANDLE(uint32_t u32IntStatus);
int32_t DataCompare(uint8_t InBuffer[],uint8_t OutBuffer[],int32_t len);
extern char GetChar(void);

uint32_t GetUartClk(void)
{
    uint32_t clk =0 , div;

    div = ( (CLK->CLKDIV0 & CLK_CLKDIV0_UART_N_Msk) >> 8) + 1;

    switch (CLK->CLKSEL1 & CLK_CLKSEL1_UART_S_Msk) {
    case 0:
        clk = __HXT; /* HXT */
        break;
    case 1:
        clk = __LXT;  /* LXT */
        break;
    case 2:
        clk = SysGet_PLLClockFreq(); /* PLL */
        break;
    case 3:
        clk = __HIRC12M; /* HIRC */
        break;
    }

    clk /= div;

    return clk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* RS485 HANDLE                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_INT_HANDLE(void)
{
    volatile char addr;
    volatile char regRX;

    if((UART1->ISR & UART_ISR_RLS_IS_Msk) && (UART1->ISR & UART_ISR_RDA_IS_Msk)) {  /* RLS INT & RDA INT */
        if((UART1->TRSR & UART_TRSR_RS485_ADDET_F_Msk) && (UART1->FUN_SEL & 0x3)) { /* ADD_IF, RS485 mode */
            addr = UART1->RBR;
            UART1->TRSR |= UART_TRSR_RS485_ADDET_F_Msk;             /* clear ADD_IF flag */
        }
    } else if((UART1->ISR & UART_ISR_RDA_IS_Msk)) { /* Rx Ready */
        /* Time-out INT */
        regRX = UART1->RBR;

        if(IsRS485ISR_TX_PORT)
            UART1->THR  = regRX;
        else
            u8RecData[r_pointer++] = regRX;
    } else if((UART1->ISR & UART_ISR_RTO_IS_Msk)) { /* Rx Ready */
        /* Time-out INT */
        regRX = UART1->RBR;

        if(IsRS485ISR_TX_PORT)
            UART1->THR  = regRX;
        else
            u8RecData[r_pointer++] = regRX;
    } else if(UART1->ISR & UART_ISR_BUF_ERR_IS_Msk) {       /* Buff Error INT */
        printf("\nBuffer Error...\n");
        while(1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Sub-Function for LIN                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

/* Compute the checksum byte */
/* Offset :               */
/*    [1] : Compute not include ID  (LIN1.1) */
/*    [2] : Compute n include ID  (LIN2.0)   */

uint32_t cCheckSum(uint8_t DataBuffer[], uint32_t Offset)
{
    uint32_t i,CheckSum =0;

    for(i=Offset,CheckSum=0; i<=9; i++) {
        CheckSum+=DataBuffer[i];
        if (CheckSum>=256)
            CheckSum-=255;
    }
    return (255-CheckSum);
}

/* Compute the Parity Bit */
int8_t Parity(int i)
{
    int8_t number = 0 ;
    int8_t ID[6];
    int8_t p_Bit[2];
    int8_t mask =0;

    if(i>=64)
        printf("The ID is not match protocol\n");
    for(mask=0; mask<7; mask++)
        ID[mask] = (i & (1<<mask))>>mask;

    p_Bit[0] = (ID[0] + ID[1] + ID[2] + ID[4])%2;
    p_Bit[1] = (!((ID[1] + ID[3] + ID[4] + ID[5])%2));

    number = i + (p_Bit[0] <<6) + (p_Bit[1]<<7);
    return number;

}

int32_t DataCompare(uint8_t InBuffer[],uint8_t OutBuffer[],int32_t len)
{
    int i=0;
    for(i=0; i<len; i++) {
        if(InBuffer[i]!=OutBuffer[i]) {
            printf("In[%d] = %x , Out[%d] = %d\n",i,InBuffer[i],i,OutBuffer[i]);
            return FALSE;
        }
    }
    return TRUE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupt Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    if((UART1->FUN_SEL & 0x3) == 0x3) { // RS485 function
        RS485_INT_HANDLE();
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Sub-Function in RS485 Function Test                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485Send(uint8_t *BufferPtr, uint32_t Length)
{
    uint32_t i;

    for ( i = 0; i < Length; i++ ) {
        while ( !(UART1->FSR & UART_FSR_TX_EMPTY_F_Msk) );
        while ( !(UART1->FSR & UART_FSR_TE_F_Msk) );

        if (i==0)    /* Address Byte*/
            UART1->TLCTL = ((UART1->TLCTL & ~UART_TLCTL_EPE_Msk) | UART_TLCTL_SPE_Msk | UART_TLCTL_PBE_Msk);
        else         /* Data Byte*/
            UART1->TLCTL |= (UART_TLCTL_EPE_Msk | UART_TLCTL_SPE_Msk | UART_TLCTL_PBE_Msk);
        UART1->THR = *BufferPtr;

        BufferPtr++;
    }
    return;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_TransmitTest()
{
    int32_t i;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|               RS485 Function Test                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Switch RS485 Mode.                                    |\n");
    printf("| 2). Send %4d bytes though RS485 interface                |\n",RXBUFSIZE);
    printf("| 3). Test with Item [4].RS485 receive test                 |\n");
    printf("| 4). Press any key to start.                               |\n");
    printf("+-----------------------------------------------------------+\n\n");
    GetChar();

    /* variables set default value */
    IsRS485ISR_RX_PORT = FALSE;
    IsRS485ISR_TX_PORT = TRUE;
    r_pointer = 0;

    /* Set UART Configuration */
    UART_Open(UART1, 115200);
    /* Set Data Format*/ /* Only need parity enable whenever parity ODD/EVEN */
    UART_SetLine_Config(UART1, 0, UART_WORD_LEN_8, UART_PARITY_EVEN, UART_STOP_BIT_1);

    UART1->MCSR &= ~UART_MCSR_LEV_RTS_Msk;
    /* Set RS485 Configuration */
    UART1->CTL |= UART_CTL_RX_DIS_Msk;
    UART_SelectRS485Mode(UART1, UART_ALT_CTL_RS485_AAD_Msk|UART_ALT_CTL_RS485_AUD_Msk, 0xC0);

    u8RecData[0] = 0xC0;                            /* RS485_SLAVE_ADR; */

    for(i=1; i<RXBUFSIZE; i++) {
        u8RecData[i] = i & 0xFF;
    }

    RS485Send((uint8_t *)u8RecData,RXBUFSIZE);
    printf("Transfer Done\n");

}

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
    UART_Open(UART1, 57600);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Main Function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART0_Init();
    /* Init UART1 */
    UART1_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|               UART Sample Program                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| RS485 Transmit function test                               |\n");
    printf("+-----------------------------------------------------------+\n");

    RS485_TransmitTest();   /* RS485 Transmit Test (Need two module board to test) */

    while(1);
}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



