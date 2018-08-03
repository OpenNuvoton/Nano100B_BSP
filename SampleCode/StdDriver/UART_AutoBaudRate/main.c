/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/06/26 1:52p $
 * @brief    Demonstrate how to use auto baud rate detection function.
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
    uint32_t clk =0, div;

    div = ( (CLK->CLKDIV0 & CLK_CLKDIV0_UART_N_Msk) >> 8) + 1;

    switch (CLK->CLKSEL1 & CLK_CLKSEL1_UART_S_Msk)
    {
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
/* UART Auto Baud Rate HANDLE                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_ABAUD_INT_HANDLE(void)
{
    uint32_t result;
    uint32_t clk;

    g_bWait = FALSE;

    if (UART1->TRSR & UART_TRSR_ABAUD_F_Msk)
    {
        UART1->TRSR = UART_TRSR_ABAUD_F_Msk;

        clk = GetUartClk();
        result = clk / (UART1->BAUD + 1);
        printf("> BAUD Detect Complete, divider = %d [%xh]\n", UART1->BAUD, UART1->BAUD);
        printf("> BAUD Rate = %d\n", result);
    }

    if (UART1->TRSR & UART_TRSR_ABAUD_TOUT_F_Msk)
    {
        UART1->TRSR = UART_TRSR_ABAUD_TOUT_F_Msk;
        printf("\nFailed -- Auto Baud Rate Time Out...");
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

    for(i=Offset,CheckSum=0; i<=9; i++)
    {
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
    for(mask=0; mask<6; mask++)
        ID[mask] = (i & (1<<mask))>>mask;

    p_Bit[0] = (ID[0] + ID[1] + ID[2] + ID[4])%2;
    p_Bit[1] = (!((ID[1] + ID[3] + ID[4] + ID[5])%2));

    number = i + (p_Bit[0] <<6) + (p_Bit[1]<<7);
    return number;

}

int32_t DataCompare(uint8_t InBuffer[],uint8_t OutBuffer[],int32_t len)
{
    int i=0;
    for(i=0; i<len; i++)
    {
        if(InBuffer[i]!=OutBuffer[i])
        {
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
    if(UART1->ISR & UART_ISR_ABAUD_IS_Msk)
    {
        UART_ABAUD_INT_HANDLE();
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Auto Baud Rate Test                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_AutoBaudRateTest()
{
    printf("\nUART Auto Baud Rate Test. ");
    printf("\n(Press '1' to start test UART1 Auto Baud Rate.)\n");

    /* Set UART Configuration */
    UART_Open(UART1, 115200);

    /* Set UART1 Auto Baud Rate Enable*/
    UART1->CTL |= UART_CTL_ABAUD_EN_Msk;

    /* Enable Interrupt and install the call back function */
    UART_EnableInt(UART1, UART_IER_ABAUD_IE_Msk);
    NVIC_EnableIRQ(UART1_IRQn);

    while(g_bWait);

    /* Disable Interrupt */
    UART_DisableInt(UART1, UART_IER_ABAUD_IE_Msk);

    g_bWait =TRUE;
    //sParam.u8AutoBaudRate = DISABLE;

    printf("\nUART Auto Baud Rate Test End.\n");

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
    /* Init UART0 for printf */
    UART1_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|               UART Sample Program                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| UART Auto Baud Rate test                                  |\n");
    printf("+-----------------------------------------------------------+\n");

    UART_AutoBaudRateTest();    /* Auto Baud Rate test */

    while(1);
}


#ifdef USE_ASSERT
/**
  * @brief  The function prints the source file name and line number where the assert_param() error
  *         occurs, and then stops in an infinite loop. User can add his own codes here if necessary.
  * @param[in] file Source file name
  * @param[in] line Line number
  * @return None
  */
void assert_error(uint8_t * file, uint32_t line)
{
    GCR->PB_L_MFP = (GCR->PB_L_MFP & ~0x77) | (PB1_MFP_UART0_TX | PB0_MFP_UART0_RX);  /* Select multi-function pin for UART0 */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN;    /* Enable UART0 clock */
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_UART_MASK;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_MASK) | CLK_CLKSEL1_UART_HXT;  /* Select 12 Mhz XTAL */


    /* Set UART to 115200,n,8,1,none */
    UART0->BAUD = 0x67;             /* Baud Rate:115200 for 12MHz */
    UART0->TLCTL = 0x03;            /* Word len is 8 bits */

    printf("[%s] line %d : wrong parameters.\r\n", file, line);

    /* Infinite loop */
    while(1) ;

}
#endif


/*** (C) COPYRIGHT 2012 Nuvoton Technology Corp. ***/



