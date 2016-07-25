/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/11 5:30p $
 * @brief    Demonstrate how to wake up system form Power-down mode by UART interrupt.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "Nano100Series.h"

//#define ENABLE_GPIO_WAKEUP 1


/* Global variables */
__IO int32_t   _Wakeup_Flag = 0;    /* 1 indicates system wake up from power down mode */

/*---------------------------------------------------------------------------------------------------------*/
/* PDWU Handle function                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void PDWU_IRQHandler()
{
    printf("PDWU_IRQHandler running...\n");
    CLK->WK_INTSTS = 1; /* clear interrupt status */
    _Wakeup_Flag = 1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* GPABC Wake Up Handle function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void GPABC_IRQHandler(void)
{
    /* To check if PB.5 interrupt occurred */
    if (PB->ISRC & BIT4) {
        PB->ISRC = BIT4;
        printf("PB.4 INT occurred. \n");

    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Wake Up Handle function                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    uint32_t u32IntStatus;

    u32IntStatus = UART1->ISR;

    /* Wake Up */
    if (u32IntStatus & UART_ISR_WAKE_IS_Msk) {
        printf("UART_Wakeup. \n");
        UART1->ISR = UART_ISR_WAKE_IS_Msk; //clear status
    }

}

/**
  * @brief  Save multi-function pin setting and then go to power down.
  * @param  None.
  * @return None.
  */
void Enter_PowerDown()
{
    SYS_UnlockReg();

    UART_EnableInt(UART1, UART_IER_WAKE_IE_Msk);
    UART1->CTL |= UART_CTL_WAKE_DATA_EN_Msk;
    NVIC_EnableIRQ(UART1_IRQn);

#ifdef ENABLE_GPIO_WAKEUP
    NVIC_EnableIRQ(GPABC_IRQn);
#endif

    CLK->PWRCTL |= CLK_PWRCTL_WAKEINT_EN;
    NVIC_EnableIRQ(PDWU_IRQn);

    CLK_PowerDown();
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

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    CLK->CLKSEL1 &= ~CLK_CLKSEL1_LCD_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_LCD_S_LXT);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable
    CLK->APBCLK |= CLK_APBCLK_UART1_EN; // UART0 Clock Enable

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX);

    /* Set PB multi-function pins for UART1 RXD, TXD, RTS  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB4_MFP_Msk | SYS_PB_L_MFP_PB5_MFP_Msk);
    SYS->PB_L_MFP |=  (SYS_PB_L_MFP_PB4_MFP_UART1_RX | SYS_PB_L_MFP_PB5_MFP_UART1_TX);

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

/**
  * @brief  Main routine.
  * @param  None.
  * @return None.
  */
int32_t main(void)
{
    SYS_Init();
    UART0_Init();
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Rx WAkeup Function Test                             |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo UART1 Rx(PB.4) wakeup from   |\n");
    printf("|    power down mode.                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Please input any data to uart1 Rx pin to wakeup system.   |\n");
    printf("+-----------------------------------------------------------+\n");

#ifdef ENABLE_GPIO_WAKEUP
    GPIO_EnableInt(PB, 4, GPIO_INT_BOTH_EDGE);
    PB->ISRC = BIT4;
#endif

    printf("Going to Power Down...\n\n");

    while(!(UART0->FSR & UART_FSR_TE_F_Msk)) ;  /* waits for message send out */

    Enter_PowerDown();

    if (_Wakeup_Flag == 1) {
        _Wakeup_Flag = 0;

        printf("\n Wakeup OK!!");

        CLK_SysTickDelay(335000);
    }

    printf("\n Wakeup demo end.");

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
    MFP_UART0_TO_PORTA();                  /* UART0 TX/RX to PA14/PA15*/
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


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



