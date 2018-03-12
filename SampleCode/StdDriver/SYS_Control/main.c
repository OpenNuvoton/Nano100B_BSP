/******************************************************************************
* @file     main.c
* @version  V1.00
* $Revision: 10 $
* $Date: 15/06/16 4:34p $
* @brief    Demonstrate how to output system clock to CLKO (PC.5) pin
*           with the system clock / 4 frequency.
*
* @note
* Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Nano100Series.h"

#define SIGNATURE       0x125ab234
#define FLAG_ADDR       0x20001FFC

extern char GetChar(void);
__IO uint32_t u32PWDU_WakeFlag = 0;
__IO uint32_t u32WDT_Ticks = 0;
/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void BOD_IRQHandler(void)
{
    /* Clear Interrupt Flag */
    SYS->BODSTS |= SYS_BODSTS_BOD_INT_Msk;

    printf("Brown Out is Detected\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT_IRQ IRQ Handler                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void WDT_IRQHandler(void)
{
    __IO uint32_t u32IntSts;

    u32IntSts = WDT->ISR;

    if (u32IntSts & WDT_ISR_WAKE_IS_Msk)
        printf("     WDT Wake Up Interrupt Occurs.\n");
    else
        printf("     WDT Interrupt Occurs <%d>.\n", u32WDT_Ticks);

    if (u32IntSts & WDT_ISR_IS_Msk)
        u32WDT_Ticks++;

    WDT->ISR = u32IntSts; /* clear interrupts */
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Power Down Wake Up IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void PDWU_IRQHandler(void)
{
    printf("     PDWU_IRQHandler running...\n");

    u32PWDU_WakeFlag = 1;
    CLK->WK_INTSTS = CLK_WK_INTSTS_PD_WK_IS_Msk; /* clear interrupt */
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /* Enable external 12MHz HXT, 32KHz LXT and HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk | CLK_PWRCTL_LXT_EN_Msk | CLK_PWRCTL_HIRC_EN_Msk);

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART_S_HIRC,CLK_UART_CLK_DIVIDER(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
#if 0
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX );
#else
    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX|SYS_PA_H_MFP_PA14_MFP_UART0_RX);
#endif

    /* Set PB multi-function pins for Clock Output */
    SYS->PB_H_MFP = ( SYS->PB_H_MFP & ~SYS_PB_H_MFP_PB12_MFP_Msk ) |  SYS_PB_H_MFP_PB12_MFP_CKO;
    /* Lock protected registers */
    SYS_LockReg();
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    //UART_Open(UART0, 115200);
#if 1
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | CLK_CLKSEL1_UART_S_HXT;  /* Select 12 Mhz XTAL */

    UART0->BAUD = 0x67;              /* Baud Rate:115200  OSC:12MHz */
    UART0->TLCTL = 0x03;             /* Character len is 8 bits */
#endif
}

int32_t main (void)
{
    uint32_t u32data;

    /* HCLK will be set to 42MHz in SYS_Init(void)*/
    if(SYS->RegLockAddr == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();


    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    /*
        This sample code will show some function about system manager controller and clock controller:
        1. Read PDID
        2. Get and clear reset source
        3. Setting about BOD
        4. Output system clock from CKO pin, and the output frequency = system clock / 4
    */

    printf("+----------------------------------------+\n");
    printf("|    Nano100 System Driver Sample Code   |\n");
    printf("+----------------------------------------+\n");

    if (M32(FLAG_ADDR) == SIGNATURE)
    {
        printf("  CPU Reset success!\n");
        M32(FLAG_ADDR) = 0;
        printf("  Press any key to continue ...\n");
        GetChar();
    }

    /*---------------------------------------------------------------------------------------------------------*/
    /* Misc system function test                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Read Part Device ID */
    printf("Product ID 0x%x\n", SYS->PDID);

    /* Get reset source from last operation */
    u32data = SYS_GetResetSrc();
    printf("Reset Source 0x%x\n", u32data);

    /* Clear reset source */
    SYS_ClearResetSrc(u32data);

    /* Unlock protected registers for Brown-Out Detector and power down settings */
    SYS_UnlockReg();

    /* Check if the write-protected registers are unlocked before BOD setting and CPU Reset */
    if (SYS->RegLockAddr != 0)
    {
        printf("Protected Address is Unlocked\n");
    }

    /* Enable Brown-Out Detector and Low Voltage Reset function, and set Brown-Out Detector voltage 2.5V ,
             Enable Brown-Out Interrupt function */
    SYS_EnableBOD(SYS_BODCTL_BOD25_INT_EN_Msk,SYS_BODCTL_BOD25_EN_Msk);

    /* Enable BOD IRQ */
    NVIC_EnableIRQ(BOD_IRQn);

    /* Waiting for message send out */
    // _UART_WAIT_TX_EMPTY(UART0);

    /* Enable CKO and output frequency = system clock / 4 */
    CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_S_HCLK,1);

    /* Switch HCLK clock source to Internal 11.0592MHz */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC,CLK_HCLK_CLK_DIVIDER(1));


    /* Enable WDT clock */
    CLK_EnableModuleClock(WDT_MODULE);

    /* Enable WDT and interrupt */
    WDT->CTL = 0x00000050 | 0x00000004 | 0x00000008;
    WDT->IER |=  0x00000001;
    NVIC_EnableIRQ(WDT_IRQn);

    CLK->PWRCTL |= CLK_PWRCTL_WAKEINT_EN;  /* Enable wake up interrupt source */
    NVIC_EnableIRQ(PDWU_IRQn);   /* Enable IRQ request for PDWU interrupt */

    printf("u32PWDU_WakeFlag = %x\n",u32PWDU_WakeFlag);
    printf("Enter Power Down Mode >>>>>>>>>>>\n");
    u32PWDU_WakeFlag = 0;                   /* clear software semaphore */
    while(!(UART0->FSR & UART_FSR_TX_EMPTY_F_Msk)) ;  /* waits for message send out */
    CLK_PowerDown();

    /* CPU Reset test */
    printf("Waits for 5 times WDT interrupts.....\n");
    while (u32WDT_Ticks <= 5);

    printf("<<<<<<<<<< Program resumes execution.\n");
    printf("u32PWDU_WakeFlag = %x\n",u32PWDU_WakeFlag);

    /* Write a signature work to SRAM to check if it is reset by software */
    M32(FLAG_ADDR) = SIGNATURE;
    printf("\n\n  >>> Reset CPU <<<\n");

    /* Reset CPU */
    SYS_ResetCPU();

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
