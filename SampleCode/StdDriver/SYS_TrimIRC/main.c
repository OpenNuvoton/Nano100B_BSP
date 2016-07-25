/******************************************************************************
* @file     main.c
* @version  V1.00
* $Revision: 4 $
* $Date: 15/06/17 5:14p $
* @brief    Demonstrate how to use LXT to trim HIRC
*
* @note
* Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Nano100Series.h"
/*---------------------------------------------------------------------------------------------------------*/
/*  IRCTrim IRQ Handler                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void HIRC_IRQHandler()
{
    uint32_t status;
    status=SYS_GET_IRCTRIM_INT_FLAG();
    if(status & BIT1) {
        printf("Trim Failure Interrupt\n");
        SYS_CLEAR_IRCTRIM_INT_FLAG(SYS_IRCTRIMINT_FAIL_INT);
    }
    if(status & BIT2) {
        SYS_CLEAR_IRCTRIM_INT_FLAG(SYS_IRCTRIMINT_32KERR_INT);
        printf("LXT Clock Error Lock\n");
    }
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

    /*  Set HCLK frequency 32MHz */
    CLK_SetCoreClock(32000000);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(FDIV_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART_S_HXT,CLK_UART_CLK_DIVIDER(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX );

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
    UART_Open(UART0, 115200);
}

int32_t main (void)
{
    uint32_t status;
    /* HCLK will be set to 32MHz in SYS_Init(void)*/
    if(SYS->RegLockAddr == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);


    printf("+----------------------------------------+\n");
    printf("|      Nano100 Trim IRC Sample Code      |\n");
    printf("+----------------------------------------+\n");

    /* Enable Interrupt */
    NVIC_EnableIRQ(HIRC_IRQn);

    /*  Enable IRC Trim, set HIRC clock to 12Mhz and enable interrupt */
    SYS_EnableIRCTrim(SYS_IRCTRIMCTL_TRIM_12M,SYS_IRCTRIMIEN_32KERR_EN|SYS_IRCTRIMIEN_FAIL_EN);

    /* Waiting for HIRC Frequency Lock */
    CLK_SysTickDelay(2000);

    status=SYS_GET_IRCTRIM_INT_FLAG();
    if(status & BIT0)
        printf("HIRC Frequency Lock\n");

    /* Enable CKO and output frequency = HIRC / 2 */
    CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_S_HIRC,0);
    printf("Press any key to disable IRC Trim Funciton\n");
    getchar();

    /* Disable IRC Trim */
    SYS_DisableIRCTrim();
    printf("Disable IRC Trim\n");
    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
