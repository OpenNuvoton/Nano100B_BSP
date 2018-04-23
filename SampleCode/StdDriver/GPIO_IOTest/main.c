/**************************************************************************//**
 * @file     main.c
 * @version  V2.10
 * $Date: 15/07/20 9:38a $
 * @brief    Use GPIO driver to control the GPIO pin direction and the high/low state,
 *           and show how to use GPIO interrupts.
 *
 * @note
 * Copyright (C) 2012 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"

/**
 * @brief       PortA/PortB/PortC IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortA/PortB/PortC default IRQ, declared in startup_nano100series.s.
 */
void GPABC_IRQHandler(void)
{
	uint32_t reg;
    /* To check if PB.5 interrupt occurred */
    if (PB->ISRC & BIT5)
    {
        PB->ISRC = BIT5;
        PD0 = PD0 ^ 1;
        printf("PB.5 INT occurred. \n");

    }
    else
    {
        /* Un-expected interrupt. Just clear all PORTA, PORTB, PORTC interrupts */
        reg = PA->ISRC;
        PA->ISRC = reg;
        reg = PB->ISRC;
        PB->ISRC = reg;
        reg = PC->ISRC;
        PC->ISRC = reg;
        printf("Un-expected interrupts. \n");
    }
}


/**
 * @brief       PortD/PortE/PortF IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortD/PortE/PortF default IRQ, declared in startup_nano100series.s.
 */
void GPDEF_IRQHandler(void)
{
	uint32_t reg;
    /* To check if PE.2 interrupt occurred */
    if (PE->ISRC & BIT2)
    {
        PE->ISRC = BIT2;
        PD0 = PD0 ^ 1;
        printf("PE.2 INT occurred. \n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORTD, PORTE and PORTF interrupts */
        reg = PD->ISRC;
        PD->ISRC = reg;
        reg = PE->ISRC;
        PE->ISRC = reg;
        reg = PF->ISRC;
        PF->ISRC = reg;
        printf("Un-expected interrupts. \n");
    }
}


/**
 * @brief       External INT0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT0(PA.2) default IRQ, declared in startup_nano100series.s.
 */
void EINT0_IRQHandler(void)
{
    /* For PB.14, clear the INT flag */
    PB->ISRC = BIT14;
    PD0 = PD0 ^ 1;
    printf("PB.14 EINT0 occurred. \n");
}


/**
 * @brief       External INT1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1(PF.2) default IRQ, declared in startup_Nano100series.s.
 */
void EINT1_IRQHandler(void)
{
    /* For PB.15, clear the INT flag */
    PB->ISRC = BIT15;
    PD0 = PD0 ^ 1;
    printf("PB.15 EINT1 occurred. \n");
}

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
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
//    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
//    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX|SYS_PA_H_MFP_PA14_MFP_UART0_RX);
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB1_MFP_UART0_TX|SYS_PB_L_MFP_PB0_MFP_UART0_RX);

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

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    int32_t i32Err;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------------------+ \n");
    printf("|    NANO100 GPIO Driver Sample Code   | \n");
    printf("+--------------------------------------+ \n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect PA.0 and PA.1 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Configure PA.0 as Output mode and PA.1 as Input mode then close it */
    GPIO_SetMode(PA, BIT0, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT1, GPIO_PMD_INPUT);

    i32Err = 0;
    printf("  GPIO Output/Input test ...... \n");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    PA0 = 0;
    CLK_SysTickDelay(10);   /* wait for IO stable */
    if (PA1 != 0)
    {
        i32Err = 1;
    }

    PA0 = 1;
    CLK_SysTickDelay(10);   /* wait for IO stable */
    if (PA1 != 1)
    {
        i32Err = 1;
    }

    if ( i32Err )
    {
        printf("  [FAIL] --- Please make sure PA.0 and PA.1 are connected. \n");
    }
    else
    {
        printf("  [OK] \n");
    }

    /* Configure PA.0 and PA.1 to default Input mode */
    GPIO_SetMode(PA, BIT0, GPIO_PMD_INPUT);
    GPIO_SetMode(PA, BIT1, GPIO_PMD_INPUT);


    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("\n  GPB5, GPE2, GPB14(EINT0) and GPB15(EINT1) are used to test interrupt\n  and control LED (GPA7)\n");

    /*Configure PA7 for LED control */
    GPIO_SetMode(PA, BIT7, GPIO_PMD_OUTPUT);

    /* Configure PB5 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT5, GPIO_PMD_INPUT);
    GPIO_ENABLE_PULL_UP(PB, BIT5);
    GPIO_EnableInt(PB, 5, GPIO_INT_RISING);

    NVIC_EnableIRQ(GPABC_IRQn);

    /*  Configure PE2 as Input mode pull-up and enable interrupt by low level trigger */
    GPIO_SetMode(PE, BIT2, GPIO_PMD_INPUT);
    GPIO_ENABLE_PULL_UP(PE, BIT2);
    GPIO_EnableInt(PE, 2, GPIO_INT_LOW);

    NVIC_EnableIRQ(GPDEF_IRQn);

    /* Configure PB14 as EINT0 pin and enable interrupt by falling edge trigger */
    SYS->PB_H_MFP = (SYS->PB_H_MFP & (~0x07000000)) | 0x01000000;
    GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);
    GPIO_EnableEINT0(PB, 14, GPIO_INT_FALLING);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Configure PB15 as EINT1 pin and enable interrupt by rising and falling edge trigger */
    SYS->PB_H_MFP = (SYS->PB_H_MFP & (~0x70000000)) | 0x10000000;
    GPIO_SetMode(PB, BIT15, GPIO_PMD_INPUT);
    GPIO_EnableEINT1(PB, 15, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_HCLK, GPIO_DBCLKSEL_1);
    GPIO_ENABLE_DEBOUNCE(PB, BIT5);
    GPIO_ENABLE_DEBOUNCE(PE, BIT2);
    GPIO_ENABLE_DEBOUNCE(PB, BIT14);
    GPIO_ENABLE_DEBOUNCE(PB, BIT15);

    /* Waiting for interrupts */
    while (1);

}


