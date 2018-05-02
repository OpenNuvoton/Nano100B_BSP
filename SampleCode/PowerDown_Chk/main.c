/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/09/04 10:32a $
 * @brief    PWRDWN_CHECK project for Nano100 series MCU
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "Nano100Series.h"
#include "sys.h"

#ifdef __DEBUG_MSG
#define DEBUG_MSG   printf
#else
#define DEBUG_MSG(...)
#endif


/* External functions */


/* Global variables */
__IO int32_t   _Wakeup_Flag = 0;    /* 1 indicates system wake up from power down mode */
__IO uint32_t  _Pin_Setting[11];    /* store Px_H_MFP and Px_L_MFP */
__IO uint32_t  _PullUp_Setting[6];  /* store GPIOx_PUEN */



/**
  * @brief  Store original setting of multi-function pin selection.
  * @param  None.
  * @return None.
  */
void SavePinSetting()
{
    /* Save Pin selection setting */
    _Pin_Setting[0] = SYS->PA_L_MFP;
    _Pin_Setting[1] = SYS->PA_H_MFP;
    _Pin_Setting[2] = SYS->PB_L_MFP;
    _Pin_Setting[3] = SYS->PB_H_MFP;
    _Pin_Setting[4] = SYS->PC_L_MFP;
    _Pin_Setting[5] = SYS->PC_H_MFP;
    _Pin_Setting[6] = SYS->PD_L_MFP;
    _Pin_Setting[7] = SYS->PD_H_MFP;
    _Pin_Setting[8] = SYS->PE_L_MFP;
    _Pin_Setting[9] = SYS->PE_H_MFP;
    _Pin_Setting[10] = SYS->PF_L_MFP;

    /* Save Pull-up setting */
    _PullUp_Setting[0] =  PA->PUEN;
    _PullUp_Setting[1] =  PB->PUEN;
    _PullUp_Setting[2] =  PC->PUEN;
    _PullUp_Setting[3] =  PD->PUEN;
    _PullUp_Setting[4] =  PE->PUEN;
    _PullUp_Setting[5] =  PF->PUEN;
}

/**
  * @brief  Save multi-function pin setting and then go to power down.
  * @param  None.
  * @return None.
  */
void Enter_PowerDown()
{
    /* Back up original setting */
    SavePinSetting();

    /* Set function pin to GPIO mode */
    SYS->PA_L_MFP = 0;
    SYS->PA_H_MFP = 0;
    SYS->PB_L_MFP = (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX);
    SYS->PB_H_MFP = 0;
    SYS->PC_L_MFP = 0;
    SYS->PC_H_MFP = 0;
    SYS->PD_L_MFP = 0;
    SYS->PD_H_MFP = 0;
    SYS->PE_L_MFP = 0;
    SYS->PE_H_MFP = 0;
    SYS->PF_L_MFP = 0x00007777; // exclude GPF0:ICE_DAT, GPF1:ICE_CLK, GPF2:HXT_OUT, GPF3:HXT_IN

    /* Enable GPIO pull up */
    PA->PUEN = 0xFFFF;
    PB->PUEN = 0xFFFF;
    PC->PUEN = 0xFFFF;
    PD->PUEN = 0xFFFF;
    PE->PUEN = 0xFFFF;
    PF->PUEN = 0x0030;      /* exclude GPF0:ICE_DAT, GPF1:ICE_CLK, GPF2:HXT_OUT, GPF3:HXT_IN */

    //CLK->PWRCTL |= CLK_PWRCTL_PD_WK_IE_Msk;  /* Enable wake up interrupt source */
    //NVIC_EnableIRQ(PDWU_IRQn);             /* Enable IRQ request for PDWU interrupt */

    SYS_UnlockReg();
    SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
    CLK->PWRCTL |= (CLK_PWRCTL_PD_EN_Msk | CLK_PWRCTL_WK_DLY_Msk );

    //__WFI();   /* system really enter power down here ! */
}

void PowerDown_Check()
{
    uint32_t i,j;
    volatile uint32_t Tmp_NVIC_ISER;
    uint32_t gpio_error_count = 0;
    char *symbo_gpio[6] = {"A", "B", "C", "D", "E", "F"};
    GPIO_T *tGPIO;
    uint32_t SYS_MFP = 0xffffffff;

    printf("\n/***** PowerDown Check *****/");

    printf("\n/***** GPIO Check: *****/");

    //save NVIC setting
    Tmp_NVIC_ISER = NVIC->ISER[0];

    //disables a device-specific interrupt in the NVIC interrupt controller
    NVIC->ICER[0] = 0xFFFFFFFF;

    for(j = 0; j < 11; j++)
    {
        SYS_MFP = *((volatile unsigned int *)(SYS_BASE + 0x30 + (j *4)));

        for(i = 0; i < 8; i++)
        {
            if(SYS_MFP & (0x7 << (i*4)))
            {
                gpio_error_count++;
                printf("\nGP%s Pin %d not set GPIO !!", symbo_gpio[(j/2)], (i+(8*(j%2))) );
            }
        }
    }

    if( (SYS->PF_L_MFP & SYS_PF_L_MFP_PF2_MFP_HXT_OUT) != SYS_PF_L_MFP_PF2_MFP_HXT_OUT)
    {
        printf("\nGPF Pin 2 not set HXT_OUT !!!");
        printf("\nPlease Check GPF Pin 2 not connect crystal !!!");
    }

    if( (SYS->PF_L_MFP & SYS_PF_L_MFP_PF3_MFP_HXT_IN) != SYS_PF_L_MFP_PF3_MFP_HXT_IN)
    {
        printf("\nGPF Pin 3 not set HXT_IN !!!");
        printf("\nPlease Check GPF Pin 3 not connect crystal !!!");
    }

    for(j = 0; j < 6; j++)
    {
        tGPIO =(GPIO_T*)((uint32_t)PA + (j * (0x40)));

        for(i = 0; i < 16; i++)
        {
            if((j == 5) & (i > 5)) // GPF
                break;

            if(!(tGPIO->PIN & (1 << i)))
            {
                gpio_error_count++;
                printf("\nGP%s Pin %d can not pull high!!", symbo_gpio[j], i);
            }
        }
    }

    if(gpio_error_count)
        printf("\nGPIO Check fail !!");
    else
        printf("\nGPIO Check pass");

    printf("\n");

    printf("\n/***** Clock Check: *****/");

    if(CLK->PWRCTL & CLK_PWRCTL_HXT_EN)
        printf("\nHXT Enable");
    else
        printf("\nHXT Disable");

    if(CLK->PWRCTL & CLK_PWRCTL_LXT_EN)
        printf("\nLXT Enable");
    else
        printf("\nLXT Disable");

    if(CLK->PWRCTL & CLK_PWRCTL_HIRC_EN)
        printf("\nHIRC Enable");
    else
        printf("\nHIRC Disable");

    if(CLK->PWRCTL & CLK_PWRCTL_LIRC_EN)
        printf("\nLIRC Enable");
    else
        printf("\nLIRC Disable");

    printf("\n");
    printf("\n/***** Power Down setting check: *****/");
    if(CLK->PWRCTL & CLK_PWRCTL_PWRDOWN_EN)
        printf("\nChip Power-down mode Enable ");
    else
        printf("\nChip Power-down mode Disable !!!");

    if(SCB->SCR & 0x04)
        printf("\nSCB->SCR set (0x%x) : ok", SCB->SCR);
    else
        printf("\nSCB->SCR set (0x%x) : error", SCB->SCR);

    //Restore NVIC->ISER setting
    NVIC->ISER[0] = Tmp_NVIC_ISER;

    printf("\n");
    printf("\n/***** Wake-up setting check: *****/");
    if(CLK->PWRCTL & CLK_PWRCTL_DELY_EN)
        printf("\nWake-up Delay Counter Enable");
    else
        printf("\nWake-up Delay Counter Disable");

    if(CLK->PWRCTL & CLK_PWRCTL_WAKEINT_EN)
        printf("\nPower-down Mode Wake-up Interrupt Enable");
    else
        printf("\nPower-down Mode Wake-up Interrupt Disable");

    if( NVIC->ISER[0] & (1 << ((uint32_t)(PDWU_IRQn) & 0x1F)) )
        printf("\nEnable PDWU_IRQn request for PDWU interrupt");
    else
        printf("\nDisable PDWU_IRQn request for PDWU interrupt");

    printf("\n");
    printf("\n/***** ISR check: *****/");
    if(NVIC_GetPendingIRQ(PDWU_IRQn))
        printf("\nPowerDown WakeUp Interrupt status is pending. (check fail)");
    else
        printf("\nPowerDown WakeUp Interrupt status is not pending. (check ok)");

    if(CLK->WK_INTSTS & 0x1)
        printf("\nWake-up Interrupt Status is set. (check fail)");
    else
        printf("\nWake-up Interrupt Status is not set. (check ok)");

    printf("\n");
    printf("\n/***** PowerDown Check End *****/");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x3 << CLK_CLKSEL1_UART_S_Pos);// Clock source from HIRC

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX);

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

/**
  * @brief  Main routine.
  * @param  None.
  * @return None.
  */
int32_t main(void)
{
    SYS_Init();
    UART0_Init();

    /* Prepare to enter power down mode, to demo the usage of PowerDown_Check(),
       this function does not actually put this system to  power down mode  */
    Enter_PowerDown();
    /* Check system state before enter power down mode */
    PowerDown_Check();

    while(1);

}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



