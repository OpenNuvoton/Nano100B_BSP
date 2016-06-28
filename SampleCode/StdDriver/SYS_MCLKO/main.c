/******************************************************************************
* @file     main.c
* @version  V1.00
* $Revision: 1 $
* $Date: 15/02/05 11:06a $
* @brief    Demonstrate how to output module clock to PC.0.
*
* @note
* Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Nano100Series.h"

#define SIGNATURE       0x125ab234
#define FLAG_ADDR       0x20001FFC

extern char GetChar(void);

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

    /*  Set HCLK frequency 12MHz */
    CLK_SetCoreClock(12000000);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART_S_HIRC,CLK_UART_CLK_DIVIDER(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX|SYS_PA_H_MFP_PA14_MFP_UART0_RX);

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
	  int tdelay=1000000;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    /*
        This sample code will Output module clock from PC.0 pin.        
    */

    printf("+-----------------------------------------+\n");
    printf("| Nano100 Module Clock Output Sample Code |\n");
    printf("+-----------------------------------------+\n");
	  	
	  /* Enable PLL clock and set PLL clock to 48Mhz */
	  CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_HIRC,48000000);
	
	  CLK->MCLKO |= CLK_MCLKO_MCLK_EN_Msk ;
	  printf("This sample code will Output module clock from PC.0 pin.\n");
	  while(1)
		{
			printf("MCLK output = ISP_CLK\n");
			CLK->MCLKO = (CLK->MCLKO & ~CLK_MCLKO_MCLK_SEL_Msk) | CLK_MCLKO_MCLK_SEL_ISP_CLK;
			CLK_SysTickDelay(tdelay);
			
			printf("MCLK output = HIRC\n");
			CLK->MCLKO = (CLK->MCLKO & ~CLK_MCLKO_MCLK_SEL_Msk) | CLK_MCLKO_MCLK_SEL_HIRC;
			CLK_SysTickDelay(tdelay);

			printf("MCLK output = HXT\n");
			CLK->MCLKO = (CLK->MCLKO & ~CLK_MCLKO_MCLK_SEL_Msk) | CLK_MCLKO_MCLK_SEL_HXT;
			CLK_SysTickDelay(tdelay);

			printf("MCLK output = LXT\n");
			CLK->MCLKO = (CLK->MCLKO & ~CLK_MCLKO_MCLK_SEL_Msk) | CLK_MCLKO_MCLK_SEL_LXT;
			CLK_SysTickDelay(tdelay);

			printf("MCLK output = LIRC\n");
			CLK->MCLKO = (CLK->MCLKO & ~CLK_MCLKO_MCLK_SEL_Msk) | CLK_MCLKO_MCLK_SEL_LIRC;
			CLK_SysTickDelay(tdelay);

			printf("MCLK output = PLL ouptut\n");
			CLK->MCLKO = (CLK->MCLKO & ~CLK_MCLKO_MCLK_SEL_Msk) | CLK_MCLKO_MCLK_SEL_PLLO;
			CLK_SysTickDelay(tdelay);

			printf("MCLK output = PLL input\n");
			CLK->MCLKO = (CLK->MCLKO & ~CLK_MCLKO_MCLK_SEL_Msk) | CLK_MCLKO_MCLK_SEL_PLLI;
			CLK_SysTickDelay(tdelay);
			
			printf("MCLK output = sytem tick \n");
			CLK->MCLKO = (CLK->MCLKO & ~CLK_MCLKO_MCLK_SEL_Msk) | CLK_MCLKO_MCLK_SEL_SYSTICK;
			CLK_SysTickDelay(tdelay);

			printf("MCLK output = HCLK\n");
			CLK->MCLKO = (CLK->MCLKO & ~CLK_MCLKO_MCLK_SEL_Msk) | CLK_MCLKO_MCLK_SEL_ISP_CLK;
			CLK_SysTickDelay(tdelay);
			
			printf("MCLK output = PCLK\n");
			CLK->MCLKO = (CLK->MCLKO & ~CLK_MCLKO_MCLK_SEL_Msk) | CLK_MCLKO_MCLK_SEL_PCLK;
			CLK_SysTickDelay(tdelay);						
		}
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
