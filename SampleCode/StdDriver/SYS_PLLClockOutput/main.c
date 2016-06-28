/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 15/06/16 4:36p $
 * @brief    Demonstrate how to Change system clock to different PLL frequency and 
 *           output system clock from CLKO pin.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

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
/*  Simple calculation test function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PI_NUM  256
int32_t f[PI_NUM + 1];
uint32_t piTbl[19] =
{
    3141,
    5926,
    5358,
    9793,
    2384,
    6264,
    3383,
    2795,
    288,
    4197,
    1693,
    9937,
    5105,
    8209,
    7494,
    4592,
    3078,
    1640,
    6284
};

int32_t piResult[19];

int32_t pi(void)
{
    int32_t i, i32Err;
    int32_t a = 10000, b = 0, c = PI_NUM, d = 0, e = 0, g = 0;

    for(; b - c;)
        f[b++] = a / 5;

    i = 0;
    for(; d = 0, g = c * 2; c -= 14,/*printf("%.4d\n",e+d/a),*/ piResult[i++] = e + d / a, e = d % a)
    {
        if(i == 19)
            break;

        for(b = c; d += f[b] * a, f[b] = d % --g, d /= g--, --b; d *= b);
    }
    i32Err = 0;
    for(i = 0; i < 19; i++)
    {
        if(piTbl[i] != piResult[i])
            i32Err = -1;
    }

    return i32Err;
}

void Delay(uint32_t x)
{
    int32_t i;

    for(i = 0; i < x; i++)
    {
        __NOP();
        __NOP();
    }
}

uint32_t g_au32PllSetting[] =
{
    24000000,  /* PLL = 24MHz */
	  31500000,  /* PLL = 31.5MHz */
    36000000,  /* PLL = 36MHz */
		42000000,  /* PLL = 42MHz */    
};


void SYS_PLL_Test(void)
{
    int32_t  i;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PLL clock configuration test                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n-------------------------[ Test PLL ]-----------------------------\n");

    for(i = 0; i < sizeof(g_au32PllSetting) / sizeof(g_au32PllSetting[0]) ; i++)
    {
        /* Select HCLK clock source from PLL.
           PLL will be configured to twice specified frequency.
           And HCLK clock source divider will be set to 2.
        */
        CLK_SetCoreClock(g_au32PllSetting[i]);

        printf("  Change system clock to %d Hz ...................... ", SystemCoreClock);

        /* Output selected clock to CKO, CKO Clock = HCLK / 2^(1 + 1) */
        CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_S_HCLK, 1);

        /* The delay loop is used to check if the CPU speed is increasing */
        Delay(0x400000);

        if(pi())
        {
            printf("[FAIL]\n");
        }
        else
        {
            printf("[OK]\n");
        }

        /* Disable CLKO clock */
        CLK_DisableCKO();
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
    UART_Open(UART0, 115200);
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

    if (M32(FLAG_ADDR) == SIGNATURE) {
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
    if (SYS->RegLockAddr != 0) {
        printf("Protected Address is Unlocked\n");
    }

    /* Enable Brown-Out Detector and Low Voltage Reset function, and set Brown-Out Detector voltage 2.5V ,
             Enable Brown-Out Interrupt function */
    SYS_EnableBOD(SYS_BODCTL_BOD25_INT_EN_Msk,SYS_BODCTL_BOD25_EN_Msk);

    /* Enable BOD IRQ */
    NVIC_EnableIRQ(BOD_IRQn);
		
    /* Run PLL Test */
    SYS_PLL_Test();		

    /* Waiting for message send out */
    UART_WAIT_TX_EMPTY(UART0);

    /* Enable CKO and output frequency = system clock / 4 */
    CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_S_HCLK,1);

    /* Switch HCLK clock source to Internal 11.0592MHz */			
		CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLK_N_Msk) | CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | CLK_HCLK_CLK_DIVIDER(1);    

    /* Set PLL to Power down mode and HW will also clear PLLSTB bit in CLKSTATUS register */
    CLK_DisablePLL();

    /* Reset CPU */
    SYS_ResetCPU();
				
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
