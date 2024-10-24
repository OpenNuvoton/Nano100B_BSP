/**************************************************************************//**
 * @file     ld_boot.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 14/09/12 5:03p $
 * @brief    Show how to branch programs between LDROM, APROM start page,
 *           and APROM other page.
 *
 * @note
 * Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "map.h"


#ifdef __ARMCC_VERSION
void __set_SP(uint32_t _sp)
{
    __set_MSP(_sp);
}
#endif

/**
 * @brief    Routine to get a char
 * @param    None
 * @returns  Get value from UART debug port or semihost
 * @details  Wait UART debug port or semihost to input a char.
 */
static char GetChar(void)
{
    while(1)
    {
        if ((UART0->FSR & UART_FSR_RX_EMPTY_F_Msk) == 0)
        {
            return (UART0->RBR);
        }
    }
}

extern void SendChar_ToUART(int ch);

static void PutString(char *str)
{
    while (*str != '\0')
    {
        SendChar_ToUART(*str++);
    }
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HXT_EN_Pos); // HXT Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable

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
    //SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    //SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_TX | SYS_PB_L_MFP_PB1_MFP_UART0_RX);
    SYS->PB_L_MFP = ((SYS->PB_L_MFP & ~0xFF) | 0x11);

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



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    int             u8Item;
    int             cbs;
    uint32_t        au32Config[2];
    FUNC_PTR        *func;                 /* function pointer */
#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
    uint32_t        u32Data;
#endif

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for PutString */
    UART0_Init();

    /* Enable FMC ISP function */
    SYS_UnlockReg();
    FMC_Open();

#if defined (__ARMCC_VERSION) || defined (__ICCARM__)  /* Removed Under GCC to reduce code size */
    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        PutString("\n\nFailed to read Config!\n\n");
        return -1;
    }
    cbs = (au32Config[0] >> 6) & 0x3;
    printf("Config0 = 0x%x, Config1 = 0x%x, CBS=%d\n\n", au32Config[0], au32Config[1], cbs);
#endif

    do
    {
        PutString("\n\n\n");
        PutString("+----------------------------------------------+\n");
        PutString("|       LD boot program running on LDROM       |\n");
        PutString("+----------------------------------------------+\n");
        PutString("|               Program Select                 |\n");
        PutString("+----------------------------------------------|\n");
        PutString("| [0] Run ISP program (at APROM 22K)           |\n");
        PutString("| [1] Branch and run APROM program             |\n");
        PutString("+----------------------------------------------+\n");
        PutString("Please select...");
        u8Item = GetChar();

        switch (u8Item)
        {
        case '0':
            FMC_SetVectorPageAddr(ISP_CODE_BASE);
            func =  (FUNC_PTR *)FMC_Read(ISP_CODE_ENTRY+4);

#if defined (__ARM_VERSION) || defined (__ICCARM__)  /* Removed Under GCC to reduce code size */
            PutString("Please make sure isp.bin is in APROM address 0x5800.\n");
            PutString("If not, please run \"[1] Branch and run APROM program\"\n");
            PutString("\nChange VECMAP and branch to ISP code...\n");
            while (!UART_IS_TX_EMPTY(UART0));
#endif
            /*
             *  The stack base address of an executable image is located at offset 0x0.
             *  Thus, this sample get stack base address of ISP code from ISP_CODE_ENTRY + 0x0.
             */
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) /* for GNU C compiler */
            u32Data = FMC_Read(ISP_CODE_BASE);
            asm("msr msp, %0" : : "r" (u32Data));
#else
            __set_SP(*(uint32_t *)ISP_CODE_BASE);
#endif
            func();
            break;

        case '1':
            FMC_SetVectorPageAddr(USER_AP_ENTRY);
            func = (FUNC_PTR *)FMC_Read(USER_AP_ENTRY+4);

            PutString("\n\nChange VECMAP and branch to user application...\n");
            while (!UART_IS_TX_EMPTY(UART0));

            /*
             *  The stack base address of an executable image is located at USER_AP_ENTRY offset 0x0.
             *  Thus, this sample get stack base address of AP code from USER_AP_ENTRY + 0x0.
             */
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) /* for GNU C compiler */
            u32Data = FMC_Read(USER_AP_ENTRY);
            asm("msr msp, %0" : : "r" (u32Data));
#else
            __set_SP(inpw(USER_AP_ENTRY));
#endif
            func();
            break;

        default :
            continue;
        }
    }

    while (1);

}



