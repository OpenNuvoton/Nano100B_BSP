/**************************************************************************//**
 * @file     ld_boot.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/06/16 4:55p $
 * @brief    Demonstrate a simple IAP function to show three independent
 *           programs including main routine, independent interrupt handler
 *           and updating or switching to another program with IAP function.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include "Nano100Series.h"
#include "map.h"

int IsDebugFifoEmpty(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple printf() function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void printf_UART(const char *str,...);
void SendChar_ToUART(int ch)
{
    while(UART1->FSR & UART_FSR_TX_FULL_F_Msk);
    UART1->THR = ch;
    if(ch == '\n')
    {
        while(UART1->FSR & UART_FSR_TX_FULL_F_Msk);
        UART1->THR = '\r';
    }
}

void printInteger(uint32_t u32Temp)
{
    uint8_t print_buf[16];
    uint32_t i=15,j;

    *(print_buf+i) = '\0';
    j = u32Temp >> 31;
    if(j)
        u32Temp = ~u32Temp+1;
    do
    {
        i--;
        *(print_buf+i) = '0'+u32Temp%10;
        u32Temp = u32Temp /10;
    }
    while (u32Temp != 0);
    if(j)
    {
        i--;
        *(print_buf+i) = '-';
    }
		printf_UART((char *)(print_buf+i));
}

void printHex(uint32_t u32Temp)
{
    uint8_t print_buf[16];
    uint32_t i=15;
    uint32_t temp;

    *(print_buf+i) = '\0';
    do
    {
        i--;
        temp = u32Temp%16;
        if(temp < 10)
            *(print_buf+i) = '0'+temp;
        else
            *(print_buf+i) = 'a'+(temp-10) ;
        u32Temp = u32Temp/16;
    }
    while (u32Temp != 0);
		printf_UART((char *)(print_buf+i));
}

#define vaStart(list, param) list = (uint8_t *)((int)&param + sizeof(param))
#define vaArg(list, type) ((type *)(list += sizeof(type)))[-1]

void printf_UART(const char *str,...)
{
    //va_list args;
    uint8_t *args;

    vaStart( args, str );

    while (*str != '\0')
    {
        if(*str == '%')
        {
            str++;
            if (*str == '\0') return;
            if( *str == 'd' )
            {
                str++;
                printInteger(vaArg( args, int ));
            }
            else if( *str == 'x' )
            {
                str++;
                printHex(vaArg( args, int ));
            }
        }
        SendChar_ToUART(*str++);
    }
}


#ifdef __ARMCC_VERSION
void __set_SP(uint32_t _sp)
{
    __set_MSP(_sp);
}
#endif

static __INLINE void BranchTo(uint32_t u32Address)
{
    FUNC_PTR        *func;
    FMC_SetVectorPageAddr(u32Address);
    func =  (FUNC_PTR *)(*(uint32_t *)(u32Address+4));
    printf_UART("branch to address 0x%x\n", (int)func);
    printf_UART("\n\nChange VECMAP and branch to user application...\n");
    while (!(UART1->FSR & UART_FSR_TX_EMPTY_F_Msk));
    __set_SP(*(uint32_t *)u32Address);
    func();
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Select IP clock source */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_UART_CLK_DIVIDER(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PC.10 and PC.11 multi-function pins for UART1 RXD, UART1 TXD */
    SYS->PC_H_MFP = (SYS_PC_H_MFP_PC11_MFP_UART1_TX | SYS_PC_H_MFP_PC10_MFP_UART1_RX);
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    int             cbs;
    uint32_t        au32Config[2];
    uint32_t        au32Version[2];

    volatile int    loop;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART1, 115200);

    /* Enable FMC ISP function */
    SYS_UnlockReg();
    FMC_Open();

    FMC_ReadConfig(au32Config, 2);
    cbs = (au32Config[0] >> 6) & 0x3;
    printf_UART("Config0 = 0x%x, Config1 = 0x%x, CBS=%d\n\n", au32Config[0], au32Config[1], cbs);

    printf_UART("\n\n\n");
    printf_UART("+---------------------------------------------------+\n");
    printf_UART("|       Boot loader program running on LDROM        |\n");
    printf_UART("+---------------------------------------------------+\n");

    au32Version[0] = FMC_Read(USER_AP0_ENTRY+0x1000);
    au32Version[1] = FMC_Read(USER_AP1_ENTRY+0x1000);

    printf_UART("|               APROM Version Check                 |\n");
    printf_UART("+---------------------------------------------------|\n");
    printf_UART("|        Version for Application No.0:0x%x          |\n",au32Version[0]);
    printf_UART("|        Version for Application No.1:0x%x          |\n",au32Version[1]);
    printf_UART("+---------------------------------------------------|\n");
    printf_UART("|                  Boot Selection                   |\n");
    if((au32Version[0]>=au32Version[1])&(au32Version[0]!=0xFFFFFFFF))
    {
        printf_UART("|AP0 has latest program and then system jumps to AP0|\n");
        BranchTo(USER_AP0_ENTRY);
    }
    else if(au32Version[1]!=0xFFFFFFFF)
    {
        printf_UART("|AP1 has latest program and then system jumps to AP1|\n");
        BranchTo(USER_AP1_ENTRY);
    }
    if((au32Version[0]<=au32Version[1])&(au32Version[1]!=0xFFFFFFFF))
    {
        printf_UART("|AP1 has latest program and then system jumps to AP1|\n");
        BranchTo(USER_AP1_ENTRY);
    }
    else if(au32Version[0]!=0xFFFFFFFF)
    {
        printf_UART("|AP0 has latest program and then system jumps to AP0|\n");
        printf_UART("+---------------------------------------------------+\n");
        BranchTo(USER_AP0_ENTRY);
    }
    printf_UART("| Don't find any program on APROM                   |\n");
    printf_UART("| Please implement ISP function to update APROM     |\n");
    printf_UART("+---------------------------------------------------+\n");
    while(1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Empty functions for reduce code size to fit into LDROM & solve the functions are not be defined.       */
/*---------------------------------------------------------------------------------------------------------*/
void ProcessHardFault()
{}

void SH_Return()
{}

