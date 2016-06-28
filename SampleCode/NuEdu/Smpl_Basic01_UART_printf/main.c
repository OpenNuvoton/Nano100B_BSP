/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/06/17 7:28p $
 * @brief    Demonstrate a simple printf function to replace the standard printf library
 *           for reducing the code size issue.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdarg.h>

#include "Nano100Series.h"
#include "NuEdu-Basic01.h"
void SendChar_ToUART(int ch);

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

    /* Set PC.10 and PC.11 multi-function pins for UART1 RXD, UART0 TXD */
    SYS->PC_H_MFP = (SYS_PC_H_MFP_PC11_MFP_UART1_TX | SYS_PC_H_MFP_PC10_MFP_UART1_RX);

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple printf() function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void printf_UART(uint8_t *str,...);
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
    }while (u32Temp != 0);
    if(j)
    {
        i--;
        *(print_buf+i) = '-';        
    }
    printf_UART(print_buf+i);
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
    }while (u32Temp != 0);
    printf_UART(print_buf+i);
}
void printf_UART(uint8_t *str,...)
{
		va_list args;
		va_start( args, str );
    while (*str != '\0')
    {
			if(*str == '%')
			{
				str++;
				if (*str == '\0') return;
				if( *str == 'd' )
				{
					str++;
					printInteger(va_arg( args, int ));
				}else if( *str == 'x' )
				{
					str++;
					printHex(va_arg( args, int ));
				}             
			}
        SendChar_ToUART(*str++);
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main()
{
    uint32_t u32Key,i=0;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();	
	
    /* Init Key and LED GPIO type */
    GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);
    Initial_KEY_INPUT();
    initial_led();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART1, 115200);


    printf("+-----------------------------------------+\n");
    printf("|    Nano100 Series UART Sample Code      |\n");
    printf("+-----------------------------------------+\n");	
	
    while(1)
    {
        /* Detect Key status */
        u32Key = Get_KEY_INPUT();
        if(PB14==0)
        {
            LED_on(i);
            printf("+----------------------------------+\n");
            printf("|    Standare printf function:%d   |\n",i++);
            printf("+----------------------------------+\n");
        }
        if((u32Key & 0x01)==0)
        {
            LED_on(i);
            printf_UART("+------------------------------+\n");
            printf_UART("|  Simple printf function:%d   |\n",i--);
            printf_UART("+------------------------------+\n");
        }
    }
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
