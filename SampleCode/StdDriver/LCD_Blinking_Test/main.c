/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 14/09/11 5:22p $
 * @brief    Demonstrate LCD blinking function.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "Nano100Series.h"
#include "lcd.h"
#include "uart.h"
#include "sys.h"
#include "clk.h"

#include "LCDLIB.h"

#define LCD_ALPHABET_NUM        7
#define LCD_DIGIT_NUM           4

/*!<Enable LCD for 100/128-Pin Package */
#define MFP_LCD_TYPEA() { \
                            SYS->PA_L_MFP |= 0x77770000;    /* seg 36 ~ 39 */\
                            SYS->PA_H_MFP |= 0x7777;        /* seg 20 ~ 23 */\
                            SYS->PB_L_MFP = 0x77777777;     /* seg 10 ~ 13, 4 ~ 7 */\
                            SYS->PB_H_MFP = 0x77777777;     /* LCD V1 ~ V3, seg 30 ~ 31, 24 ~ 26 */\
                            SYS->PC_L_MFP |= 0x777777;      /* LCD COM3 ~ COM0, DH1/DH2 */\
                            SYS->PC_H_MFP |= 0x77000000;    /* seg 32 ~ 33 */\
                            SYS->PD_L_MFP |= 0x77770000;    /* seg 2 ~ 3, 34 ~ 35 */\
                            SYS->PD_H_MFP = 0x77777777;     /* seg 0 ~ 1, 14 ~ 19 */\
                            SYS->PE_L_MFP |= 0x70000000;    /* seg 8 */\
                            SYS->PE_H_MFP |= 0x77700007;    /* seg 9, 27 ~ 29 */\
                        }

/*!<Enable LCD for 64-Pin Package */
#define MFP_LCD_TYPEB() { \
                            SYS->PA_L_MFP |= 0x77777700;    /* seg 18 ~ 23 */\
                            SYS->PA_H_MFP = 0x77777777;     /* seg 6 ~ 9, 24 ~ 27 */\
                            SYS->PB_L_MFP = 0x77777777;     /* COM2, COM3, seg 0 ~ 5 */\
                            SYS->PB_H_MFP = 0x77777777;     /* LCD V1 ~ V3, seg 10 ~ 14 */\
                            SYS->PC_L_MFP |= 0x70007777;    /* LCD COM1 ~ COM0, DH1/DH2, seg 17 */\
                            SYS->PC_H_MFP |= 0x77007777;    /* seg 28 ~ 31, 15 ~ 16 */\
                        }

long long char_to_int(char c)
{
    if(c=='0') return 0;
    else if(c=='1') return 1;
    else if(c=='2') return 2;
    else if(c=='3') return 3;
    else if(c=='4') return 4;
    else if(c=='5') return 5;
    else if(c=='6') return 6;
    else if(c=='7') return 7;
    else if(c=='8') return 8;
    else if(c=='9') return 9;

    return -1;
}

long long local_atoi(char text[])
{
    int len = strlen(text);
    int len2, negflag=0;
    long long mul=len;
    long long i=0, j=0, mul2=1;
    long long result=0;

    if( text[0] == '-') {
        negflag = 1;
        len2 = len - 1;
        for(i=0; i<len2; i++) {
            text[i] = text[i+1];
        }
        text[i] = '\0';
        len--;
        mul = len;
    }

    for(i=0; i < len; i++) {
        if(mul==1) mul2 = 1;
        else if(mul>1)
            for(j=0; j<(mul-1); j++)
                mul2 *= 10;
        result += mul2*char_to_int(text[i]);
        mul--;
        mul2=1;
    }

    if(negflag==1)
        result = 0 - result;

    return result;
}

uint32_t sysGetNum(void)
{
    uint8_t cInputTemp=0x00, InputString[16]= {0};
    uint32_t nLoop = 0;
    while(cInputTemp != 0x0D) {
        cInputTemp = getchar();
        if(cInputTemp == 27) {
            return cInputTemp;
        }
        if(cInputTemp == 'x' || cInputTemp == 'X' || cInputTemp == 'f'||
                cInputTemp == 'F' || cInputTemp == 'r' || cInputTemp == 'R') {
            return cInputTemp;
        }
        if(cInputTemp == '-') {
            InputString[nLoop] = cInputTemp;
            printf("%c",cInputTemp);
            nLoop++;
        } else if(cInputTemp >= '0' && cInputTemp <= '9') {
            InputString[nLoop] = cInputTemp;
            printf("%c",cInputTemp);
            nLoop++;
        }
    }
    return local_atoi((char *)InputString);
}


/**
 * @brief  LCD ISR to handle interrupt event
 * @param  None
 * @retval None
 */
void LCD_IRQHandler(void)
{

    if( LCD->FCSTS & LCD_FCSTS_FCSTS_Msk) {
        LCD->FCSTS = LCD_FCSTS_FCSTS_Msk;

        printf("IST: LCD Frame Count interrupt...\n");
    } else if( LCD->FCSTS & LCD_FCSTS_PDSTS_Msk) {
        LCD->FCSTS = LCD_FCSTS_PDSTS_Msk;

        printf("IST: LCD Power Down interrupt...\n");
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

    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_LXT_EN_Pos); // LXT Enable

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);
    /* Waiting for 32KHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_LXT_STB_Msk);

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
    CLK->APBCLK |= CLK_APBCLK_LCD_EN;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD  */
    SYS->PA_H_MFP &= ~(SYS_PA_H_MFP_PA14_MFP_Msk|SYS_PA_H_MFP_PA15_MFP_Msk);
    SYS->PA_H_MFP |=  (SYS_PA_H_MFP_PA14_MFP_UART0_RX|SYS_PA_H_MFP_PA15_MFP_UART0_TX);

    /* Select LCD COMs, SEGs, V1 ~ V3, DH1, DH2 */
    MFP_LCD_TYPEA();

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


static void TestItem (void)
{
    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|                    LCD Sample Program                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Blinking Display Test                                    |\n");
    printf("+-----------------------------------------------------------+\n");
}

int32_t main(void)
{
    char text[LCD_ALPHABET_NUM]="";
    int32_t blink_time;

    SYS_Init();

    UART0_Init();

    /* LCD Initialize */
    LCD_Open(LCD_C_TYPE, 4, LCD_BIAS_THIRD, LCD_FREQ_DIV64, LCD_CPVOl_3V);

    LCD_EnableDisplay();

    strcpy(text, "");   // clear buffer

    TestItem();

    LCDLIB_Printf(0, "NUVOTON");

    printf("Input the blinking time(ms): ");
    blink_time = sysGetNum();
    printf("\n");

    LCD_EnableInt(LCD_FRAMECOUNT_INT);
    NVIC_EnableIRQ(LCD_IRQn);
    LCD_EnableBlink(blink_time);

    printf("Any key to end Blinking display...");
    getchar();
    LCD_DisableBlink();

    LCD_DisableDisplay();

    while(1);

}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



