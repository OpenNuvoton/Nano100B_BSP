/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/05/26 8:08a $
 * @brief    Demonstrate how to keep LCD display while system enters Power-down mode.
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
#include "sys.h"
#include "rtc.h"
#include "LCDLIB.h"

#ifdef __DEBUG_MSG
#define DEBUG_MSG   printf
#else
#define DEBUG_MSG(...)
#endif

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

/* External functions */
void textticker(char *string, uint32_t delayus);
void showTime(uint32_t hour, uint32_t minute);
void planNextRTCInterrupt(S_RTC_TIME_DATA_T *sCurTime);

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
  * @brief  Restore original setting of multi-function pin selection.
  * @param  None.
  * @return None.
  */
void RestorePinSetting()
{
    /* Restore Pin selection setting */
    SYS->PA_L_MFP = _Pin_Setting[0];
    SYS->PA_H_MFP = _Pin_Setting[1];
    SYS->PB_L_MFP = _Pin_Setting[2];
    SYS->PB_H_MFP = _Pin_Setting[3];
    SYS->PC_L_MFP = _Pin_Setting[4];
    SYS->PC_H_MFP = _Pin_Setting[5];
    SYS->PD_L_MFP = _Pin_Setting[6];
    SYS->PD_H_MFP = _Pin_Setting[7];
    SYS->PE_L_MFP = _Pin_Setting[8];
    SYS->PE_H_MFP = _Pin_Setting[9];
    SYS->PF_L_MFP = _Pin_Setting[10];

    /* Restore Pull-up setting */
    PA->PUEN = _PullUp_Setting[0];
    PB->PUEN = _PullUp_Setting[1];
    PC->PUEN = _PullUp_Setting[2];
    PD->PUEN = _PullUp_Setting[3];
    PE->PUEN = _PullUp_Setting[4];
    PF->PUEN = _PullUp_Setting[5];
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
    SYS->PB_L_MFP = 0;
    SYS->PB_H_MFP = 0;
    SYS->PC_L_MFP = 0;
    SYS->PC_H_MFP = 0;
    SYS->PD_L_MFP = 0;
    SYS->PD_H_MFP = 0;
    SYS->PE_L_MFP = 0;
    SYS->PE_H_MFP = 0;
    SYS->PF_L_MFP = 0x00007700;

    /* Enable GPIO pull up */
    PA->PUEN = 0xFFFF;
    PB->PUEN = 0xFFFF;
    PC->PUEN = 0xFFFF;
    PD->PUEN = 0xFFFF;
    PE->PUEN = 0xFFFF;
    PF->PUEN = 0x0033;      /* exclude GPF2 and GPF3 which are HXT OUT/IN */

    /* Disable LCD clock */
    CLK->APBCLK &= ~CLK_APBCLK_LCD_EN; /* Disable LCD clock */
    CLK->PWRCTL &= ~CLK_PWRCTL_LXT_EN_Msk; /* disable LXT - 32Khz */

    CLK_PowerDown();

}


/**
  * @brief  This function will not be called.
  * @param  None.
  * @return None.
  */
void Leave_PowerDown()
{

    /* Restore pin setting */
    RestorePinSetting();

    /* Set PF.0 and PF.1 to ICE Data and Clock */
    SYS->PF_L_MFP |= 0x00000077;

    /* Enable LCD clock */
    CLK->PWRCTL |= CLK_PWRCTL_LXT_EN_Msk; /* enable LXT - 32Khz */
    CLK->APBCLK |= CLK_APBCLK_LCD_EN; /* enable LCD clock */

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
    CLK->APBCLK |= CLK_APBCLK_RTC_EN;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->PA_H_MFP &= ~(SYS_PA_H_MFP_PA14_MFP_Msk|SYS_PA_H_MFP_PA15_MFP_Msk);
    SYS->PA_H_MFP |=  (SYS_PA_H_MFP_PA14_MFP_UART0_RX|SYS_PA_H_MFP_PA15_MFP_UART0_TX);

    /* Select LCD COMs, SEGs, V1 ~ V3, DH1, DH2 */
    MFP_LCD_TYPEA();

    /* Digital Input Path Disable */
    /* V1, V2 and V3 */
    PB->OFFD |= (0xE000000);
    /* DH1 and DH2 */
    PC->OFFD |= (0x30000);
    /* COM0~3 */
    PC->OFFD |= (0x3C0000);
    /* SEG0~39 */
    PA->OFFD |= (0x0FF00000);
    PB->OFFD |= (0xF1FF0000);
    PC->OFFD |= (0xC0000000);
    PD->OFFD |= (0xFFF00000);
    PE->OFFD |= (0xE1800000);

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
    S_RTC_TIME_DATA_T sCurTime;

    SYS_Init();
    UART0_Init();

    DEBUG_MSG("\nNANO130 NuTiny EVB Demo Program V1.0.0\n");
    DEBUG_MSG("[Power Down Demo]\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Time Setting */
    sCurTime.u32Year       = 2013;
    sCurTime.u32Month      = 10;
    sCurTime.u32Day        = 15;
    sCurTime.u32Hour       = 0;
    sCurTime.u32Minute     = 0;
    sCurTime.u32Second     = 0;
    sCurTime.u32DayOfWeek  = RTC_TUESDAY;
    sCurTime.u32TimeScale  = RTC_CLOCK_24;

    RTC_Open(&sCurTime);
    DEBUG_MSG("RTC Init. complete!\n");

    /* Do LCD Initializaton */
    LCD_Open(LCD_C_TYPE, 4, LCD_BIAS_THIRD, LCD_FREQ_DIV64, LCD_CPVOl_3V);
    LCD_EnableDisplay();

    DEBUG_MSG("LCD Init. complete!\n");

    /* Start displaying on LCD */
    LCDLIB_SetSymbol(2, 27, 1);
    CLK_SysTickDelay(335000);

    textticker("*** NUVOTON NANO130 ***", 335000);
    CLK_SysTickDelay(335000);

    LCDLIB_Printf(0, "NUVOTON");
    LCD_EnableBlink(250);
    CLK_SysTickDelay(335000);
    CLK_SysTickDelay(335000);
    CLK_SysTickDelay(335000);
    CLK_SysTickDelay(335000);
    CLK_SysTickDelay(335000);
    LCD_DisableBlink();

    /* Read curent RTC time */
    RTC_GetDateAndTime(&sCurTime);
    DEBUG_MSG("Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);

    /* Display RTC time */
    showTime(sCurTime.u32Hour, sCurTime.u32Minute);

    DEBUG_MSG("Start MAIN loop.\n");
    while(1)
    {
        textticker("*** GOING TO POWER DOWN ***", 335000);
        DEBUG_MSG("Going to Power Down...\n");
#ifdef __DEBUG_MSG
        while(!(UART1->FSR & UART_FSR_TE_F)) ;  /* waits for message send out */
#endif
        /* Enter power down mode */
        /* This sample will not wake up*/
        Enter_PowerDown();

        DEBUG_MSG("Program resume...\n");

        if (_Wakeup_Flag == 1)
        {
            _Wakeup_Flag = 0;

            textticker("*** WAKE UP ***", 335000);
            CLK_SysTickDelay(335000);
            CLK_SysTickDelay(335000);
            CLK_SysTickDelay(335000);
        }
    };


}


#ifdef USE_ASSERT
/**
  * @brief  The function prints the source file name and line number where the assert_param() error
  *         occurs, and then stops in an infinite loop. User can add his own codes here if necessary.
  * @param[in] file Source file name
  * @param[in] line Line number
  * @return None
  */
void assert_error(uint8_t * file, uint32_t line)
{
    MFP_UART0_TO_PORTA();                  /* UART0 TX/RX to PA14/PA15*/
    CLK->APBCLK |= CLK_APBCLK_UART0_EN;    /* Enable UART0 clock */
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_UART_MASK;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_MASK) | CLK_CLKSEL1_UART_HXT;  /* Select 12 Mhz XTAL */


    /* Set UART to 115200,n,8,1,none */
    UART0->BAUD = 0x67;             /* Baud Rate:115200 for 12MHz */
    UART0->TLCTL = 0x03;            /* Word len is 8 bits */

    printf("[%s] line %d : wrong parameters.\r\n", file, line);

    /* Infinite loop */
    while(1) ;

}
#endif


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



