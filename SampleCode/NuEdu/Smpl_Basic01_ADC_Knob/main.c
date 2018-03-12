/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/09/26 2:34p $
 * @brief    Demonstrate how to use ADC to measure variable resistor
 *           and change the LED brightness with PWM according to the ADC conversion results.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "NuEdu-Basic01.h"

void UART1_Init(void);
void Write_LED_Color_Flash(uint32_t Speed);
void Write_LED_Bar(uint32_t Number);

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    uint32_t Volume;
    uint32_t LED_Value;

    //Initial System
    SYS_Init();

    //Initial UART
    UART1_Init();

    //Open Volume Knob Device
    Open_Volume_Knob();

    printf("Volume Knob Value:\n");

    while(1)
    {
        //Get Volume Knob Data
        Volume = Get_Volume_Knob();                 //Volume Range: 0 ~ 4095
        printf("%d\n", Volume);

        //Use Volume Control to LED Flash Speed
        Write_LED_Color_Flash(Volume);

        //Show Volume scale on LED Bar
        LED_Value = Volume * (12 + 1) / 4096;       //LED Bar Count Range: 0 ~ 12
        Write_LED_Bar(LED_Value);

        //Close Volume Knob Device when you want
//      if(Volume_Data==4095)
//          Close_Volume_Knob();
    }
}

void UART1_Init(void)
{
    SYS->PC_H_MFP &= ~( SYS_PC_H_MFP_PC11_MFP_Msk | SYS_PC_H_MFP_PC10_MFP_Msk);
    SYS->PC_H_MFP |= (SYS_PC_H_MFP_PC11_MFP_UART1_TX|SYS_PC_H_MFP_PC10_MFP_UART1_RX);

    SYS_UnlockReg();
    if(!(CLK->CLKSTATUS&CLK_CLKSTATUS_HXT_STB_Msk))
    {
        CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN);                            //Enable XTAL's 12 MHz
        SystemCoreClockUpdate();
    }
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    CLK_EnableModuleClock(UART1_MODULE);
    SYS_LockReg();

    UART_Open(UART1, 115200);
//  printf("\nUART Open\n");
}

#define _LED_Color_Count    3
#define _LED_B              PA14
#define _LED_R              PA12
#define _LED_G              PA13

void Write_LED_Color_Flash(uint32_t Speed)
{
    uint32_t i;
    static uint32_t LED_Count=0;
    volatile uint32_t *ptrLED[_LED_Color_Count] = {&_LED_B, &_LED_R, &_LED_G};

    GPIO_SetMode(PA, BIT14, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT13, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT12, GPIO_PMD_OUTPUT);

    //Delay Time Control
    if(Speed>4096)  Speed = 4096;
    for(i=0; i<((4096-Speed)*100); i++);

    for(i=0; i<_LED_Color_Count; i++)
    {
        if(LED_Count==i)
            *ptrLED[i] = 0; //LED ON
        else
            *ptrLED[i] = 1; //LED OFF
    }

    LED_Count++;
    if(LED_Count==_LED_Color_Count)
        LED_Count = 0;
}

#define _LED_Bar_Count      8
#define _LED1               PB0
#define _LED2               PB1
#define _LED3               PE9
#define _LED4               PE10
#define _LED5               PE11
#define _LED6               PD8
#define _LED7               PD9
#define _LED8               PC7

void Write_LED_Bar(uint32_t Number)
{
    uint32_t i;
    volatile uint32_t *ptrLED[_LED_Bar_Count] = {&_LED1, &_LED2, &_LED3, &_LED4, &_LED5, &_LED6,
                                                 &_LED7, &_LED8
                                                };
    initial_led();
    for(i=0; i<_LED_Bar_Count; i++)
    {
        if(Number>i)
            *ptrLED[i] = 0; //LED ON
        else
            *ptrLED[i] = 1; //LED OFF
    }
}
