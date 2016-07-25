
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/12 4:59p $
 * @brief    Demonstrate LDROM updated through ISP function by branching to
 *           LDROM by software reset, and show debug messages via UART.
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"



int32_t main (void)
{
    int i;
    /*  Configure PE2 as Input mode pull-up and enable interrupt by falling edge trigger */
    GPIO_SetMode(PF, BIT5, GPIO_PMD_OUTPUT);
    while(1) {
        for(i=0; i<10; i++)
            CLK_SysTickDelay(10000);

        PF5 = PF5 ^ 1;
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


