/**************************************************************************//**
 * @file     RTC_Common.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/01/06 8:10p $
 * @brief    Nano100 series RTC driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "Nano100Series.h"

void planNextRTCInterrupt(S_RTC_TIME_DATA_T *sCurTime)
{
    // plan next interrupt timing
    if(sCurTime->u32Minute < 59)
        sCurTime->u32Minute += 1;
    else {
        if(sCurTime->u32Hour < 23)
            sCurTime->u32Hour += 1;
        else {  // next day
            sCurTime->u32Hour = 0;

            // new year first day
            if(sCurTime->u32Month==12 && sCurTime->u32Day==31) {
                sCurTime->u32Year += 1;
                sCurTime->u32Month = 1;
                sCurTime->u32Day = 1;
            } else if(sCurTime->u32Month==1 ||
                      sCurTime->u32Month==3 ||
                      sCurTime->u32Month==5 ||
                      sCurTime->u32Month==7 ||
                      sCurTime->u32Month==8 ||
                      sCurTime->u32Month==10 ||
                      sCurTime->u32Month==12) { // 1,3,5,7,8,10,12 31-day month
                if(sCurTime->u32Day < 31)
                    sCurTime->u32Day += 1;
                else {
                    sCurTime->u32Day = 1;
                    sCurTime->u32Month += 1;
                }
            } else if(sCurTime->u32Month==2) { // 2, 28 or 29-day month
                if(RTC_IS_LEAP_YEAR()) { // leap year
                    if(sCurTime->u32Day < 29)
                        sCurTime->u32Day += 1;
                    else {
                        sCurTime->u32Day = 1;
                        sCurTime->u32Month += 1;
                    }
                } else {
                    if(sCurTime->u32Day < 28)
                        sCurTime->u32Day += 1;
                    else {
                        sCurTime->u32Day = 1;
                        sCurTime->u32Month += 1;
                    }
                }
            } else if(sCurTime->u32Month==4 ||
                      sCurTime->u32Month==6 ||
                      sCurTime->u32Month==9 ||
                      sCurTime->u32Month==11) { // 4,6,9,11 30-day
                if(sCurTime->u32Day < 30)
                    sCurTime->u32Day += 1;
                else {
                    sCurTime->u32Day = 1;
                    sCurTime->u32Month += 1;
                }
            }
        }

        sCurTime->u32Month = 0;
    }
    sCurTime->u32Second = 0;

    RTC_SetAlarmDateAndTime(sCurTime);
    RTC_EnableInt(RTC_RIER_AIER_Msk);
    NVIC_EnableIRQ(RTC_IRQn);

}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



