/**************************************************************************//**
 * @file     LCD_Commom.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/26 2:12p $
 * @brief    Common files for Tiny board sample code.
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
#include "LCDLIB.h"
#include "sys.h"

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

void ControlSegment(uint32_t onoff, int com, int seg)
{
    LCD_SetPixel(com, seg, onoff);
}


void textticker(char *string, uint32_t delayus)
{
    char showstring[7], tmpstring[7];
    int showidx, textidx, storeshowidx, storetextidx;
    int textlen;
    uint32_t i;

    strcpy(&showstring[0], "       ");

    textlen = strlen(string);

    showidx = 6;

    for(textidx=0; textidx<textlen; textidx++) {
        // clear showstring
        for(storetextidx=0; storetextidx<=6; storetextidx++) showstring[storetextidx] = ' ';

        storetextidx = textidx;
        if((6-showidx) > 0) {
            storetextidx -= (6-showidx);
        }
        for(storeshowidx=showidx; storeshowidx<=6; storeshowidx++) {
            showstring[storeshowidx] = string[storetextidx++];
        }
        if(showidx!=0)
            showidx--;
        //printf("%s \r\n", showstring);
        LCDLIB_Printf(0, &showstring[0]);
        CLK_SysTickDelay(delayus);

    }

    for(showidx=0; showidx<=6; showidx++) {
        strcpy(tmpstring, showstring);
        for(storeshowidx=0; storeshowidx<=6; storeshowidx++) {
            for(i = 0; i < 10000; i++);

            if((storeshowidx+1) <= 6)
                showstring[storeshowidx] = tmpstring[storeshowidx+1];
            else
                showstring[storeshowidx] = ' ';
        }

        LCDLIB_Printf(0, &showstring[0]);
        CLK_SysTickDelay(delayus);
    }

}

// hour: 24-hours
void showTime(uint32_t hour, uint32_t minute)
{
    long long time;
    // show time
    time = hour * 100 + minute;

    LCDLIB_PrintNumber(1, time);
    LCD_SetPixel(3,29,1);
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



