/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/09/11 7:10p $
 * @brief    Demonstrate how to read phone book information in the SIM card.
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "sclib.h"

/* The definition of commands used in this sample code and directory structures could
   be found in GSM 11.11 which is free for download from Internet. */

// Select File
const uint8_t au8SelectMF[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
const uint8_t au8SelectDF_TELECOM[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x7F, 0x10};
const uint8_t au8SelectEF_ADN[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x6F, 0x3A};
//Get Response
uint8_t au8GetResp[] = {0xA0, 0xC0, 0x00, 0x00, 0x00};
//Read Record
uint8_t au8ReadRec[] = {0xA0, 0xB2, 0x01, 0x04, 0x00};
//Verify CHV, CHV = Card Holder Verification information
uint8_t au8VerifyChv[] = {0xA0, 0x20, 0x00, 0x01, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t buf[300];
uint32_t len;

/**
  * @brief  The interrupt services routine of smartcard port 1
  * @param  None
  * @return None
  */
void SC1_IRQHandler(void)
{
    // Please don't remove any of the function calls below
    if(SCLIB_CheckCDEvent(1))
        return; // Card insert/remove event occurred, no need to check other event...
    SCLIB_CheckTimeOutEvent(1);
    SCLIB_CheckTxRxEvent(1);
    SCLIB_CheckErrorEvent(1);

    return;
}

/**
  * @brief  Ask user to input PIN from console
  * @param  None
  * @return None
  * @details Valid input characters (0~9) are echo to console and store in command buffer.
  *         Backspace key can delete previous input digit, ESC key delete all input digits.
  *         Valid PIN length is between 4~8 digits. If PIN length is shorter than 8
  *         digits, an Enter key can terminate the input procedure.
  */
void get_pin(void)
{
    int i = 0;
    char c = 0;

    printf("Please input PIN number:");
    while(i < 8)
    {
        c = getchar();
        if(c >= 0x30 && c <= 0x39)      // Valid input characters (0~9)
        {
            au8VerifyChv[5 + i] = c;
            printf("%c", c);
            i++;
        }
        else if(c == 0x7F)    // DEL (Back space)
        {
            i--;
            printf("%c", c);
        }
        else if(c == 0x0D)     // Enter
        {
            if(i >= 4)  //Min CHV length is 4 digits
                break;
        }
        else if(c == 0x1B)    //ESC
        {
            printf("\nPlease input PIN number:");
            i = 0;  // retry
        }
        else
        {
            continue;
        }

    }

    // Fill remaining digits with 0xFF
    for(; i < 8; i++)
    {
        au8VerifyChv[5 + i] = 0xFF;
    }

    printf("\n");

    return;
}

/**
  * @brief  Send verify command to verify CHV1
  * @param  Remaining retry count, valid values are between 3~1
  * @return Unlock SIM card success or not
  * @retval 0 Unlock success
  * @retval -1 Unlock failed
  */
int unlock_sim(uint32_t u32RetryCnt)
{
    while(u32RetryCnt > 0)
    {

        get_pin(); // Ask user input PIN

        if(SCLIB_StartTransmission(1, au8VerifyChv, 13, buf, &len) != SCLIB_SUCCESS)
        {
            printf("Command Verify CHV failed\n");
            break;
        }
        if(buf[0] == 0x90 || buf[1] == 0x00)
        {
            printf("Pass\n");
            return 0;
        }
        else
        {
            u32RetryCnt--;
            printf("Failed, remaining retry count: %d\n", u32RetryCnt);
        }
    }

    printf("Oops, SIM card locked\n");

    return -1;
}

/**
  * @brief  Read phone book and print on console
  * @param  Phone book record number
  * @return None
  */
void read_phoneBook(uint32_t cnt)
{

    int i, j, k;

    for(i = 1; i < cnt + 1; i++)
    {
        au8ReadRec[2] = (uint8_t)i;
        if(SCLIB_StartTransmission(1, au8ReadRec, 5, buf, &len) != SCLIB_SUCCESS)
        {
            printf("Command Read Record failed\n");
            break;
        }
        if(buf[0] == 0xFF) // This is an empty entry
            continue;
        printf("\n======== %d ========", i);
        printf("\nName: ");
        for(j = 0; buf[j] != 0xFF; j++)
        {
            printf("%c", buf[j]);
        }
        while(buf[j] == 0xFF)   // Skip reset of the Alpha Identifier bytes
            j++;

        printf("\nNumber: ");
        j += 2; // Skip Length of BCD and TNO/NPI
        for(k = 0; k < 10; k++)
        {
            if((buf[j + k] & 0xf) != 0xF)
                printf("%c", (buf[j + k] & 0xf) + 0x30);
            else
                break;

            if((buf[j + k] >> 4) != 0xF)
                printf("%c", (buf[j + k] >> 4) + 0x30);
            else
                break;
        }
    }
    printf("\n");
    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SC1_MODULE);


    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(SC1_MODULE, CLK_CLKSEL2_SC_S_HXT, CLK_SC1_CLK_DIVIDER(3)); // SC clock is 4MHz

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPA.14 and GPA.15 multi-function pins for UART0 RXD and TXD */
    SYS->PA_H_MFP &= ~(SYS_PA_H_MFP_PA14_MFP_Msk | SYS_PA_H_MFP_PA15_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA14_MFP_UART0_RX | SYS_PA_H_MFP_PA15_MFP_UART0_TX);
    /* Set PD.0~PD.4 for SC1 interface */
    SYS->PD_L_MFP &= ~(SYS_PD_L_MFP_PD0_MFP_Msk |
                       SYS_PD_L_MFP_PD1_MFP_Msk |
                       SYS_PD_L_MFP_PD2_MFP_Msk |
                       SYS_PD_L_MFP_PD3_MFP_Msk |
                       SYS_PD_L_MFP_PD4_MFP_Msk);

    SYS->PD_L_MFP |= (SYS_PD_L_MFP_PD0_MFP_SC1_CLK |
                      SYS_PD_L_MFP_PD1_MFP_SC1_DAT |
                      SYS_PD_L_MFP_PD2_MFP_SC1_PWR |
                      SYS_PD_L_MFP_PD3_MFP_SC1_RST |
                      SYS_PD_L_MFP_PD4_MFP_SC1_CD);


    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    int retval;
    int retry = 0, cnt, chv1_disbled = 0;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\nThis sample code reads ATR from smartcard\n");

    // Open smartcard interface 1. CD pin state low indicates card insert and PWR pin high raise VCC pin to card
    SC_Open(SC1, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
    NVIC_EnableIRQ(SC1_IRQn);

    // Wait 'til card insert
    while(SC_IsCardInserted(SC1) == FALSE);
    // Activate slot 1
    retval = SCLIB_Activate(1, FALSE);

    if(retval != SCLIB_SUCCESS)
    {
        printf("SIM card activate failed\n");
        goto exit;
    }

    if(SCLIB_StartTransmission(1, (uint8_t *)au8SelectMF, 7, buf, &len) != SCLIB_SUCCESS)
    {
        printf("Command Select MF failed\n");
        goto exit;
    }


    if(len == 2 && buf[0] == 0x9F )    // response data length
    {
        au8GetResp[4] = buf[1];
        if(SCLIB_StartTransmission(1, au8GetResp, 5, buf, &len) != SCLIB_SUCCESS)
        {
            printf("Command Get response failed\n");
            goto exit;
        }
    }
    else
    {
        printf("Unknown response\n");
        goto exit;
    }

    if(buf[len - 2] != 0x90 || buf[len - 1] != 0x00)
    {
        printf("Cannot select MF\n");
        goto exit;
    }

    // Check if SIM is locked
    if(buf[18] & 0x80)
    {
        if((retry = (buf[18] & 0xF)) == 0)
        {
            printf("SIM locked, and unlock retry count exceed\n");
            goto exit;
        }
    }
    // Some SIM cards has file protect by CHV1, but CHV1 disabled.
    if(buf[13] & 0x80)
    {
        printf("CHV1 disabled\n");
        chv1_disbled = 1;
    }
    if(SCLIB_StartTransmission(1, (uint8_t *)au8SelectDF_TELECOM, 7, buf, &len) != SCLIB_SUCCESS)
    {
        printf("Command Select DF failed\n");
        goto exit;
    }

    if(SCLIB_StartTransmission(1, (uint8_t *)au8SelectEF_ADN, 7, buf, &len) != SCLIB_SUCCESS)
    {
        printf("Command Select EF failed\n");
        goto exit;
    }

    if(len == 2 && buf[0] == 0x9F )    // response data length
    {
        au8GetResp[4] = buf[1];
        if(SCLIB_StartTransmission(1, au8GetResp, 5, buf, &len) != SCLIB_SUCCESS)
        {
            printf("Command Get response failed\n");
            goto exit;
        }
    }
    else
    {
        printf("Unknown response\n");
        goto exit;
    }

    au8ReadRec[4] = buf[14]; // Phone book record length
    cnt = ((buf[2] << 8) + buf[3]) / buf[14];   // Phone book record number

    if(((buf[8] & 0x10) == 0x10) && (chv1_disbled == 0))    //Protect by CHV1 ?
    {
        if(unlock_sim(retry) < 0)
        {
            printf("Unlock SIM card failed\n");
            goto exit;
        }
    }

    read_phoneBook(cnt);
    printf("Done\n");
exit:
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/


