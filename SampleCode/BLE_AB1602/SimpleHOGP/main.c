/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A BLE simple HOGP sample.
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Nano100Series.h"
#include "bt.h"
#include "bt_porting.h"
#include "SimpleHOGP.h"

void SYS_Init(void)
{
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX );
}

int main()
{
    int8_t ch;

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /*airoha porting init*/
    Airoha_PortingInit();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);


    /* Bt Init*/
    SimpleHOGP_Init();

    printf("Please Input Any Key\n\n");
    printf("[ for volumn down\n");
    printf("] for volumn up\n");
    do
    {
        ch = getchar();
        switch(ch)
        {
        case '[':
            SimpleHOGP_Send_MediaKey(0x00ea);       // Vol -
            break;
        case ']':
            SimpleHOGP_Send_MediaKey(0x00e9);       // Vol +
            break;

        case '!':
            SimpleHOGP_Send_Key(0x02, 0x1e);
            break;
        case '@':
            SimpleHOGP_Send_Key(0x02, 0x1f);
            break;
        case '#':
            SimpleHOGP_Send_Key(0x02, 0x20);
            break;
        case '$':
            SimpleHOGP_Send_Key(0x02, 0x21);
            break;
        case '%':
            SimpleHOGP_Send_Key(0x02, 0x22);
            break;
        case '^':
            SimpleHOGP_Send_Key(0x02, 0x23);
            break;
        case '&':
            SimpleHOGP_Send_Key(0x02, 0x24);
            break;
        case '*':
            SimpleHOGP_Send_Key(0x02, 0x25);
            break;
        case '(':
            SimpleHOGP_Send_Key(0x02, 0x26);
            break;
        case '0':
            SimpleHOGP_Send_Key(0x00, 0x27);
            break;
        case ')':
            SimpleHOGP_Send_Key(0x02, 0x27);
            break;
        case 0x0a:      // Enter
            break;
        case 0x0d:      // Enter
            SimpleHOGP_Send_Key(0x00, 0x28);
            break;
        case 0x1B:      // Escape
            SimpleHOGP_Send_Key(0x00, 0x29);
            break;
        case 0x08:      // Backspace
            SimpleHOGP_Send_Key(0x00, 0x2a);
            break;
        case 0x09:      // Tab
            SimpleHOGP_Send_Key(0x00, 0x2b);
            break;
        case ' ':      // Space
            SimpleHOGP_Send_Key(0x00, 0x2c);
            break;
        case '-':
            SimpleHOGP_Send_Key(0x00, 0x2d);
            break;
        case '_':
            SimpleHOGP_Send_Key(0x02, 0x2d);
            break;
        case '=':
            SimpleHOGP_Send_Key(0x00, 0x2e);
            break;
        case '+':
            SimpleHOGP_Send_Key(0x02, 0x2e);
            break;
        case '\\':
            SimpleHOGP_Send_Key(0x00, 0x31);
            break;
        case '|':
            SimpleHOGP_Send_Key(0x02, 0x31);
            break;
        case ';':
            SimpleHOGP_Send_Key(0x00, 0x33);
            break;
        case ':':
            SimpleHOGP_Send_Key(0x02, 0x33);
            break;
        case '\'':
            SimpleHOGP_Send_Key(0x00, 0x34);
            break;
        case '"':
            SimpleHOGP_Send_Key(0x02, 0x34);
            break;
        case '`':
            SimpleHOGP_Send_Key(0x00, 0x35);
            break;
        case '~':
            SimpleHOGP_Send_Key(0x02, 0x35);
            break;
        case ',':
            SimpleHOGP_Send_Key(0x00, 0x36);
            break;
        case '<':
            SimpleHOGP_Send_Key(0x02, 0x36);
            break;
        case '.':
            SimpleHOGP_Send_Key(0x00, 0x37);
            break;
        case '>':
            SimpleHOGP_Send_Key(0x02, 0x37);
            break;
        case '/':
            SimpleHOGP_Send_Key(0x00, 0x38);
            break;
        case '?':
            SimpleHOGP_Send_Key(0x02, 0x38);
            break;
        default:
            if (ch >= 'a' && ch <= 'z')
                SimpleHOGP_Send_Key(0x00, ch - 'a' + 0x04);
            else if (ch >= 'A' && ch <= 'Z')
                SimpleHOGP_Send_Key(0x02, ch - 'A' + 0x04);
            else if (ch >= '1' && ch <='9')
                SimpleHOGP_Send_Key(0x00, ch - '1' + 0x1e);
            else
                printf("non-support key %c 0x%02X\n", ch, ch);
            break;
        }

    }
    while(1);
}
