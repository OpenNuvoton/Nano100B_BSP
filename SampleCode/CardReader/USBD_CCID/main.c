/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 4 $
 * $Date: 15/07/27 7:07p $
 * @brief    CCID (Circuit card interface device) smart card reader sample code.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "ccid.h"
#include "ccid_if.h"
#include "sclib.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define INT_BUFFER_SIZE     64    /* Interrupt message buffer size */
#define BULK_BUFFER_SIZE    512   /* bulk message buffer size */

uint8_t UsbIntMessageBuffer[INT_BUFFER_SIZE];
uint8_t UsbMessageBuffer[BULK_BUFFER_SIZE];

uint8_t volatile gu8IsDeviceReady;
uint8_t volatile gu8AbortRequestFlag;
uint8_t volatile gu8IsBulkOutReady;
uint8_t volatile gu8IsBulkInReady;

uint8_t *pu8IntInBuf;
uint8_t *pUsbMessageBuffer;
uint32_t volatile u32BulkSize;

int32_t volatile gi32UsbdMessageLength;

/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @retval None
  */
void SC0_IRQHandler(void)
{
    // Please don't remove any of the function calls below
    if(SCLIB_CheckCDEvent(0)) {
        RDR_to_PC_NotifySlotChange();
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), pu8IntInBuf, 2);
        USBD_SET_PAYLOAD_LEN(EP4, 2);
        return; // Card insert/remove event occurred, no need to check other event...
    }
    SCLIB_CheckTimeOutEvent(0);
    SCLIB_CheckTxRxEvent(0);
    SCLIB_CheckErrorEvent(0);

    return;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12MHz HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk);
    CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_HXT, 96000000);
    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_HXT_STB_Msk | CLK_CLKSTATUS_PLL_STB_Msk);

    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_HCLK_CLK_DIVIDER(3));

    /* Select IP clock source */
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_USB_CLK_DIVIDER(2));
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL2_SC_S_HXT, CLK_SC0_CLK_DIVIDER(3)); // SC clock is 4MHz

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB1_MFP_UART0_TX | SYS_PB_L_MFP_PB0_MFP_UART0_RX);

    /* Set PA.8 ~ PA.11 and PB.4 for SC0 interface */
    SYS->PA_H_MFP &= ~(SYS_PA_H_MFP_PA8_MFP_Msk |
                       SYS_PA_H_MFP_PA9_MFP_Msk |
                       SYS_PA_H_MFP_PA10_MFP_Msk |
                       SYS_PA_H_MFP_PA11_MFP_Msk);
    SYS->PB_L_MFP &= ~SYS_PB_L_MFP_PB4_MFP_Msk;

    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA8_MFP_SC0_CLK |
                      SYS_PA_H_MFP_PA9_MFP_SC0_DAT |
                      SYS_PA_H_MFP_PA10_MFP_SC0_PWR |
                      SYS_PA_H_MFP_PA11_MFP_SC0_RST);
    SYS->PB_L_MFP |= SYS_PB_L_MFP_PB4_MFP_SC0_CD;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    UART0->BAUD = 0x67;              /* Baud Rate:115200  OSC:12MHz */
    UART0->TLCTL = 0x03;             /* Character len is 8 bits */
}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    SYS_Init();
    UART0_Init();

    printf("NuMicro USB CCID SmartCard Reader\n");

    // Open smartcard interface 0. CD pin state low indicates card remove and PWR pin low raise VCC pin to card
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
    NVIC_EnableIRQ(SC0_IRQn);

    SC0->IER = SC_IER_CD_IE_Msk;

    USBD_Open(&gsInfo, CCID_ClassRequest, NULL);

    /* Endpoint configuration */
    CCID_Init();
    //Set priority is a must under current architecture. Otherwise smartcard interrupt will be blocked by USBD interrupt
    NVIC_SetPriority (USBD_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();

    while(1);


}



/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

