/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A BLE to UART sample.
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "Nano100Series.h"

/*************************airoha**************************/
#include "bt_porting.h"
#include "bt.h"
#include "ble_gap.h"
#include "cfg_sector_m0.h"
#include "trspx.h"

#define HCLK_CLOCK          42000000
#define UART_TX_PDMA_CH     3
//#define ENABLE_RTS_CTS

/*uart rx buffer control*/
#define RX_BUF_RESET()      (rx_buf_ridx = rx_buf_widx = 0)
#define RX_BUF_PUSH(d)      (rx_buf[(uint8_t)(rx_buf_widx++)] = (d))
#define RX_BUF_POP()        (rx_buf[(rx_buf_ridx++)])
#define RX_BUF_DROP(d)      (rx_buf_ridx += d)
#define RX_BUF_READ(idx)    (rx_buf[(uint8_t)(rx_buf_ridx+idx)])
#define RX_BUF_EMPTY()      ((rx_buf_ridx) == (rx_buf_widx))
#define RX_BUF_FULL()       ((rx_buf_ridx) == ((uint8_t)(rx_buf_widx + 1)))
#define RX_BUF_COUNT()      ((uint8_t)(rx_buf_widx - rx_buf_ridx))

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
int cmd_urx(void *context, int argc, char** argv);
int cmd_utx(void *context, int argc, char** argv);
void UART1_TX_Send(uint32_t len, uint8_t *ptr);
void UART1_IRQHandler(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

static uint8_t rx_buf[256];
static uint8_t rx_buf_ridx, rx_buf_widx;
/*---------------------------------------------------------------------------------------------------------*/
/* static function                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
/* 1000 bytes pending to stop uart1 rx, then 500 bytes will allow again */
static uint8_t RFTX_Pending_Check()
{
    static volatile uint8_t rf_pending = 0;
    if(rf_pending)
    {
        if(BT_Pending_TxCnt() < 500)
        {
            rf_pending = 0;
            NVIC_EnableIRQ(UART1_IRQn);
        }
    }
    else if(BT_Pending_TxCnt() > 1000)
    {
        NVIC_DisableIRQ(UART1_IRQn);
        rf_pending = 1;
    }
    return rf_pending;
}

static void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HXT_EN_Pos); // HXT Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable
    CLK->APBCLK |= CLK_APBCLK_UART1_EN; // UART1 Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX);

    /* Set PB multi-function pins for UART1 RXD, TXD */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB4_MFP_Msk | SYS_PB_L_MFP_PB5_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB4_MFP_UART1_RX | SYS_PB_L_MFP_PB5_MFP_UART1_TX);
#if defined(ENABLE_RTS_CTS)
    /* Set PB multi-function pins for UART1 RTS, CTS */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB6_MFP_Msk | SYS_PB_L_MFP_PB7_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB6_MFP_UART1_RTS  | SYS_PB_L_MFP_PB7_MFP_UART1_CTS);
#endif
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
    /* 8 bytes to save cpu*/
    UART0->TLCTL |= UART_TLCTL_RFITL_8BYTES;
    UART0->TMCTL |= 40;//timeout 40bits
    /*enable uart*/
    UART_EnableInt(UART0, (UART_IER_RDA_IE_Msk /*| UART_IER_THRE_IE_Msk*/ |
                           UART_IER_RTO_IE_Msk));
    NVIC_EnableIRQ(UART0_IRQn);
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 module */
    SYS_ResetModule(UART1_RST);
    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
    /* 8 bytes to save cpu*/
    UART1->TLCTL |= UART_TLCTL_RFITL_8BYTES;
    UART1->TMCTL |= 160;//timeout 80bits
#if defined(ENABLE_RTS_CTS)
    /*enable flow control*/
    UART1->TLCTL |= UART_TLCTL_RTS_TRI_LEV_8BYTES;
    UART_EnableFlowCtrl(UART1);
#endif
    /*enable uart*/
    UART_EnableInt(UART1, (UART_IER_RDA_IE_Msk /*| UART_IER_THRE_IE_Msk*/ |
                           UART_IER_RTO_IE_Msk));
    NVIC_EnableIRQ(UART1_IRQn);
}


void UART1_TX_Send(uint32_t len, uint8_t *ptr)
{
    if(PDMA_IS_CH_BUSY(UART_TX_PDMA_CH))
    {
        /*need to maintain queueing to resist the CTS flow stop!!!!*/
        printf("drop data\n");
        return; //drop data;
    }
    PDMA_Open(1 << UART_TX_PDMA_CH);
    /* UART Tx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(UART_TX_PDMA_CH, PDMA_WIDTH_8, len);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(UART_TX_PDMA_CH, (uint32_t)ptr, PDMA_SAR_INC,
                         (uint32_t)&UART1->THR, PDMA_DAR_FIX);
    /* Set service selection; set Memory-to-Peripheral mode. */
    PDMA_SetTransferMode(UART_TX_PDMA_CH, PDMA_UART1_TX, FALSE, 0);
    /* Trigger PDMA */
    UART1->CTL |= UART_CTL_DMA_TX_EN_Msk;
    PDMA_Trigger(UART_TX_PDMA_CH);
}



/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    const void *btdata[6] = {NULL/*mp_data*/, &cfg_sector};
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /*airoha porting init*/
    Airoha_PortingInit();
    /* Lock protected registers */
    SYS_LockReg();
    /*for debug*/
    GPIO_SetMode(PB, BIT13, GPIO_PMD_OUTPUT);
    PB13 = 0;
    /* Init UART0 for printf and testing */
    UART0_Init();
    /* Init UART1 for BLE to UART */
    UART1_Init();
    RX_BUF_RESET();
    /* Bt Init*/
    BT_InitEx2(TRSPX_bt_evt_hdl, (void**)btdata, 2, BT_LOG_HCI_CMD | BT_LOG_HCI_EVT);

    TRSPX_init();
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    while(1)
    {
        RFTX_Pending_Check();
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint32_t u32IntSts = UART0->ISR;
    if(u32IntSts & UART_ISR_RDA_IS_Msk)
    {
        /* Get all the input characters */
        while(!UART_GET_RX_EMPTY(UART0))
        {
            /* Get the character from UART Buffer */
            UART_READ(UART0);
        }
    }
    if(u32IntSts & UART_ISR_THRE_IS_Msk)
    {
    }
}


void UART1_IRQHandler(void)
{
    uint8_t tout = 0;
    uint32_t u32IntSts = UART1->ISR;
    /*Check if RF pending too much*/
    if(RFTX_Pending_Check())
        return;
    if(u32IntSts & (UART_ISR_RTO_IS_Msk))
        tout = 1;
    while(!UART_GET_RX_EMPTY(UART1))
    {
        uint8_t dat;
        if(RX_BUF_FULL())
            break;
        dat = UART_READ(UART1);
        RX_BUF_PUSH(dat);
    }
    if(RX_BUF_COUNT() >= TRSPX_mtu)
    {
        uint8_t i;
        for(i = 0; i < TRSPX_mtu; i++)
            TRSPX_Read_Data[i] = RX_BUF_POP();
        //printf("%d\n", TRSPX_mtu);
        TRSPX_send(TRSPX_mtu);
    }
    else if(tout)
    {
        uint8_t cnt, i;
        cnt = RX_BUF_COUNT();
        for(i = 0; i < cnt; i++)
            TRSPX_Read_Data[i] = RX_BUF_POP();
        //printf("%d\n", cnt);
        TRSPX_send(cnt);
    }
}
