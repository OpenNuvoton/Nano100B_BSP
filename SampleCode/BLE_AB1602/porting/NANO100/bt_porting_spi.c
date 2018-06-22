#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include "Nano100Series.h"
#include "ab_queue.h"
/******************************************************************************
 * typedef
 ******************************************************************************/
typedef struct
{
    uint16_t txlen;
    uint16_t rxlen;
    uint8_t *rxbuf;
    void (*tran_complete)(uint8_t *rxbuf, uint16_t rxlen);
    uint8_t *rxbuf_orig;
    uint16_t rxlen_orig;
    uint8_t is_rxbuf_self_create:1;
    uint8_t is_half_duplex : 1;
    uint8_t __attribute__ ((aligned(4)))
    txbuf[1];   // because nuvoton's PDMA can't allow non 4 aligned address
} spi_tran_entry;

#define DMA_Master_TX 1
#define DMA_Master_RX 2

/******************************************************************************
 * Variable
 ******************************************************************************/
static void *tran_queue = NULL;
static void (*bt_data_rdy)(void) = NULL;
static uint8_t spi_busy = false;
/******************************************************************************
 * Prototype
 ******************************************************************************/
static void free_spi_tran_entry(spi_tran_entry* entry);

/******************************************************************************
 * Private Functions
 ******************************************************************************/
static void free_spi_tran_entry(spi_tran_entry* entry)
{
    if(entry->is_rxbuf_self_create)
    {
        free(entry->rxbuf);
    }
    AB_queue_entry_free(entry);
}

static spi_tran_entry *alloc_spi_tran_entry(uint8_t *txbuf, uint16_t txlen,
        uint8_t *rxbuf, uint16_t rxlen,
        uint8_t is_half_duplex, void(*tran_complete)(uint8_t *, uint16_t))
{
    spi_tran_entry *entry;

    /*if user didn't provide rx buf, we need to create one*/
    if(is_half_duplex)
        entry = AB_queue_entry_alloc(sizeof(spi_tran_entry) + (txlen + rxlen) - 1);
    else
        entry = AB_queue_entry_alloc(sizeof(spi_tran_entry) + txlen - 1);

    if(!entry)
        return NULL;

    entry->is_half_duplex = is_half_duplex;

    if(is_half_duplex)
    {
        if(rxlen)
        {
            entry->rxbuf_orig = rxbuf;
            entry->rxlen_orig = rxlen;
            rxbuf = malloc(rxlen + txlen);
            entry->is_rxbuf_self_create = true;
        }
        else
        {
            entry->rxbuf_orig = 0;
            entry->rxlen_orig = 0;
            entry->is_rxbuf_self_create = false;
        }
    }
    else
    {
        if(rxlen && !rxbuf)
        {
            entry->rxbuf_orig = rxbuf;
            rxbuf = malloc(rxlen);
            entry->is_rxbuf_self_create = true;
        }
        else
            entry->is_rxbuf_self_create = false;
    }
    entry->tran_complete = tran_complete;
    if(is_half_duplex)
    {
        if(rxlen)
            entry->rxlen = rxlen + txlen;
        else
            entry->rxlen = 0;

        entry->txlen = txlen + rxlen;
        memcpy(entry->txbuf, txbuf, txlen+rxlen/*for test*/);
    }
    else
    {
        entry->rxlen = rxlen;
        entry->txlen = txlen;
        memcpy(entry->txbuf, txbuf, txlen);
    }
    entry->rxbuf = rxbuf;

    return entry;
}

void GPDEF_IRQHandler(void)
{
    /* To check if PE.1 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PE, BIT1))
    {
        GPIO_CLR_INT_FLAG(PE, BIT1);
        if(bt_data_rdy)
            bt_data_rdy();
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORTD, PORTE, PORTF interrupts */
        PD->ISRC = PD->ISRC;
        PE->ISRC = PE->ISRC;
        PF->ISRC = PF->ISRC;
        printf("Un-expected interrupts. \n");
    }
}

void SPITransactionStart(spi_tran_entry *entry)
{
    if(((uint32_t)entry->txbuf & 0x3) || ((uint32_t)entry->rxbuf & 0x3))
        printf("spi (%d, %d) = (%x, %x)\n", entry->txlen, entry->rxlen, entry->txbuf, entry->rxbuf);

    spi_busy = true;

    if(entry->rxlen)
    {
        SPI0->DMA |= SPI_DMA_PDMA_RST_Msk;
        while((SPI0->STATUS & (SPI_STATUS_RX_EMPTY_Msk | SPI_STATUS_RX_FULL_Msk)) != SPI_STATUS_RX_EMPTY_Msk)
            SPI0->FFCTL |= SPI_FFCTL_RX_CLR_Msk;

        PDMA_SetTransferAddr(DMA_Master_RX, (uint32_t)&SPI0->RX0, PDMA_SAR_FIX,
                             (uint32_t)entry->rxbuf, PDMA_DAR_INC);

        PDMA_SetTransferAddr(DMA_Master_TX, (uint32_t)entry->txbuf, PDMA_SAR_INC,
                             (uint32_t)&SPI0->TX0, PDMA_DAR_FIX);
        PDMA_SetTransferCnt(DMA_Master_TX, PDMA_WIDTH_8, entry->txlen);
        PDMA_SetTransferCnt(DMA_Master_RX, PDMA_WIDTH_8, entry->rxlen);

        SPI_TRIGGER_RX_PDMA(SPI0);
        SPI_TRIGGER_TX_PDMA(SPI0);

        PDMA_Trigger(DMA_Master_RX);
        PDMA_Trigger(DMA_Master_TX);
    }
    else //tx only
    {
        while((SPI0->STATUS & (SPI_STATUS_RX_EMPTY_Msk | SPI_STATUS_RX_FULL_Msk)) != SPI_STATUS_RX_EMPTY_Msk)
            SPI0->FFCTL |= SPI_FFCTL_RX_CLR_Msk;

        PDMA_SetTransferAddr(DMA_Master_RX, (uint32_t)&SPI0->RX0, PDMA_SAR_FIX,
                             (uint32_t)entry->txbuf, PDMA_DAR_INC);

        PDMA_SetTransferAddr(DMA_Master_TX, (uint32_t)entry->txbuf, PDMA_SAR_INC,
                             (uint32_t)&SPI0->TX0, PDMA_DAR_FIX);
        PDMA_SetTransferCnt(DMA_Master_TX, PDMA_WIDTH_8, entry->txlen);
        PDMA_SetTransferCnt(DMA_Master_RX, PDMA_WIDTH_8, entry->txlen);

        SPI_TRIGGER_RX_PDMA(SPI0);
        SPI_TRIGGER_TX_PDMA(SPI0);
        PDMA_Trigger(DMA_Master_RX);
        PDMA_Trigger(DMA_Master_TX);
    }
}

static void HandlerSPIComplete(spi_tran_entry *entry)
{
    spi_tran_entry *new_entry;

    spi_busy = false;

    // check for new transaction
    new_entry = AB_queue_pop(tran_queue);
    if(new_entry)
    {
        SPITransactionStart(new_entry);
    }

    // info save complete
    if(entry->tran_complete)
    {
        if(entry->is_half_duplex)
        {
            if(entry->rxbuf_orig)
            {
                memcpy(entry->rxbuf_orig, &entry->rxbuf[entry->rxlen - entry->rxlen_orig],
                       entry->rxlen_orig);
                entry->tran_complete(entry->rxbuf_orig, entry->rxlen_orig);
            }
            else
                entry->tran_complete(&entry->rxbuf[entry->rxlen - entry->rxlen_orig],
                                     entry->rxlen_orig);
        }
        else
            entry->tran_complete(entry->rxbuf, entry->rxlen);
    }
    // free save entry
    free_spi_tran_entry(entry);

}

void PDMA_IRQHandler (void)
{
    PDMA_T *pdma;
    spi_tran_entry *entry;
    pdma = (PDMA_T *)((uint32_t) PDMA1_BASE + (0x100 * (DMA_Master_TX-1)));
    entry = (spi_tran_entry*) (pdma->SAR - offsetof(spi_tran_entry, txbuf));

    uint32_t status = PDMA_GET_INT_STATUS();
    if(status & (1 << DMA_Master_TX))      /* TX CH */
    {
        if(PDMA_GET_CH_INT_STS(DMA_Master_TX) & PDMA_ISR_TD_IS_Msk)
            PDMA_CLR_CH_INT_FLAG(DMA_Master_TX, PDMA_ISR_TD_IS_Msk);
        if(PDMA_GET_CH_INT_STS(DMA_Master_TX) & PDMA_ISR_TABORT_IS_Msk) //error
        {
            printf("spi bus error\n");
            PDMA_CLR_CH_INT_FLAG(DMA_Master_TX, PDMA_ISR_TABORT_IS_Msk);
        }
    }

    if(status & (1 << DMA_Master_RX))      /* RX CH */
    {
        if(PDMA_GET_CH_INT_STS(DMA_Master_RX) & PDMA_ISR_TD_IS_Msk)
            PDMA_CLR_CH_INT_FLAG(DMA_Master_RX, PDMA_ISR_TD_IS_Msk);
        if(PDMA_GET_CH_INT_STS(DMA_Master_RX) & PDMA_ISR_TABORT_IS_Msk) //error
        {
            printf("spi bus error\n");
            PDMA_CLR_CH_INT_FLAG(DMA_Master_RX, PDMA_ISR_TABORT_IS_Msk);
        }

        //if(entry->rxlen) //TX/RX both
        {
            HandlerSPIComplete(entry);
        }
    }
}

bool AB_SPI_Open(int foo, void *deadbeef)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI0 as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2 MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, 2000000);
    /* Enable the automatic hardware slave select function. Select the SPI0_SS0 pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS0, SPI_SS0_ACTIVE_LOW);
    /* Enable FIFO mode */
    SPI_EnableFIFO(SPI0, 2, 2);
    /* Set PDMA Channel for SPI TX*/
    PDMA_Open(1 << DMA_Master_TX);
    PDMA_EnableInt(DMA_Master_TX, PDMA_IER_TD_IE_Msk);
    PDMA1->CSR = ((PDMA1->CSR & ~(PDMA_CSR_MODE_SEL_Msk)) |
                  (0x2 << PDMA_CSR_MODE_SEL_Pos));
    /* Set PDMA Channel for SPI RX*/
    PDMA_Open(1 << DMA_Master_RX);
    PDMA_EnableInt(DMA_Master_RX, PDMA_IER_TD_IE_Msk);
    PDMA2->CSR = ((PDMA2->CSR & ~(PDMA_CSR_MODE_SEL_Msk)) |
                  (0x1 << PDMA_CSR_MODE_SEL_Pos));
    /* Set PDMA SPI0 TX & RX Selection */
    PDMA_SetTransferMode(DMA_Master_TX, PDMA_SPI0_TX, 0, 0);
    PDMA_SetTransferMode(DMA_Master_RX, PDMA_SPI0_RX, 0, 0);
    NVIC_EnableIRQ(PDMA_IRQn);
    tran_queue = AB_queue_alloc();

    return true;
}

void AB_SPI_Close(int foo)
{
    SPI_Close(SPI0);
    PDMA_DisableInt(DMA_Master_TX, PDMA_IER_TD_IE_Msk);
    PDMA_DisableInt(DMA_Master_RX, PDMA_IER_TD_IE_Msk);
}

bool AB_SPI_WriteThenRead(int spi_num, uint8_t *txbuf, uint16_t txlen,
                          uint8_t *rxbuf, uint16_t rxlen, void(*tran_complete)(uint8_t *rxbuf,
                                  uint16_t rxlen))
{
    spi_tran_entry *entry;
    entry = alloc_spi_tran_entry(txbuf, txlen, rxbuf, rxlen, 1, tran_complete);
    if(!entry)
        return false;
    if(spi_busy || (SPI0->CTL & SPI_CTL_GO_BUSY_Msk))
    {
        //printf("busy\n");
        AB_queue_push(tran_queue, entry);
        return false;
    }
    else
    {
        spi_busy = true;
        SPITransactionStart(entry);
    }
    return true;
}

bool AB_SPI_WriteAndRead(int spi_num, uint8_t *txbuf, uint8_t *rxbuf,
                         uint16_t rxlen, void(*tran_complete)(uint8_t *rxbuf, uint16_t rxlen))
{
    spi_tran_entry *entry;
    entry = alloc_spi_tran_entry(txbuf, rxlen, rxbuf, rxlen, 0, tran_complete);
    if(!entry)
        return false;
    if(spi_busy || (SPI0->CTL & SPI_CTL_GO_BUSY_Msk))
    {
        //printf("busy\n");
        AB_queue_push(tran_queue, entry);
        return false;
    }
    else
    {
        SPITransactionStart(entry);
    }
    return true;
}

/*--------------------------------------------------------------------------------------------------------------*/
/* data_ready and reset pin                                                                                     */
/*--------------------------------------------------------------------------------------------------------------*/

void Drv_1602Reset(bool is_reset)
{
    PE0 = is_reset ? 0 : 1;
}

bool Drv_1602DataReady(void)
{
    return PE1;
}

void Drv_1602DataReadyIntEn(bool is_enable)
{
    if(is_enable)
    {
        GPIO_EnableEINT0(PE, 1, GPIO_INT_RISING);
        NVIC_EnableIRQ(GPDEF_IRQn);
    }
    else
    {
        NVIC_DisableIRQ(GPDEF_IRQn);
    }
}

void Drv_1602DataReadyIntReg(bool foo, void (*int_cb)(void))
{
    bt_data_rdy = int_cb;
    if(Drv_1602DataReady())
    {
        if(bt_data_rdy)
            bt_data_rdy();
    }
    Drv_1602DataReadyIntEn(true);
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/

void Airoha1602_PinInit()
{
    /* Select HCLK as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0_S_HCLK, MODULE_NoMsk);
    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);
    /* Enable PDMA peripheral clock */
    CLK_EnableModuleClock(DMA_MODULE);

    /* Setup SPI0 multi-function pins */
    SYS->PC_L_MFP &= ~(SYS_PC_L_MFP_PC0_MFP_Msk | SYS_PC_L_MFP_PC1_MFP_Msk |
                       SYS_PC_L_MFP_PC2_MFP_Msk | SYS_PC_L_MFP_PC3_MFP_Msk);
    SYS->PC_L_MFP |= (SYS_PC_L_MFP_PC0_MFP_SPI0_SS0 | SYS_PC_L_MFP_PC1_MFP_SPI0_SCLK |
                      SYS_PC_L_MFP_PC2_MFP_SPI0_MISO0 | SYS_PC_L_MFP_PC3_MFP_SPI0_MOSI0);

    /* 1602 reset N */
    SYS->PE_L_MFP = (SYS->PE_L_MFP & (~SYS_PE_L_MFP_PE0_MFP_Msk)) | SYS_PE_L_MFP_PE0_MFP_GPE0;
    GPIO_SetMode(PE, BIT0, GPIO_PMD_OUTPUT);
    PE0 = 0; //reset state

    /* 1602 data ready */
    /* Set PE multi-function pin for GPDEF_IRQ(PE.1) */
    SYS->PE_L_MFP = (SYS->PE_L_MFP & (~SYS_PE_L_MFP_PE1_MFP_Msk)) | SYS_PE_L_MFP_PE1_MFP_GPE1;
    GPIO_SetMode(PE, BIT1, GPIO_PMD_INPUT);
}

