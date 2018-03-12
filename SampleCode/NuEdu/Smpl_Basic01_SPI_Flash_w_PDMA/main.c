/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/09/18 11:49a $
 * @brief    Demonstrate how to access SPI Flash through a SPI interface with combing PDMA function
 *           and print the test results on both 7-Segments and PC via NUCOM1 port of the NuEdu-Basic01 board.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01.h"

#define TEST_NUMBER         10 /* page numbers */
#define CH1                     1
#define CH2                     2

unsigned char   SrcArray[256];
unsigned char   DestArray[256];


/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXT_EN_Msk;

    /* Waiting for 12MHz clock ready */
    while((CLK->CLKSTATUS & CLK_CLKSTATUS_HXT_STB_Msk) == 0) {};

    /* Switch PLL clock source to XTAL */
    CLK->PLLCTL &= CLK_PLLCTL_PLL_SRC_HXT;

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL &= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency 84 MHz */
    CLK->PLLCTL |= 0x0218;

    /* Waiting for clock ready */
    while((CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk) == 0) {};

    /* Set HCLK_N = 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLK_N_Msk) | 0x01;

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();

}


/*---------------------------------------------------------------------------------------------------------*/
/* Init UART1                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{
    /* Set PC.10 and PC.11 multi-function pins for UART1_RXD, UART1_TXD */
    SYS->PC_H_MFP |= (SYS_PC_H_MFP_PC11_MFP_UART1_TX | SYS_PC_H_MFP_PC10_MFP_UART1_RX);

    /* Enable UART1 IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART1_EN_Msk;

    /* Set UART1 IP clock source */
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* Configure UART1 and set UART1 Baudrate */
    UART1->BAUD = UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(FREQ_12MHZ, 115200);
    UART1->TLCTL = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main()
{
    unsigned int u32ByteCount;
    unsigned int u32PageNumber;
    unsigned int u32ProgramFlashAddress = 0;
    unsigned int u32VerifyFlashAddress = 0;
    unsigned int MidDid;

    PDMA_T *PDMA_CH1, *PDMA_CH2;

    // PDMA Channel 1/2 control registers
    PDMA_CH1 = (PDMA_T *)((uint32_t) PDMA1_BASE + (0x100 * (CH1-1)));
    PDMA_CH2 = (PDMA_T *)((uint32_t) PDMA1_BASE + (0x100 * (CH2-1)));

    /* Initial system */
    SYS_Init();

    /* Initial UART1 to 115200-8n1 for print message */
    UART1_Init();

    printf("Hello World.\n");
    printf("PLL Clock = %d Hz\n", CLK_GetPLLClockFreq());
    printf("Core Clock = %d Hz\n\n", CLK_GetHCLKFreq());
    printf("+-------------------------------------------------------+\n");
    printf("|    Nano100 Series SPI_Flash Sample Code with PDMA     |\n");
    printf("+-------------------------------------------------------+\n");

    /* Open 7-Seg */
    Open_Seven_Segment();

    /* Open SPI for Serial Flash */
    Open_SPI_Flash();

    /* Initial PDMA Channels */
    Init_PDMA_CH1_for_SPI0_TX((uint32_t)SrcArray);
    Init_PDMA_CH2_for_SPI0_RX((uint32_t)DestArray);

    /* Enable PDMA IRQ */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Read MID & DID */
    MidDid = SpiFlash_w_PDMA_ReadMidDid();
    printf("\nMID and DID = %x", MidDid);

    /* Erase SPI Flash */
    SpiFlash_w_PDMA_ChipErase();
    printf("\nFlash Erasing... ");

    /* Wait ready */
    SpiFlash_w_PDMA_WaitReady();
    printf("Done!");

    /* Fill the Source Data and clear Destination Data Buffer */
    for(u32ByteCount=0; u32ByteCount<256; u32ByteCount++)
    {
        SrcArray[u32ByteCount] = u32ByteCount;
        DestArray[u32ByteCount] = 0;
    }

    u32ProgramFlashAddress = 0;
    u32VerifyFlashAddress = 0;
    for(u32PageNumber=0; u32PageNumber<TEST_NUMBER; u32PageNumber++)
    {
        printf("\n\nTest Page Number = %d", u32PageNumber);
        Show_Seven_Segment(u32PageNumber,1);
        CLK_SysTickDelay(200000);

        /*=== Program SPI Flash ===*/
        printf("\n Flash Programming... ");

        /* Trigger PDMA specified Channel */
        PDMA_CH1->CSR |= (PDMA_CSR_TRIG_EN_Msk | PDMA_CSR_PDMACEN_Msk);

        /* Page Program */
        SpiFlash_w_PDMA_PageProgram(u32ProgramFlashAddress, 256);
        SpiFlash_w_PDMA_WaitReady();
        u32ProgramFlashAddress += 0x100;
        printf("Done!");

        /*=== Read Back and Compare Data ===*/
        printf("\n Flash Verifying... ");

        /* Trigger PDMA specified Channel */
        PDMA_CH2->CSR |= (PDMA_CSR_TRIG_EN_Msk | PDMA_CSR_PDMACEN_Msk);

        /* Page Read */
        SpiFlash_w_PDMA_ReadData(u32VerifyFlashAddress, 256);
        u32VerifyFlashAddress += 0x100;

        for(u32ByteCount=0; u32ByteCount<256; u32ByteCount++)
        {
            if(DestArray[u32ByteCount]!=u32ByteCount)
            {
                /* Error */
                printf("\n\nSPI Flash R/W Fail!");
                while(1);
            }
        }

        /* Clear Destination Data Buffer */
        for(u32ByteCount=0; u32ByteCount<256; u32ByteCount++)
            DestArray[u32ByteCount] = 0;
        printf("Done!");
    }

    printf("\n\nSPI Flash with PDMA Test Ok!");
    printf("\n\n");

    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
