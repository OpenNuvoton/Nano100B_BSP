/**************************************************************************//**
 * @file     main.c
 * @version  V2.10
 * $Date: 15/06/30 11:25a $
 * @brief    Calculate the CRC-CCITT checksum value by CRC DMA mode.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Nano100Series.h"

uint8_t volatile g_u8IsTargetAbortINTFlag = 0, g_u8IsBlockTransferDoneINTFlag = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_nano100series.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = CRC_GET_INT_FLAG();
    if (status & DMA_CRC_DMAISR_BLKD_IF_Msk) {
        /* Clear Block Transfer Done Interrupt Flag */
        CRC_CLR_INT_FLAG(DMA_CRC_DMAISR_BLKD_IF_Msk);

        g_u8IsBlockTransferDoneINTFlag++;
    } else if (status & DMA_CRC_DMAISR_TABORT_IF_Msk) {
        /* Clear Target Abort Interrupt Flag */
        CRC_CLR_INT_FLAG(DMA_CRC_DMAISR_TABORT_IF_Msk);

        g_u8IsTargetAbortINTFlag++;
    } else {
        printf("Un-expected interrupts. \n");
    }
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

    CLK->AHBCLK |= CLK_AHBCLK_DMA_EN_Msk;
    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
//    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
//    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX|SYS_PA_H_MFP_PA14_MFP_UART0_RX);
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB1_MFP_UART0_TX|SYS_PB_L_MFP_PB0_MFP_UART0_RX);

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
/*  CRC-CCITT Polynomial Mode Test                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void CRC_CCITTPolyModeTest(uint32_t u32SrcAddr, uint32_t u32TransByteCount)
{
    uint32_t u32TargetChecksum = 0x29B1, u32CalChecksum = 0;

    printf("# Calculate string \"123456789\" CRC-CCITT checksum value by CRC DMA mode. \n");
    printf("    - Seed value is 0xFFFF          \n");
    printf("    - Checksum Complement disable   \n");
    printf("    - Checksum Reverse disable      \n");
    printf("    - Write Data Complement disable \n");
    printf("    - Write Data Reverse disable    \n");
    printf("    - Checksum should be 0x%X       \n", u32TargetChecksum);
    printf("... \n\n");

    g_u8IsTargetAbortINTFlag = g_u8IsBlockTransferDoneINTFlag = 0;

    /* Enable CRC channel clock */
    /* Configure CRC Operation Settings for CRC DMA mode */
    CRC_Open(CRC_CCITT, 0, 0xFFFF, 0);

    /* Enable DMA Target Abort and Block Transfer Done Interrupt */
    CRC_ENABLE_INT(DMA_CRC_DMAIER_TABORT_IE_Msk|DMA_CRC_DMAIER_BLKD_IE_Msk);

    /* Enable PDMA and CRC NVIC */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Trigger CRC DMA transfer */
    CRC_StartDMATransfer(u32SrcAddr, u32TransByteCount);

    /* Wait CRC Interrupt Flag occurred */
    while (1) {
        if (g_u8IsTargetAbortINTFlag == 1) {
            printf("DMA Target Abort Interrupt occurred. \n");
            break;
        }
        if (g_u8IsBlockTransferDoneINTFlag == 1) {
            break;
        }
    }

    /* Disable PDMA and CRC NVIC */
    NVIC_DisableIRQ(PDMA_IRQn);

    /* Get CRC Checksum value */
    u32CalChecksum = CRC_GetChecksum();
    if (g_u8IsBlockTransferDoneINTFlag == 1) {
        printf("CRC checksum is 0x%X ... %s.\n", u32CalChecksum, (u32CalChecksum==u32TargetChecksum)?"PASS":"FAIL");
    }

    printf("\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  CRC-8 Polynomial Mode Test                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CRC_CRC8PolyModeTest(uint32_t u32SrcAddr, uint32_t u32TransByteCount)
{
    uint32_t i = 0;
    uint32_t u32TargetChecksum = 0x58, u32CalChecksum = 0;
    uint8_t *p8SrcAddr;

    printf("# Calculate string \"123456789\" CRC-8 checksum value by CRC CPU mode. \n");
    printf("    - Seed value is 0x5A            \n");
    printf("    - CPU Write Length is 8-bit     \n");
    printf("    - Checksum Complement disable   \n");
    printf("    - Checksum Reverse disable      \n");
    printf("    - Write Data Complement disable \n");
    printf("    - Write Data Reverse disable    \n");
    printf("    - Checksum should be 0x%X       \n", u32TargetChecksum);
    printf("... \n\n");

    p8SrcAddr = (uint8_t *)u32SrcAddr;

    /* Enable CRC channel clock */
    /* Configure CRC Operation Settings for CRC DMA mode */
    CRC_Open(CRC_8, 0, 0x5A, CRC_CPU_WDATA_8);

    for (i=0; i<u32TransByteCount; i++) {
        CRC_WRITE_DATA((p8SrcAddr[i]&0xFF));
    }

    /* Get CRC Checksum value */
    u32CalChecksum = CRC_GetChecksum();
    printf("CRC checksum is 0x%X ... %s.\n", u32CalChecksum, (u32CalChecksum==u32TargetChecksum)?"PASS":"FAIL");

    printf("\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    const uint8_t acCRCSrcPattern[] = "123456789";

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz \n", SystemCoreClock);

    printf("+--------------------------------------+ \n");
    printf("|    NANO100 CRC Driver Sample Code    | \n");
    printf("+--------------------------------------+ \n");

    printf(" CRC-CCITT Polynomial mode test \n");
    CRC_CCITTPolyModeTest((uint32_t )acCRCSrcPattern, strlen((char *)acCRCSrcPattern));

    printf(" CRC-8 Polynomial mode test \n");
    CRC_CRC8PolyModeTest((uint32_t )acCRCSrcPattern, strlen((char *)acCRCSrcPattern));

    printf("\nExit CRC Sample Code. \n");
    while(1);
}


