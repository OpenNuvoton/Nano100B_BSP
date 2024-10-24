/**************************************************************************//**
 * @file     ap_main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/06/16 4:55p $
 * @brief    Demonstrate a simple IAP function to show three independent programs
 *           including main routine, independent interrupt handler and updating
 *           or switching to another program with IAP function.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01.h"
#include "map.h"


static int  load_image_to_flash(uint32_t image_base, uint32_t image_limit, uint32_t flash_addr, uint32_t max_size);
int IsDebugFifoEmpty(void);


#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
volatile uint32_t const VersionNumber __attribute__ ((section(".ARM.__at_0x1000"))) = 0x00001;
#elif
volatile uint32_t const VersionNumber __attribute__ ((at(0x1000+USER_AP0_ENTRY)))=0x00001;
#endif

void TMR0_IRQHandler(void)
{
    static uint32_t sec = 1;
    printf("%d sec\n", sec++);
    LED_on(sec);

    // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER0);

}
#ifdef __ARMCC_VERSION
static __INLINE void __set_SP(uint32_t _sp)
{
    __set_MSP(_sp);
}
#endif

static __INLINE void BranchTo(uint32_t u32Address)
{
    FUNC_PTR        *func;
    FMC_SetVectorPageAddr(u32Address);
    func =  (FUNC_PTR *)(*(uint32_t *)(u32Address+4));
    printf("branch to address 0x%x\n", (int)func);
    printf("\n\nChange VECMAP and branch to user application...\n");
    while (!IsDebugFifoEmpty());
    __set_SP(*(uint32_t *)u32Address);
    func();
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Select IP clock source */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HIRC, 0);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PC.10 and PC.11 multi-function pins for UART1 RXD, UART1 TXD */
    SYS->PC_H_MFP = (SYS_PC_H_MFP_PC11_MFP_UART1_TX | SYS_PC_H_MFP_PC10_MFP_UART1_RX);
    /* Lock protected registers */
    SYS_LockReg();
}

void Timer0_Init(void)
{
    // Give a dummy target frequency here. Will over write capture resolution with macro
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);

    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);


    // Start Timer 0
    TIMER_Start(TIMER0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    int         cbs, ch;
    uint32_t    au32Config[2];

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART1, 115200);
    initial_led();
    Timer0_Init();

    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    printf("\n\n");
    printf("+--------------------------------------------------+\n");
    printf("|         User program running on APROM:0x%x       |\n",*(uint32_t*)0x4);
    printf("+--------------------------------------------------+\n");

    /*-------------------------------------------------------------
     *  Check Boot loader image
     *------------------------------------------------------------*/
    if(FMC_Read(FMC_LDROM_BASE)==0xFFFFFFFF)
    {
        printf("Don't find boot loader\n");
        printf("Writing fmc_ld_boot.bin image to LDROM...\n");
        FMC_ENABLE_LD_UPDATE();
        if (load_image_to_flash((uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit,
                                FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
        {
            printf("Load image to LDROM failed!\n");
            return -1;
        }
        FMC_DISABLE_LD_UPDATE();
        while (!IsDebugFifoEmpty());
        NVIC_SystemReset();
    }

    /*-------------------------------------------------------------
     *  Modify CBS to 00b (boot from APROM)
     *------------------------------------------------------------*/
    FMC_ReadConfig(au32Config, 2);
    cbs = (au32Config[0] >> 6) & 0x3;
    printf("Config0 = 0x%x, Config1 = 0x%x, CBS=%d\n\n", au32Config[0], au32Config[1], cbs);

    if (cbs)
    {
        printf("\n\nChange boot setting to [Boot from APROM].\n");
        FMC_ENABLE_CFG_UPDATE();
        au32Config[0] &= ~0xc0;          /* set CBS to 00b */
        au32Config[0] |= 0x1;           /* disable Data Flash */
        FMC_WriteConfig(au32Config, 2);
    }
    while(1)
    {
        printf("\n\nDo you want to update AP1?(Yes/No)\n");
        while (1)
        {
            ch = getchar();
            if ((ch == 'Y') || (ch == 'y'))
            {
                printf("Writing fmc_isp.bin image to APROM address 0x%x...\n", USER_AP1_ENTRY);
                FMC_ENABLE_AP_UPDATE();
                if (load_image_to_flash((uint32_t)&loaderImage2Base, (uint32_t)&loaderImage2Limit,
                                        USER_AP1_ENTRY, USER_AP1_MAX_SIZE) != 0)
                {
                    printf("Load image to APROM failed!\n");
                    return -1;
                }
                FMC_DISABLE_AP_UPDATE();
                break;
            }
            if ((ch == 'N') || (ch == 'n')) break;
        }

        printf("\n\nDo you want to branch AP1?(Yes/No)\n");
        while (1)
        {
            ch = getchar();
            if ((ch == 'Y') || (ch == 'y')) BranchTo(USER_AP1_ENTRY);
            if ((ch == 'N') || (ch == 'n')) break;
        }
    }
}


static int  load_image_to_flash(uint32_t image_base, uint32_t image_limit, uint32_t flash_addr, uint32_t max_size)
{
    uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = image_limit - image_base;
    if (u32ImageSize == 0)
    {
        printf("  ERROR: Loader Image is 0 bytes!\n");
        return -1;
    }

    if (u32ImageSize > max_size)
    {
        printf("  ERROR: Loader Image is larger than %d KBytes!\n", max_size/1024);
        return -1;
    }

    printf("Program image to flash address 0x%x...", flash_addr);
    pu32Loader = (uint32_t *)image_base;
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        if (FMC_Erase(flash_addr + i))
        {
            printf("Erase failed on 0x%x\n", flash_addr + i);
            return -1;
        }

        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            FMC_Write(flash_addr + i + j, pu32Loader[(i + j) / 4]);
        }
    }
    printf("OK.\n");

    printf("Verify ...");

    /* Verify loader */
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            u32Data = FMC_Read(flash_addr + i + j);
            if (u32Data != pu32Loader[(i+j)/4])
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", flash_addr + i + j, u32Data, pu32Loader[(i+j)/4]);
                return -1;
            }

            if (i + j >= u32ImageSize)
                break;
        }
    }
    printf("OK.\n");
    return 0;
}


