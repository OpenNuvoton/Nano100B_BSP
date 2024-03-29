/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/06/12 3:17p $
 * @brief    NANO100 series FMC driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

//* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include "Nano100Series.h"


/** @addtogroup NANO100_Device_Driver NANO100 Device Driver
  @{
*/

/** @addtogroup NANO100_FMC_Driver FMC Driver
  @{
*/


/** @addtogroup NANO100_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

int32_t  g_FMC_i32ErrCode;

/**
  * @brief Disable FMC ISP function.
  * @return None
  */
void FMC_Close(void)
{
    FMC->ISPCON &= ~FMC_ISPCON_ISPEN_Msk;
}


/**
  * @brief Execute ISP command to erase a flash page. The page size is 512 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 512-byte aligned address.
  * @return ISP page erase success or not.
  * @retval   0   Success
  * @retval  -1   Erase failed or time-out
  * @note         Global error code g_FMC_i32ErrCode
  *               -1  time-out error
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
    int32_t  tout = FMC_TIMEOUT_ERASE;

    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if ((tout <= 0) || (FMC->ISPCON & FMC_ISPCON_ISPFF_Msk))
    {
        g_FMC_i32ErrCode = -1;
		FMC->ISPCON |= FMC_ISPCON_ISPFF_Msk;
        return -1;
    }
    return 0;
}


/**
  * @brief Get the current boot source.
  * @return The current boot source.
  * @retval   0  Is boot from APROM.
  * @retval   1  Is boot from LDROM.
  */
int32_t FMC_GetBootSource (void)
{
    if (FMC->ISPCON & FMC_ISPCON_BS_Msk)
        return 1;
    else
        return 0;
}


/**
  * @brief Enable FMC ISP function
  * @return None
  */
void FMC_Open(void)
{
    FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;
}


/**
  * @brief Execute ISP command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  */
uint32_t FMC_Read(uint32_t u32Addr)
{
    int32_t  tout = FMC_TIMEOUT_ERASE;

    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADR = u32Addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    return FMC->ISPDAT;
}


/**
  * @brief    Read company ID.
  * @return   The company ID.
  *           Return 0xFFFFFFFF if read failed.
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadCID(void)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_CID;
    FMC->ISPADR = 0x0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    return FMC->ISPDAT;
}


/**
  * @brief    Read product ID.
  * @return   The product ID.
  *           Return 0xFFFFFFFF if read failed.
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadPID(void)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_PID;
    FMC->ISPADR = 0x04;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    return FMC->ISPDAT;
}


/**
  * @brief    This function reads one of the four UCID.
  * @param[in]   u32Index Index of the UCID to read. u32Index must be 0, 1, 2, or 3.
  * @return   The UCID.
  *           Return 0xFFFFFFFF if read failed.
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADR = (0x04 * u32Index) + 0x10;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    return FMC->ISPDAT;
}


/**
  * @brief    This function reads one of the three UID.
  * @param[in]  u32Index Index of the UID to read. u32Index must be 0, 1, or 2.
  * @return   The UID.
  *           Return 0xFFFFFFFF if read failed.
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadUID(uint32_t u32Index)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADR = 0x04 * u32Index;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    return FMC->ISPDAT;
}


/**
  * @brief    Get the base address of Data Flash if enabled.
  * @return   Base address of Data Flash
  */
uint32_t FMC_ReadDataFlashBaseAddr(void)
{
    return FMC->DFBADR;
}


/**
  * @brief    This function will force re-map assigned flash page to CPU address 0x0.
  * @param[in]    u32PageAddr   Address of the page to be mapped to CPU address 0x0.
  * @return   None
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  time-out
  */
void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    int32_t  tout = FMC_TIMEOUT_WRITE;

    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
        g_FMC_i32ErrCode = -1;
}


/**
  * @brief    Obtain the current vector page address setting.
  * @return   The vector page address.
  */
uint32_t FMC_GetVectorPageAddr(void)
{
    return (FMC->ISPSTA & 0x0FFFFF00ul);
}


/**
  * @brief Execute ISP command to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return None
  */
int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t  tout = FMC_TIMEOUT_WRITE;

    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if ((tout <= 0) || (FMC->ISPSTA & FMC_ISPSTA_ISPFF_Msk))
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}


/**
  * @brief Execute ISP command to read User Configuration.
  * @param[out]  u32Config A two-word array.
  *              u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
  * @param[in] u32Count Avaliable word count in u32Config.
  * @return Success or not.
  * @retval   0  Success.
  * @retval   -1  Invalid parameter.
  */
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count)
{
    u32Config[0] = FMC_Read(FMC_CONFIG_BASE);
    if (u32Count < 2)
        return -1;
    u32Config[1] = FMC_Read(FMC_CONFIG_BASE+4);
    return 0;
}


/**
  * @brief Execute ISP command to write User Configuration.
  * @param[in] u32Config A two-word array.
  *            u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
  * @param[in] u32Count Available word count in u32Config.
  * @return Success or not.
  * @retval   0  Success.
  * @retval   -1  Invalid parameter.
  */
int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count)
{
    FMC_ENABLE_CFG_UPDATE();
    FMC_Erase(FMC_CONFIG_BASE);
    FMC_Write(FMC_CONFIG_BASE, u32Config[0]);
    FMC_Write(FMC_CONFIG_BASE+4, u32Config[1]);
    FMC_DISABLE_CFG_UPDATE();
    return 0;
}


/*@}*/ /* end of group NANO100_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO100_FMC_Driver */

/*@}*/ /* end of group NANO100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


