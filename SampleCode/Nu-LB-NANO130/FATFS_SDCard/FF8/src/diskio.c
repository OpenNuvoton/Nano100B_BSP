/*-----------------------------------------------------------------------*/
/* Low level disk control module for Win32              (C)ChaN, 2007    */
/*-----------------------------------------------------------------------*/


#include <stdio.h>
#include <string.h>

#include "diskio.h"
#include "SDCard.h"

#define STORAGE_BUFFER_SIZE 1024        /* Data transfer buffer size in 512 bytes alignment */
uint32_t Storage_Block[STORAGE_BUFFER_SIZE / 4];
#define STORAGE_DATA_BUF   ((uint32_t)&Storage_Block[0])
extern void SpiRead(uint32_t addr, uint32_t size, uint8_t *buffer);
extern void SpiWrite(uint32_t addr, uint32_t size,  uint8_t *buffer);
//extern int8_t Is_Initialized ;

void RoughDelay(uint32_t t)
{
    volatile int32_t delay;

    delay = t;

    while(delay-- >= 0);
}

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE drv        /* Physical drive nmuber */
)
{
    DSTATUS sta=RES_OK;
    uint32_t timeout = 0;

    //card detect = GPB14
    //GPIOB->PMD = (GPIOB->PMD & ~GPIO_PMD_PMD14_MASK) | GPIO_PMD_PMD14_INPUT;
    GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);

    printf("Check SD Card insert ?\n");
    while(PB14 == 1)
    {
        if(timeout++ > 0x200)
            break;
    }

    if(PB14 != 1)
    {
        printf("Card Detected!!\n");

        //power pin(GPE6)
        //GPIOE->PMD = (GPIOE->PMD & ~GPIO_PMD_PMD6_MASK) | GPIO_PMD_PMD6_OUTPUT;
        GPIO_SetMode(PE, BIT6, GPIO_PMD_OUTPUT);
        PE6 = 0;

        RoughDelay(100000);
        if(SDCARD_Open() == SD_SUCCESS)
        {
            sta =   RES_OK;
            printf("SDCard Open success\n");
        }
        else
        {
            sta = STA_NOINIT;
            printf("SDCard Open failed\n");
        }
    }
    else
        printf("Can't detect card !!\n");

    return sta;

}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE drv        /* Physical drive nmuber (0) */
)
{
    DSTATUS sta1=STA_OK;
    if (drv)
        sta1 =   STA_NOINIT;
    return sta1;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
#define DRVSDCARD_BLOCK_SIZE 512
//extern int8_t SDtype;

DRESULT disk_read (
    BYTE drv,           /* Physical drive nmuber (0) */
    BYTE *buff,         /* Pointer to the data buffer to store read data */
    DWORD sector,       /* Start sector number (LBA) */
    BYTE count          /* Sector count (1..255) */
)
{
    DRESULT res;
    uint32_t size;

    if (drv)
    {
        res = (DRESULT)STA_NOINIT;
        return res;
    }

    if(count==0||count>2)
    {
        res = (DRESULT)STA_NOINIT;
        return res;
    }

    size = count*512;
    SpiRead(sector, size, buff);

    res = RES_OK;   /* Clear STA_NOINIT */;

    return res;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT disk_write (
    BYTE drv,           /* Physical drive nmuber (0) */
    const BYTE *buff,   /* Pointer to the data to be written */
    DWORD sector,       /* Start sector number (LBA) */
    BYTE count          /* Sector count (1..255) */
)
{

    DRESULT  res;
    uint32_t size;

    if (drv)
    {
        res = (DRESULT)STA_NOINIT;
        return res;
    }

    if(count==0||count>2)
    {
        res = (DRESULT)  STA_NOINIT;
        return res;
    }

    size=count*512;

    SpiWrite(sector, size,(uint8_t *)buff);

    res = RES_OK;

    return res;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE drv,       /* Physical drive nmuber (0) */
    BYTE ctrl,      /* Control code */
    void *buff      /* Buffer to send/receive data block */
)
{
    DRESULT res;

//  BYTE n;

    if (drv) return RES_PARERR;

    switch (ctrl)
    {
    case CTRL_SYNC :        /* Make sure that no pending write process */
        res = RES_OK;
        break;

    case GET_SECTOR_COUNT : /* Get number of sectors on the disk (DWORD) */
        SDCARD_GetCardSize(buff);
        res = RES_OK;
        break;

    case GET_SECTOR_SIZE :  /* Get R/W sector size (WORD) */
        *(DWORD*)buff = 512;    //512;
        res = RES_OK;
        break;

    case GET_BLOCK_SIZE :   /* Get erase block size in unit of sector (DWORD) */
        *(DWORD*)buff = 1;
        res = RES_OK;
        break;


    default:
        res = RES_PARERR;
    }


    res = RES_OK;


    return res;
}




