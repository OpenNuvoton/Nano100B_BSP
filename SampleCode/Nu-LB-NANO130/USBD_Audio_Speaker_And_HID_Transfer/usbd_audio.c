/**************************************************************************//**
 * @file     usbd_audio.c
 * @brief    December series USBD driver Sample file
 * @version  1.0.0
 * @date     23, December, 2013
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "Nano100Series.h"
#include "usbd_audio.h"

#if 1
#define DBG_PRINTF      printf
#else
#define DBG_PRINTF(...)
#endif

/*--------------------------------------------------------------------------*/
static volatile uint8_t bPlayVolumeLAdjust = FALSE;
static volatile uint8_t bPlayVolumeRAdjust = FALSE;
static volatile uint8_t bIsI2CIdle = TRUE;


/*--------------------------------------------------------------------------*/
/* Global variables for Audio class */
uint32_t g_usbd_UsbAudioState = 0;

uint8_t  g_usbd_RecMute = 0x01;
int16_t  g_usbd_RecVolume = 0x1000;
int16_t  g_usbd_RecMaxVolume = 0x7FFF;
int16_t  g_usbd_RecMinVolume = 0x8000;
int16_t  g_usbd_RecResVolume = 0x400;

uint8_t  g_usbd_PlayMute = 0x01;
int16_t  g_usbd_PlayVolumeL = 0x1000;
int16_t  g_usbd_PlayVolumeR = 0x1000;
int16_t  g_usbd_PlayMaxVolume = 0x7FFF;
int16_t  g_usbd_PlayMinVolume = 0x8000;
int16_t  g_usbd_PlayResVolume = 0x400;

/*--------------------------------------------------------------------------*/


/**
 * @brief       USBD Interrupt Service Routine
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is the USBD ISR
 */
void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if(u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
        }
        if(u32State & USBD_STATE_SUSPEND)
        {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }
        if(u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
        }
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_USB)
    {
        // USB event
        if(u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if(u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);

            // control IN
            USBD_CtrlIn();
        }

        if(u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
            USBD_CtrlOut();
        }

        if(u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);

            // Isochronous IN
            EP2_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);

            // Isochronous OUT
            EP3_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);

            // Interrupt IN
            EP4_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);

            // Interrupt OUT
            EP5_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if(u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }
    }
}

/**
 * @brief       EP2 Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process Isochronous In event for recording.
 */
/* Record */
void EP2_Handler(void)
{
    /* ISO IN transfer ACK */
    if (g_usbd_UsbAudioState == UAC_START_AUDIO_RECORD)
    {
        UAC_DeviceEnable(UAC_MICROPHONE);
        g_usbd_UsbAudioState = UAC_PROCESSING_AUDIO_RECORD;
    }
    else if (g_usbd_UsbAudioState == UAC_PROCESSING_AUDIO_RECORD)
        g_usbd_UsbAudioState = UAC_BUSY_AUDIO_RECORD;

    if (g_usbd_UsbAudioState == UAC_BUSY_AUDIO_RECORD)
        UAC_SendRecData();
    else
        USBD_SET_PAYLOAD_LEN(EP2, 0);
}

/**
 * @brief       EP3 Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process Isochronous Out event (ISO OUT transfer ACK) for play audio.
 */
/* Play */
void EP3_Handler(void)
{
    /* ISO OUT transfer ACK */
    UAC_GetPlayData((int16_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3)), (int16_t)USBD_GET_PAYLOAD_LEN(EP3));

    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
}

void EP4_Handler(void)  /* Interrupt IN handler */
{
    HID_SetInReport();
}

void EP5_Handler(void)  /* Interrupt OUT handler */
{
    uint8_t *ptr;
    /* Interrupt OUT */
    ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP5));
    HID_GetOutReport(ptr, USBD_GET_PAYLOAD_LEN(EP5));
    USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
}

/*--------------------------------------------------------------------------*/
/**
 * @brief       UAC Class Initial
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure endpoints for UAC class
 */
void UAC_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer for setup packet -> [0 ~ 0x7] */
    USBD->BUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Isochronous IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | ISO_IN_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /*****************************************************/
    /* EP3 ==> Isochronous OUT endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | ISO_OUT_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /*****************************************************/
    /* EP4 ==> Interrupt IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer range for EP4 */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    /* EP5 ==> Interrupt OUT endpoint, address 4 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_OUT | INT_OUT_EP_NUM);
    /* Buffer range for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);
    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
}


/**
 * @brief       UAC class request
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process UAC class requests
 */
void UAC_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if (buf[0] & 0x80)   /* request data transfer direction */
    {
        // Device to host
        switch (buf[1])
        {
        case UAC_GET_CUR:
        {
            switch (buf[3])
            {
            case MUTE_CONTROL:
            {
                if (REC_FEATURE_UNITID == buf[5])
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMute;
                else if (PLAY_FEATURE_UNITID == buf[5])
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMute;

                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 1);
                break;
            }
            case VOLUME_CONTROL:
            {
                if (REC_FEATURE_UNITID == buf[5])
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecVolume;
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecVolume >> 8;
                }
                else if (PLAY_FEATURE_UNITID == buf[5])
                {
                    if(buf[2] == 1)
                    {
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayVolumeL;
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayVolumeL >> 8;
                    }
                    else
                    {
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayVolumeR;
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayVolumeR >> 8;
                    }
                }
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 2);
                break;
            }
            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
            }
            }
            // Trigger next Control Out DATA1 Transaction.
            /* Status stage */
            USBD_PrepareCtrlOut(0,0);
            break;
        }

        case UAC_GET_MIN:
        {
            switch (buf[3])
            {
            case VOLUME_CONTROL:
            {
                if (REC_FEATURE_UNITID == buf[5])
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMinVolume;
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecMinVolume >> 8;
                }
                else if (PLAY_FEATURE_UNITID == buf[5])
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMinVolume;
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayMinVolume >> 8;
                }
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 2);
                break;
            }
            default:
                /* STALL control pipe */
                USBD_SetStall(0);
            }
            // Trigger next Control Out DATA1 Transaction.
            /* Status stage */
            USBD_PrepareCtrlOut(0,0);
            break;
        }

        case UAC_GET_MAX:
        {
            switch (buf[3])
            {
            case VOLUME_CONTROL:
            {
                if (REC_FEATURE_UNITID == buf[5])
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMaxVolume;
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecMaxVolume >> 8;
                }
                else if (PLAY_FEATURE_UNITID == buf[5])
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMaxVolume;
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayMaxVolume >> 8;
                }
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 2);
                break;
            }
            default:
                /* STALL control pipe */
                USBD_SetStall(0);
            }
            // Trigger next Control Out DATA1 Transaction.
            /* Status stage */
            USBD_PrepareCtrlOut(0,0);
            break;
        }

        case UAC_GET_RES:
        {
            switch (buf[3])
            {
            case VOLUME_CONTROL:
            {
                if (REC_FEATURE_UNITID == buf[5])
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecResVolume;
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecResVolume >> 8;
                }
                else if (PLAY_FEATURE_UNITID == buf[5])
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayResVolume;
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayResVolume >> 8;
                }
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 2);
                break;
            }
            default:
                /* STALL control pipe */
                USBD_SetStall(0);
            }
            // Trigger next Control Out DATA1 Transaction.
            /* Status stage */
            USBD_PrepareCtrlOut(0,0);
            break;
        }

        default:
        {
            /* Setup error, stall the device */
            USBD_SetStall(0);
        }
        }
    }
    else
    {
        // Host to device
        switch (buf[1])
        {
        case UAC_SET_CUR:
        {
            switch (buf[3])
            {
            case MUTE_CONTROL:
                if (REC_FEATURE_UNITID == buf[5])
                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecMute, buf[6]);
                else if (PLAY_FEATURE_UNITID == buf[5])
                {
                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayMute, buf[6]);
                }
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;

            case VOLUME_CONTROL:
                if (REC_FEATURE_UNITID == buf[5])
                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecVolume, buf[6]);
                else if (PLAY_FEATURE_UNITID == buf[5])
                {
                    if (buf[2] == 1)
                    {
                        USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayVolumeL, buf[6]);
                        if(g_usbd_PlayVolumeL & 0x8000)
                            g_usbd_PlayVolumeL = (g_usbd_PlayVolumeL & 0x7FFF) >> 8;
                        else
                            g_usbd_PlayVolumeL = (g_usbd_PlayVolumeL >> 7);
                        bPlayVolumeLAdjust = TRUE; //ATOM_I2C_WriteWAU8822(11, i16PlayVolumeL | 0x100);   /* Set left DAC volume */
                    }
                    else
                    {
                        USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayVolumeR, buf[6]);
                        if(g_usbd_PlayVolumeR & 0x8000)
                            g_usbd_PlayVolumeR = (g_usbd_PlayVolumeR & 0x7FFF) >> 8;
                        else
                            g_usbd_PlayVolumeR = (g_usbd_PlayVolumeR >> 7);
                        bPlayVolumeRAdjust = TRUE; //ATOM_I2C_WriteWAU8822(12, i16PlayVolumeR | 0x100);   /* Set right DAC volume */
                    }
                }
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;

            default:
                /* STALL control pipe */
                USBD_SetStall(0);
                break;
            }
            break;
        }
        case SET_REPORT:
        {
            if(buf[3] == 2)
            {
                /* Request Type = Output */
                USBD_SET_DATA1(EP1);
                USBD_SET_PAYLOAD_LEN(EP1, buf[6]);

                /* Status stage */
                USBD_PrepareCtrlIn(0, 0);
            }
            break;
        }
        case SET_IDLE:
        {
            /* Status stage */
            USBD_SET_DATA1(EP0);
            USBD_SET_PAYLOAD_LEN(EP0, 0);
            break;
        }
        case SET_PROTOCOL:
        default:
        {
            /* Setup error, stall the device */
            USBD_SetStall(0);
            break;
        }
        }
    }
}

/**
 * @brief       Set Interface standard request
 *
 * @param[in]   u32AltInterface Interface
 *
 * @return      None
 *
 * @details     This function is used to set UAC Class relative setting
 */
void UAC_SetInterface(uint32_t u32AltInterface)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if (buf[4] == 1)
    {
        /* Audio Iso IN interface */
        if (u32AltInterface == 1)
        {
            g_usbd_UsbAudioState = UAC_START_AUDIO_RECORD;
            USBD_SET_DATA1(EP2);
            USBD_SET_PAYLOAD_LEN(EP2, 0);
            UAC_DeviceEnable(UAC_MICROPHONE);
        }
        else if (u32AltInterface == 0)
        {
            UAC_DeviceDisable(UAC_MICROPHONE);
            USBD_SET_DATA1(EP2);
            USBD_SET_PAYLOAD_LEN(EP2, 0);
            g_usbd_UsbAudioState = UAC_STOP_AUDIO_RECORD;
        }
    }
    else if (buf[4] == 2)
    {
        /* Audio Iso OUT interface */
        if (u32AltInterface == 1)
        {
            USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
            UAC_DeviceEnable(UAC_SPEAKER);
        }
        else
            UAC_DeviceDisable(UAC_SPEAKER);
    }
}

/*******************************************************************/
/* For I2C transfer */
__IO uint32_t EndFlag0 = 0;
uint8_t Device_Addr0 = 0x1A;                /* WAU8822 Device ID */
uint8_t Tx_Data0[2];
uint8_t DataCnt0;

typedef enum
{
    E_RS_NONE,          // no resampling
    E_RS_UP,            // up sampling
    E_RS_DOWN           // down sampling
} RESAMPLE_STATE_T;


#define BUFF_LEN    32*12
#define REC_LEN     REC_RATE / 1000

/* Recoder Buffer and its pointer */
uint16_t PcmRecBuff[BUFF_LEN] = {0};
uint32_t u32RecPos_Out = 0;
uint32_t u32RecPos_In = 0;

/* Player Buffer and its pointer */
uint32_t PcmPlayBuff[BUFF_LEN] = {0};
uint32_t u32PlayPos_Out = 0;
uint32_t u32PlayPos_In = 0;

static void Delay(uint32_t t)
{
    volatile int32_t delay;

    delay = t;

    while(delay-- >= 0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of WAU8822 with I2C0                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data)
{
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, 0x1A<<1);
    I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    I2C_WAIT_READY(I2C0);

    I2C_STOP(I2C0);

    bIsI2CIdle = TRUE;
    EndFlag0 = 1;
}

static void ATOM_I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data)
{
    if(!bIsI2CIdle)
        while (EndFlag0 == 0);

    I2C_WriteWAU8822(u8addr, u16data);
}

void WAU8822_Setup(void)
{
    I2C_WriteWAU8822(0,  0x000);   /* Reset all registers */
    Delay(0x200);

    I2C_WriteWAU8822(1,  0x02F);
    I2C_WriteWAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteWAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteWAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteWAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteWAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteWAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteWAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteWAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteWAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteWAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteWAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteWAU8822(47, 0x050);   /* LLIN connected, and its Gain value */
    I2C_WriteWAU8822(48, 0x050);   /* RLIN connected, and its Gain value */
    I2C_WriteWAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteWAU8822(51, 0x001);   /* Right DAC connected to RMIX */
}

void Tx_thresholdCallbackfn(void)
{
    uint32_t i;
    uint32_t * pBuff;

    if ((u32PlayPos_Out < u32PlayPos_In - 4) || (u32PlayPos_Out > u32PlayPos_In))
    {
        pBuff = &PcmPlayBuff[u32PlayPos_Out];

        for( i = 0; i < 4; i++)
        {
            I2S_WRITE_TX_FIFO(I2S, pBuff[i]);
        }

        u32PlayPos_Out += 4;

        if (u32PlayPos_Out >= BUFF_LEN)
        {
            u32PlayPos_Out = 0;
        }
    }
    else
    {
        for( i = 0; i < 4; i++)
        {
            I2S_WRITE_TX_FIFO(I2S, 0x00);
        }
    }
}

void Rx_thresholdCallbackfn(void)
{
    int32_t  i;
    uint16_t *pBuff;

    pBuff = &PcmRecBuff[u32RecPos_In];

    for ( i = 0; i < 4; i++ )
    {
        pBuff[i] = (uint16_t)(I2S_READ_RX_FIFO(I2S) & 0xFFFF);
    }

    u32RecPos_In += 4;

    if (u32RecPos_In >= BUFF_LEN)
    {
        u32RecPos_In = 0;
    }
}

static uint8_t u8RecEn = 0;
static uint8_t u8PlayEn = 0;
void I2S_IRQHandler(void)
{
    uint32_t u32I2SIntFlag;

    u32I2SIntFlag = I2S_GET_INT_FLAG(I2S, I2S_STATUS_I2STXINT_Msk | I2S_STATUS_I2SRXINT_Msk);

    if (u32I2SIntFlag & I2S_STATUS_I2STXINT_Msk)
    {
        /* Tx threshold level */
        if (I2S_GET_INT_FLAG(I2S,I2S_STATUS_TXTHF_Msk) & I2S_STATUS_TXTHF_Msk)
        {
            if (u8PlayEn)
                Tx_thresholdCallbackfn();
        }
    }
    else if (u32I2SIntFlag & I2S_STATUS_I2SRXINT_Msk)
    {
        /* Rx threshold level */
        if (I2S_GET_INT_FLAG(I2S, I2S_STATUS_RXTHF_Msk) & I2S_STATUS_RXTHF_Msk)
        {
            if (u8RecEn)
                Rx_thresholdCallbackfn();
        }
    }
}



/**
  * @brief  SendRecData, prepare the record data for next ISO transfer.
  * @param  None.
  * @retval None.
  */
void UAC_SendRecData(void)
{
    uint16_t *pBuff;

    if ((u32RecPos_Out < u32RecPos_In - REC_LEN) || (u32RecPos_Out > u32RecPos_In))
    {
        pBuff = &PcmRecBuff[u32RecPos_Out];

        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (void *)pBuff, REC_LEN * 2);
        USBD_SET_PAYLOAD_LEN(EP2, REC_LEN * 2);
        u32RecPos_Out += REC_LEN;

        if (u32RecPos_Out >= BUFF_LEN)
        {
            u32RecPos_Out = 0;
        }
    }
    else
    {
        USBD_SET_PAYLOAD_LEN(EP2, 0);
    }
}


/**
  * @brief  UAC_DeviceEnable. To enable the device to play or record audio data.
  * @param  u8Object: To select the device, UAC_MICROPHONE or UAC_SPEAKER.
  * @retval None.
  */
void UAC_DeviceEnable(uint8_t u8Object)
{
    if (u8Object == UAC_MICROPHONE)
    {
        /* Enable record hardware */
        u8RecEn = 1;
        I2S_EnableInt(I2S, I2S_INTEN_RXTHIE_Msk);
        I2S_ENABLE_RX(I2S);
    }
    else
    {
        /* Eanble play hardware */
        u8PlayEn = 1;
        I2S_EnableInt(I2S, I2S_INTEN_TXTHIE_Msk);
        I2S_ENABLE_TX(I2S);
    }
    NVIC_EnableIRQ(I2S_IRQn);
    NVIC_SetPriority(I2S_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    TIMER_Start(TIMER0);
}


/**
  * @brief  UAC_DeviceDisable. To disable the device to play or record audio data.
  * @param  u8Object: To select the device, UAC_MICROPHONE or UAC_SPEAKER.
  * @retval None.
  */
void UAC_DeviceDisable(uint8_t u8Object)
{
    if (u8Object ==  UAC_MICROPHONE )
    {
        /* Disable record hardware/stop record */
        u8RecEn = 0;
        I2S_DisableInt(I2S, I2S_INTEN_RXTHIE_Msk);
        I2S_DISABLE_RX(I2S);
    }
    else
    {
        /* Disable play hardware/stop play */
        u8PlayEn = 0;
        I2S_DisableInt(I2S, I2S_INTEN_TXTHIE_Msk);
        I2S_DISABLE_TX(I2S);
    }
    TIMER0->CTL |= TIMER_CTL_SW_RST_Msk;
}


/**
  * @brief  GetPlayData, To get data from ISO OUT to play buffer.
  * @param  pi16src: The data buffer of ISO OUT.
  *         i16Samples: The sample number in data buffer.
  * @retval None.
  */
void UAC_GetPlayData(int16_t *pi16src, int16_t i16Samples)
{
    uint32_t u32len, i;
    uint32_t *pBuff;
    uint32_t *pSrc;

    u32len = PLAY_RATE/1000;
    pBuff = &PcmPlayBuff[u32PlayPos_In];
    pSrc = (uint32_t *) pi16src;

    for ( i = 0; i < u32len; i++ )
    {
        pBuff[i] = pSrc[i];
    }

    u32PlayPos_In += u32len;

    if (u32PlayPos_In >= BUFF_LEN)
    {
        u32PlayPos_In = 0;
    }
}

void AdjustCodecPll(RESAMPLE_STATE_T r)
{
    static uint16_t tb[3][3] = {{0x00C, 0x093, 0x0E9}, // 8.192
        {0x00E, 0x1D2, 0x1E3},  // * 1.005 = 8.233
        {0x009, 0x153, 0x1EF}
    }; // * .995 = 8.151
    static RESAMPLE_STATE_T current = E_RS_NONE;
    int i, s;

    if(r == current)
        return;
    else
        current = r;
    switch(r)
    {
    case E_RS_UP:
        s = 1;
        break;
    case E_RS_DOWN:
        s = 2;
        break;
    case E_RS_NONE:
    default:
        s = 0;
    }

    for(i=0; i<3; i++)
        ATOM_I2C_WriteWAU8822(37+i, tb[s][i]);
}

//======================================================
void TMR0_IRQHandler(void)
{
    TIMER_ClearIntFlag(TIMER0);

    if(u8PlayEn)
    {
        if(u32PlayPos_In >= u32PlayPos_Out)
        {
            if((u32PlayPos_In-u32PlayPos_Out) > (EP2_MAX_PKT_SIZE+8))
            {
                AdjustCodecPll(E_RS_UP);
            }
            else if((u32PlayPos_In-u32PlayPos_Out) < (EP2_MAX_PKT_SIZE-8))
            {
                AdjustCodecPll(E_RS_DOWN);
            }
            else
            {
                AdjustCodecPll(E_RS_NONE);
            }
        }
        else
        {
            if((u32PlayPos_In+BUFF_LEN-u32PlayPos_Out) > (EP2_MAX_PKT_SIZE+8))
            {
                AdjustCodecPll(E_RS_UP);
            }
            else if((u32PlayPos_In+BUFF_LEN-u32PlayPos_Out) < (EP2_MAX_PKT_SIZE-8))
            {
                AdjustCodecPll(E_RS_DOWN);
            }
            else
            {
                AdjustCodecPll(E_RS_NONE);
            }
        }
    }
    else if(u8RecEn)
    {
    }
}

/***************************************************************/
#define HID_CMD_SIGNATURE   0x43444948

/* HID Transfer Commands */
#define HID_CMD_NONE     0x00
#define HID_CMD_ERASE    0x71
#define HID_CMD_READ     0xD2
#define HID_CMD_WRITE    0xC3
#define HID_CMD_TEST     0xB4

#define PAGE_SIZE        2048
#define TEST_PAGES       4
#define SECTOR_SIZE      4096
#define START_SECTOR     0x10

#ifdef __ICCARM__
typedef __packed struct
{
    uint8_t u8Cmd;
    uint8_t u8Size;
    uint32_t u32Arg1;
    uint32_t u32Arg2;
    uint32_t u32Signature;
    uint32_t u32Checksum;
} CMD_T;

#else
typedef struct __attribute__((__packed__))
{
    uint8_t u8Cmd;
    uint8_t u8Size;
    uint32_t u32Arg1;
    uint32_t u32Arg2;
    uint32_t u32Signature;
    uint32_t u32Checksum;
}
CMD_T;
#endif

CMD_T gCmd;

static uint8_t  g_u8PageBuff[PAGE_SIZE] = {0};    /* Page buffer to upload/download through HID report */
static uint32_t g_u32BytesInPageBuf = 0;          /* The bytes of data in g_u8PageBuff */
static uint8_t  g_u8TestPages[TEST_PAGES * PAGE_SIZE] = {0};    /* Test pages to upload/download through HID report */

int32_t HID_CmdEraseSectors(CMD_T *pCmd)
{
    uint32_t u32StartSector;
    uint32_t u32Sectors;

    u32StartSector = pCmd->u32Arg1 - START_SECTOR;
    u32Sectors = pCmd->u32Arg2;

    DBG_PRINTF("Erase command - Sector: %d   Sector Cnt: %d\n", u32StartSector, u32Sectors);

    /* TODO: To erase the sector of storage */
    memset(g_u8TestPages + u32StartSector * SECTOR_SIZE, 0xFF, sizeof(uint8_t) * u32Sectors * SECTOR_SIZE);

    /* To note the command has been done */
    pCmd->u8Cmd = HID_CMD_NONE;

    return 0;
}


int32_t HID_CmdReadPages(CMD_T *pCmd)
{
    uint32_t u32StartPage;
    uint32_t u32Pages;

    u32StartPage = pCmd->u32Arg1;
    u32Pages     = pCmd->u32Arg2;

    DBG_PRINTF("Read command - Start page: %d    Pages Numbers: %d\n", u32StartPage, u32Pages);

    if(u32Pages)
    {
        /* Update data to page buffer to upload */
        /* TODO: We need to update the page data if got a page read command. (0xFF is used in this sample code) */
        memcpy(g_u8PageBuff, g_u8TestPages, sizeof(g_u8PageBuff));
        g_u32BytesInPageBuf = PAGE_SIZE;

        /* The signature word is used as page counter */
        pCmd->u32Signature = 1;

        /* Trigger HID IN */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), (void *)g_u8PageBuff, EP4_MAX_PKT_SIZE);
        USBD_SET_PAYLOAD_LEN(EP4, EP4_MAX_PKT_SIZE);
        g_u32BytesInPageBuf -= EP4_MAX_PKT_SIZE;
    }

    return 0;
}


int32_t HID_CmdWritePages(CMD_T *pCmd)
{
    uint32_t u32StartPage;
    uint32_t u32Pages;

    u32StartPage = pCmd->u32Arg1;
    u32Pages     = pCmd->u32Arg2;

    DBG_PRINTF("Write command - Start page: %d    Pages Numbers: %d\n", u32StartPage, u32Pages);
    g_u32BytesInPageBuf = 0;

    /* The signature is used to page counter */
    pCmd->u32Signature = 0;

    return 0;
}


int32_t gi32CmdTestCnt = 0;
int32_t HID_CmdTest(CMD_T *pCmd)
{
    int32_t i;
    uint8_t *pu8;

    pu8 = (uint8_t *)pCmd;
    DBG_PRINTF("Get test command #%d (%d bytes)\n", gi32CmdTestCnt++, pCmd->u8Size);
    for(i=0; i<pCmd->u8Size; i++)
    {
        if((i&0xF) == 0)
        {
            DBG_PRINTF("\n");
        }
        DBG_PRINTF(" %02x", pu8[i]);
    }

    DBG_PRINTF("\n");


    /* To note the command has been done */
    pCmd->u8Cmd = HID_CMD_NONE;

    return 0;
}


uint32_t CalCheckSum(uint8_t *buf, uint32_t size)
{
    uint32_t sum;
    int32_t i;

    i = 0;
    sum = 0;
    while(size--)
    {
        sum+=buf[i++];
    }

    return sum;

}


int32_t ProcessCommand(uint8_t *pu8Buffer, uint32_t u32BufferLen)
{
    uint32_t u32sum;


    USBD_MemCopy((uint8_t *)&gCmd, pu8Buffer, u32BufferLen);

    /* Check size */
    if((gCmd.u8Size > sizeof(gCmd)) || (gCmd.u8Size > u32BufferLen))
        return -1;

    /* Check signature */
    if(gCmd.u32Signature != HID_CMD_SIGNATURE)
        return -1;

    /* Calculate checksum & check it*/
    u32sum = CalCheckSum((uint8_t *)&gCmd, gCmd.u8Size);
    if(u32sum != gCmd.u32Checksum)
        return -1;

    switch(gCmd.u8Cmd)
    {
    case HID_CMD_ERASE:
    {
        HID_CmdEraseSectors(&gCmd);
        break;
    }
    case HID_CMD_READ:
    {
        HID_CmdReadPages(&gCmd);
        break;
    }
    case HID_CMD_WRITE:
    {
        HID_CmdWritePages(&gCmd);
        break;
    }
    case HID_CMD_TEST:
    {
        HID_CmdTest(&gCmd);
        break;
    }
    default:
        return -1;
    }

    return 0;
}


void HID_GetOutReport(uint8_t *pu8EpBuf, uint32_t u32Size)
{
    uint8_t  u8Cmd;
    uint32_t u32StartPage;
    uint32_t u32Pages;
    uint32_t u32PageCnt;

    /* Get command information */
    u8Cmd        = gCmd.u8Cmd;
    u32StartPage = gCmd.u32Arg1;
    u32Pages     = gCmd.u32Arg2;
    u32PageCnt   = gCmd.u32Signature; /* The signature word is used to count pages */


    /* Check if it is in the data phase of write command */
    if((u8Cmd == HID_CMD_WRITE) &&  (u32PageCnt < u32Pages))
    {
        /* Process the data phase of write command */

        /* Get data from HID OUT */
        USBD_MemCopy(&g_u8PageBuff[g_u32BytesInPageBuf], pu8EpBuf, EP5_MAX_PKT_SIZE);
        g_u32BytesInPageBuf += EP5_MAX_PKT_SIZE;

        /* The HOST must make sure the data is PAGE_SIZE alignment */
        if(g_u32BytesInPageBuf >= PAGE_SIZE)
        {
            DBG_PRINTF("Writing page %d\n", u32StartPage + u32PageCnt);
            /* TODO: We should program received data to storage here */
            memcpy(g_u8TestPages + u32PageCnt * PAGE_SIZE, g_u8PageBuff, sizeof(g_u8PageBuff));
            u32PageCnt++;

            /* Write command complete! */
            if(u32PageCnt >= u32Pages)
            {
                u8Cmd = HID_CMD_NONE;

                DBG_PRINTF("Write command complete.\n");
            }

            g_u32BytesInPageBuf = 0;
        }

        /* Update command status */
        gCmd.u8Cmd        = u8Cmd;
        gCmd.u32Signature = u32PageCnt;
    }
    else
    {
        /* Check and process the command packet */
        if(ProcessCommand(pu8EpBuf, u32Size))
        {
            DBG_PRINTF("Unknown HID command!\n");
        }
    }
}

void HID_SetInReport(void)
{
    uint32_t u32StartPage;
    uint32_t u32TotalPages;
    uint32_t u32PageCnt;
    uint8_t *ptr;
    uint8_t u8Cmd;

    u8Cmd        = gCmd.u8Cmd;
    u32StartPage = gCmd.u32Arg1;
    u32TotalPages= gCmd.u32Arg2;
    u32PageCnt   = gCmd.u32Signature;

    /* Check if it is in data phase of read command */
    if(u8Cmd == HID_CMD_READ)
    {
        /* Process the data phase of read command */
        if((u32PageCnt >= u32TotalPages) && (g_u32BytesInPageBuf == 0))
        {
            /* The data transfer is complete. */
            u8Cmd = HID_CMD_NONE;
            DBG_PRINTF("Read command complete!\n");
        }
        else
        {
            if(g_u32BytesInPageBuf == 0)
            {
                /* The previous page has sent out. Read new page to page buffer */
                /* TODO: We should update new page data here. (0xFF is used in this sample code) */
                DBG_PRINTF("Reading page %d\n", u32StartPage + u32PageCnt);
                memcpy(g_u8PageBuff, g_u8TestPages + u32PageCnt * PAGE_SIZE, sizeof(g_u8PageBuff));

                g_u32BytesInPageBuf = PAGE_SIZE;

                /* Update the page counter */
                u32PageCnt++;
            }

            /* Prepare the data for next HID IN transfer */
            ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4));
            USBD_MemCopy(ptr, (void *)&g_u8PageBuff[PAGE_SIZE - g_u32BytesInPageBuf], EP4_MAX_PKT_SIZE);
            USBD_SET_PAYLOAD_LEN(EP4, EP4_MAX_PKT_SIZE);
            g_u32BytesInPageBuf -= EP4_MAX_PKT_SIZE;
        }
    }

    gCmd.u8Cmd        = u8Cmd;
    gCmd.u32Signature = u32PageCnt;
}

