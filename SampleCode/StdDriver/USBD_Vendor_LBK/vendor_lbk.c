/******************************************************************************
 * @file     vendor_lbk.c
 * @brief    NANO100 series USBD driver Sample file
 * @version  2.0.0
 * @date     22, Sep, 2014
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "Nano100Series.h"
#include "vendor_lbk.h"


uint8_t volatile g_u8EP2Ready = 0;
uint8_t volatile g_u8EP3Ready = 0;
uint8_t volatile g_u8EP4Ready = 0;
uint8_t volatile g_u8EP5Ready = 0;
uint8_t volatile g_u8EP6Ready = 0;
uint8_t volatile g_u8EP7Ready = 0;

uint32_t g_u32BytesInBulkBuf;
uint8_t g_u8Size;

uint8_t  g_CtrlLbkBuff[64];
uint8_t  g_IntLbkBuff[8];
uint8_t  g_IsoLbkBuff[64];
uint8_t  g_BulkLbkBuff[512];

void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

//------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET) {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED()) {
            /* USB Plug In */
            USBD_ENABLE_USB();
        } else {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

//------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS) {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST) {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
        }
        if (u32State & USBD_STATE_SUSPEND) {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }
        if (u32State & USBD_STATE_RESUME) {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
        }
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_WAKEUP)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
    }

    if (u32IntSts & USBD_INTSTS_USB) {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP) {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);
            // control OUT
            USBD_CtrlOut();
        }

        // Interrupt IN
        if (u32IntSts & USBD_INTSTS_EP2) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
        }

        // Interrupt OUT
        if (u32IntSts & USBD_INTSTS_EP3) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);

            // Interrupt OUT
            g_u8EP3Ready = 1;
        }

        /* Isochronous IN */
        if (u32IntSts & USBD_INTSTS_EP4) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        /* Isochronous OUT */
        if (u32IntSts & USBD_INTSTS_EP5) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);

            // Isochronous OUT
            EP5_Handler();
        }

        /* BULK IN */
        if (u32IntSts & USBD_INTSTS_EP6) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        /* BULK OUT */
        if (u32IntSts & USBD_INTSTS_EP7) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);

            // BULK OUT
            g_u8EP7Ready = 1;
        }
    }
}

/* Iso OUT handler */
void EP5_Handler(void)
{
    USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
    USBD_MemCopy((uint8_t *)g_IsoLbkBuff,  (uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP5)) , EP5_MAX_PKT_SIZE);

    g_u8EP4Ready = 1;
}

/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void LBK_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
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
    /* EP2 ==> Interrupt IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /*****************************************************/
    /* EP3 ==> Interrupt OUT endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | INT_OUT_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);

    /* trigger to receive Interrupt OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /*****************************************************/
    /* EP4 ==> Isochronous IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | ISO_IN_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer range for EP4 */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    /*****************************************************/
    /* EP5 ==> Isochronous OUT endpoint, address 4 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_OUT | ISO_OUT_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer range for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);

    /* trigger to receive Isochronous OUT data */
    USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);

    /*****************************************************/
    /* EP6 ==> Bulk IN endpoint, address 5 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer range for EP6 */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);

    /*****************************************************/
    /* EP7 ==> Bulk OUT endpoint, address 6 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer range for EP7 */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);

    /* trigger to receive Bulk OUT data */
    USBD_SET_PAYLOAD_LEN(EP7, EP7_MAX_PKT_SIZE);
}

void Vendor_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if (buf[0] & 0x80) {
        // Device to host
        switch (buf[1]) {
        case REQ_GET_DATA:

            USBD_PrepareCtrlIn((uint8_t *)g_CtrlLbkBuff, EP0_MAX_PKT_SIZE);

            /* Data stage */
            USBD_SET_DATA1(EP0);
            USBD_SET_PAYLOAD_LEN(EP0, EP0_MAX_PKT_SIZE);
            /* Status stage */
            USBD_PrepareCtrlOut(0,0);
            break;

        default:
            /* Setup error, stall the device */
            USBD_SetStall(0);
            break;
        }
    } else {
        // Host to device
        switch (buf[1]) {
        case REQ_SET_DATA:

            USBD_PrepareCtrlOut((uint8_t *)g_CtrlLbkBuff, buf[6]);

            /* Status stage */
            USBD_SET_DATA1(EP0);
            USBD_SET_PAYLOAD_LEN(EP0, 0);
            break;

        default:
            // Stall
            /* Setup error, stall the device */
            USBD_SetStall(0);
            break;
        }
    }
}

void LBK_IsoInPushData(uint8_t *u8Addr, uint8_t u8Len)
{
    if (g_u8EP4Ready) {
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), u8Addr, u8Len);
        USBD_SET_PAYLOAD_LEN(EP4, u8Len);
        g_u8EP4Ready = 0;
    } else {
        // Not an error. USB Host did not get the last ISO-in packet.
    }
}

void LBK_BulkOut(uint8_t *u8Addr, uint32_t u32Len)
{
    if(g_u8EP7Ready) {
        g_u32BytesInBulkBuf = u32Len;

        if (g_u32BytesInBulkBuf > 0) {
            /* Set the packet size */
            if (u32Len > EP7_MAX_PKT_SIZE)
                g_u8Size = EP7_MAX_PKT_SIZE;
            else
                g_u8Size = g_u32BytesInBulkBuf;

            /* Bulk OUT buffer */
            USBD_MemCopy(u8Addr, (uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), g_u8Size);
            u8Addr += g_u8Size;

            /* kick - start */
            USBD_SET_EP_BUF_ADDR(EP7, USBD_GET_EP_BUF_ADDR(EP7));
            /* Trigger to send out the data packet */
            USBD_SET_PAYLOAD_LEN(EP7, g_u8Size);
            u32Len -= g_u8Size;
            g_u32BytesInBulkBuf -= g_u8Size;
        }
        g_u8EP6Ready = 1;
    }
}

void LBK_BulkInPushData(uint8_t *u8Addr, uint32_t u32Len)
{
    if(g_u8EP6Ready) {
        g_u32BytesInBulkBuf = u32Len;

        if (g_u32BytesInBulkBuf > 0) {
            /* Set the packet size */
            if (u32Len > EP6_MAX_PKT_SIZE)
                g_u8Size = EP6_MAX_PKT_SIZE;
            else
                g_u8Size = g_u32BytesInBulkBuf;

            USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP6)), u8Addr, g_u8Size);
            u8Addr += g_u8Size;

            /* kick - start */
            USBD_SET_EP_BUF_ADDR(EP6, USBD_GET_EP_BUF_ADDR(EP6));
            /* Trigger to send out the data packet */
            USBD_SET_PAYLOAD_LEN(EP6, g_u8Size);
            u32Len -= g_u8Size;
            g_u32BytesInBulkBuf -= g_u8Size;
        }

        g_u8EP6Ready = 0;
    }
}


void LBK_IntOut(void)
{
    if (g_u8EP3Ready) {
        USBD_MemCopy((uint8_t *)g_IntLbkBuff, (uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3)), EP3_MAX_PKT_SIZE);
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
        g_u8EP2Ready = 1;
    }

}

void LBK_IntInData(void)
{
    if (g_u8EP2Ready) {
        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)g_IntLbkBuff, EP2_MAX_PKT_SIZE);
        USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);

        g_u8EP2Ready = 0;
    }
}
