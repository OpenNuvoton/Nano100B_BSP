/******************************************************************************
 * @file     vendor_lbk.h
 * @brief    NANO100 series USB driver header file
 * @version  2.0.0
 * @date     22, March, 2013
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_LBK_H__
#define __USBD_LBK_H__

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0xFFFD

/*!<Define Vendor Class Specific Request */
#define REQ_SET_DATA          0x01
#define REQ_GET_DATA          0x12

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE               64
#define EP1_MAX_PKT_SIZE               EP0_MAX_PKT_SIZE
#define EP2_MAX_PKT_SIZE               8
#define EP3_MAX_PKT_SIZE               8
#define EP4_MAX_PKT_SIZE               64     // Iso-in
#define EP5_MAX_PKT_SIZE               64     // Iso-out
#define EP6_MAX_PKT_SIZE               64
#define EP7_MAX_PKT_SIZE               64

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE
#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
#define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN         EP3_MAX_PKT_SIZE
#define EP4_BUF_BASE        (EP3_BUF_BASE + EP3_BUF_LEN)
#define EP4_BUF_LEN         EP4_MAX_PKT_SIZE
#define EP5_BUF_BASE        (EP4_BUF_BASE + EP4_BUF_LEN)
#define EP5_BUF_LEN         EP5_MAX_PKT_SIZE
#define EP6_BUF_BASE        (EP5_BUF_BASE + EP5_BUF_LEN)
#define EP6_BUF_LEN         EP6_MAX_PKT_SIZE
#define EP7_BUF_BASE        (EP6_BUF_BASE + EP6_BUF_LEN)
#define EP7_BUF_LEN         EP7_MAX_PKT_SIZE

/* Define EP number */
#define NUMBER_OF_EP                   6
#define INT_IN_EP_NUM                  0x01
#define INT_OUT_EP_NUM                 0x02
#define ISO_IN_EP_NUM                  0x03
#define ISO_OUT_EP_NUM                 0x04
#define BULK_IN_EP_NUM                 0x05
#define BULK_OUT_EP_NUM                0x06

/* Define Descriptor information */
#define INT_IN_INTERVAL                4
#define INT_OUT_INTERVAL               4
#define ISO_IN_INTERVAL                1
#define ISO_OUT_INTERVAL               1
#define USBD_SELF_POWERED              0
#define USBD_REMOTE_WAKEUP             0
#define USBD_MAX_POWER                 50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

#define LEN_CONFIG_AND_SUBORDINATE      (LEN_CONFIG+LEN_INTERFACE+(LEN_ENDPOINT*NUMBER_OF_EP))


/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
void LBK_Init(void);
void Vendor_ClassRequest(void);

void LBK_BulkOut(uint8_t *u8Addr, uint32_t u32Len);
void LBK_BulkInPushData(uint8_t *u8Addr, uint32_t u32Len);

void LBK_IsoInPushData(uint8_t *u8Addr, uint8_t u8Len);

void LBK_IntOut(void);
void LBK_IntInData(void);

void EP5_Handler(void);

extern uint8_t  g_CtrlLbkBuff[64];
extern uint8_t  g_IntLbkBuff[8];
extern uint8_t  g_IsoLbkBuff[64];
extern uint8_t  g_BulkLbkBuff[512];

#endif  /* __USBD_LBK_H_ */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
