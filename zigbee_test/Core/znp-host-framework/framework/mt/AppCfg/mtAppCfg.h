/*
 * mtAppCfg.h
 *
 *  Created on: Apr 2, 2021
 *      Author: Sande
 */

#ifndef _MTAPPCFG_H_
#define _MTAPPCFG_H_

#include <stdint.h>

#define CFG_CHANNEL_NONE			0x00000000
#define CFG_CHANNEL_0x00000800		0x00000800 // 11
#define CFG_CHANNEL_0x00001000		0x00001000 // 12
#define CFG_CHANNEL_0x00002000		0x00002000 // 13
#define CFG_CHANNEL_0x00004000		0x00004000 // 14
#define CFG_CHANNEL_0x00008000		0x00008000 // 15
#define CFG_CHANNEL_0x00010000		0x00010000 // 16
#define CFG_CHANNEL_0x00020000		0x00020000 // 17
#define CFG_CHANNEL_0x00040000		0x00040000 // 18
#define CFG_CHANNEL_0x00080000		0x00080000 // 19
#define CFG_CHANNEL_0x00100000		0x00100000 // 20
#define CFG_CHANNEL_0x00200000		0x00200000 // 21
#define CFG_CHANNEL_0x00400000		0x00400000 // 22
#define CFG_CHANNEL_ALL				0x007FF800 // ALL

#define CFG_COMM_MODE_TOUCHLINK					0x01
#define CFG_COMM_MODE_NWK_STEERING				0x02
#define CFG_COMM_MODE_NWK_FORMATION				0x04
#define CFG_COMM_MODE_FIND_BIND					0x08

#define MT_APP_CFG_SRSP_SET_CHANNEL				0x08
#define MT_APP_CFG_SRSP_START_COMMISSIONING		0x05

#define MT_APP_CFG_COMMISSIONING_NOTIFY			0x80

typedef struct {
	uint8_t primaryChannel;
	uint32_t channel;
} setChannelFormat_t;

typedef struct {
	uint8_t commissioningMode;
} startCommissioningFormat_t;

typedef struct {
	uint8_t status;
	uint8_t commissioningMode1;
	uint8_t commissioningMode2;
} appCfgCommissioningNotifyFormat_t;

typedef struct {
	uint8_t success;
} appCfgSetChannelFormat_t;

typedef struct {
	uint8_t success;
} appCfgStartCommissioningStart_t;

typedef uint8_t (*mtAppCfgCommissioningNotifyCb_t)(appCfgCommissioningNotifyFormat_t *msg);
typedef uint8_t (*mtAppCfgSetChannelCb_t)(appCfgSetChannelFormat_t *msg);
typedef uint8_t (*mtAppCfgCommissioningStartCb_t)(appCfgStartCommissioningStart_t *msg);

typedef struct {
	mtAppCfgCommissioningNotifyCb_t pfnAppCfgCommissioningNotifyCb_t;
	mtAppCfgSetChannelCb_t pfnAppCfgSetChannelCb_t;
	mtAppCfgCommissioningStartCb_t pfnAppCfgCommissioningStartCb_t;
} mtAppCfgCb_t;

void appCfgRegisterCallbacks(mtAppCfgCb_t cbs);
void appCfgProcess(uint8_t *rpcBuff, uint8_t rpcLen);
uint8_t appCfgSetChannel(setChannelFormat_t *req);
uint8_t appCfgStartCommissioning(startCommissioningFormat_t *req);

#endif /* _MTAPPCFG_H_ */
