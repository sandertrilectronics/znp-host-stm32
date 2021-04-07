/*
 * mtAppCfg.c
 *
 *  Created on: Apr 2, 2021
 *      Author: Sande
 */

#include "mtAppCfg.h"
#include "rpc.h"
#include "dbgPrint.h"
#include <string.h>
#include <stdio.h>

static mtAppCfgCb_t mtAppCfgCbs;

void appCfgRegisterCallbacks(mtAppCfgCb_t cbs) {
	memcpy(&mtAppCfgCbs, &cbs, sizeof(mtAppCfgCb_t));
}

static void processCommissioningNotify(uint8_t *rpcBuff, uint8_t rpcLen) {
	// valid function pointer?
	if (mtAppCfgCbs.pfnAppCfgCommissioningNotifyCb_t) {
		uint8_t msgIdx = 2;
		appCfgCommissioningNotifyFormat_t rsp;

		// size error?
		if (rpcLen < 3) {
			printf("MT_RPC_ERR_LENGTH\n");
		}

		// copy command
		rsp.status = rpcBuff[msgIdx++];
		rsp.commissioningMode1 = rpcBuff[msgIdx++];
		rsp.commissioningMode2 = rpcBuff[msgIdx++];

		// callback function
		mtAppCfgCbs.pfnAppCfgCommissioningNotifyCb_t(&rsp);
	}
}

static void processSetChannel(uint8_t *rpcBuff, uint8_t rpcLen) {
	// valid function pointer?
	if (mtAppCfgCbs.pfnAppCfgSetChannelCb_t) {
		uint8_t msgIdx = 2;
		appCfgSetChannelFormat_t rsp;

		// size error?
		if (rpcLen < 1) {
			printf("MT_RPC_ERR_LENGTH\n");
		}

		// copy command
		rsp.success = rpcBuff[msgIdx++];

		// callback function
		mtAppCfgCbs.pfnAppCfgSetChannelCb_t(&rsp);
	}
}

static void processStartCommissioning(uint8_t *rpcBuff, uint8_t rpcLen) {
	// valid function pointer?
	if (mtAppCfgCbs.pfnAppCfgCommissioningStartCb_t) {
		uint8_t msgIdx = 2;
		appCfgStartCommissioningStart_t rsp;

		// size error?
		if (rpcLen < 1) {
			printf("MT_RPC_ERR_LENGTH\n");
		}

		// copy command
		rsp.success = rpcBuff[msgIdx++];

		// callback function
		mtAppCfgCbs.pfnAppCfgCommissioningStartCb_t(&rsp);
	}
}

static void processSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	//copies sresp to local buffer
	//memcpy(srspRpcBuff, rpcBuff, rpcLen);

	//srspRpcLen = rpcLen;
	switch (rpcBuff[1]) {
		case MT_APP_CFG_SRSP_SET_CHANNEL:
			dbg_print(PRINT_LEVEL_VERBOSE, "appCfgProcess: SET_CHANNEL_SRSP\n");
			processSetChannel(rpcBuff, rpcLen);
			break;
		case MT_APP_CFG_SRSP_START_COMMISSIONING:
			dbg_print(PRINT_LEVEL_VERBOSE, "appCfgProcess: START_COMMISSIONING_SRSP\n");
			processStartCommissioning(rpcBuff, rpcLen);
			break;
		default:
			dbg_print(PRINT_LEVEL_INFO, "processSrsp: unsupported message [%x:%x]\n", rpcBuff[0], rpcBuff[1]);
			break;
	}

}

void appCfgProcess(uint8_t *rpcBuff, uint8_t rpcLen) {
	dbg_print(PRINT_LEVEL_VERBOSE, "appCfgProcess: processing CMD0:%x, CMD1:%x\n", rpcBuff[0], rpcBuff[1]);

	//process the synchronous SRSP from SREQ
	if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP) {
		processSrsp(rpcBuff, rpcLen);
	}
	else {
		//Read CMD1 and processes the specific SREQ
		switch (rpcBuff[1]) {
			case MT_APP_CFG_COMMISSIONING_NOTIFY:
				dbg_print(PRINT_LEVEL_VERBOSE, "appCfgProcess: MT_AF_DATA_CONFIRM\n");
				processCommissioningNotify(rpcBuff, rpcLen);
			default:
				dbg_print(PRINT_LEVEL_WARNING, "processRpcAf: CMD0:%x, CMD1:%x, not handled\n", rpcBuff[0], rpcBuff[1]);
				break;
		}
	}
}

uint8_t appCfgSetChannel(setChannelFormat_t *req) {
	uint8_t status;
	uint8_t payload[5];

	// build command
	payload[0] = (req->primaryChannel) ? 1 : 0;
	payload[1] = req->channel & 0xFF;
	payload[2] = (req->channel >> 8) & 0xFF;
	payload[3] = (req->channel >> 16) & 0xFF;
	payload[4] = (req->channel >> 24) & 0xFF;

	// send the frame
	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_APP_CFG), 0x08, payload, 5);

	// wait for a response
	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	// return
	return status;
}

uint8_t appCfgStartCommissioning(startCommissioningFormat_t *req) {
	uint8_t status;
	uint8_t payload[1];

	// build command
	payload[0] = req->commissioningMode;

	// send the frame
	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_APP_CFG), 0x05, payload, 1);

	// wait for a response
	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	// return
	return status;
}
