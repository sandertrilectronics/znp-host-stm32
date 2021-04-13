/*
 * mtUtil.c
 *
 *  Created on: Apr 12, 2021
 *      Author: Sande
 */

#include "mtUtil.h"
#include "rpc.h"
#include "dbgPrint.h"
#include <stddef.h>
#include <string.h>

static mtUtilCb_t mtUtilCbs;

void utilRegisterCallbacks(mtUtilCb_t cbs) {
	memcpy(&mtUtilCbs, &cbs, sizeof(mtUtilCb_t));
}

static void processGetDeviceInfo(uint8_t *rpcBuff, uint8_t rpcLen) {
	// valid function pointer?
	if (mtUtilCbs.pfnUtilGetDeviceInfoCb_t) {
		uint8_t msgIdx = 2;
		utilGetDeviceInfoFormat_t rsp;

		// size error?
		if (rpcLen < 1) {
			printf("MT_RPC_ERR_LENGTH\n");
		}

		// copy command data
		rsp.success = rpcBuff[msgIdx++];
		rsp.ieee_addr = 0;
		for (uint8_t i = 0; i < 8; i++)
			rsp.ieee_addr |= ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);
		rsp.short_addr = rpcBuff[msgIdx++];
		rsp.short_addr |= (rpcBuff[msgIdx++] << 8);
		rsp.device_type = rpcBuff[msgIdx++];
		rsp.device_state = rpcBuff[msgIdx++];
#warning possible loss of data
		rsp.ass_device_cnt = MIN(rpcBuff[msgIdx], ASS_DEVICE_LIST_MAX);
		msgIdx++;
		if (rsp.ass_device_cnt) {
			for (uint8_t i = 0; i < rsp.ass_device_cnt; i++) {
				rsp.ass_device_list[i] = rpcBuff[msgIdx++];
				rsp.ass_device_list[i] |= (rpcBuff[msgIdx++] << 8);
			}
		}

		// callback function
		mtUtilCbs.pfnUtilGetDeviceInfoCb_t(&rsp);
	}
}

static void processSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	//srspRpcLen = rpcLen;
	switch (rpcBuff[1]) {
		case MT_UTIL_SRSP_GET_DEVICE_INFO:
			dbg_print(PRINT_LEVEL_VERBOSE, "utilProcess: SET_CHANNEL_SRSP\n");
			processGetDeviceInfo(rpcBuff, rpcLen);
			break;
		default:
			dbg_print(PRINT_LEVEL_INFO, "processSrsp: unsupported message [%x:%x]\n", rpcBuff[0], rpcBuff[1]);
			break;
	}
}

void utilProcess(uint8_t *rpcBuff, uint8_t rpcLen) {
	dbg_print(PRINT_LEVEL_VERBOSE, "utilProcess: processing CMD0:%x, CMD1:%x\n", rpcBuff[0], rpcBuff[1]);

	//process the synchronous SRSP from SREQ
	if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP) {
		processSrsp(rpcBuff, rpcLen);
	}
	else {
		//Read CMD1 and processes the specific SREQ
		switch (rpcBuff[1]) {
			default:
				dbg_print(PRINT_LEVEL_WARNING, "processRpcAf: CMD0:%x, CMD1:%x, not handled\n", rpcBuff[0], rpcBuff[1]);
				break;
		}
	}
}

uint8_t utilGetDeviceInfo(void) {
	uint8_t status;

	// send the frame
	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_UTIL), 0x00, NULL, 0);

	// wait for a response
	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	// return
	return status;
}
