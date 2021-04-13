/*
 * mtAf.c
 *
 * This module contains the API for the MT AF Interface.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "mtAf.h"
#include "mtParser.h"
#include "rpc.h"
#include "dbgPrint.h"

/*********************************************************************
 * MACROS
 */
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

/*********************************************************************
 * LOCAL VARIABLE
 */
static mtAfCb_t mtAfCbs;
extern uint8_t srspRpcBuff[RPC_MAX_LEN];
extern uint8_t srspRpcLen;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void processSrsp(uint8_t *rpcBuff, uint8_t rpcLen);

uint8_t afRegister(RegisterFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 9 + (req->AppNumInClusters * 2) + (req->AppNumOutClusters * 2);
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = req->EndPoint;
	cmd[cmInd++] = (uint8_t) (req->AppProfId & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->AppProfId >> 8) & 0xFF);
	cmd[cmInd++] = (uint8_t) (req->AppDeviceId & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->AppDeviceId >> 8) & 0xFF);
	cmd[cmInd++] = req->AppDevVer;
	cmd[cmInd++] = req->LatencyReq;
	cmd[cmInd++] = req->AppNumInClusters;
	for (idx = 0; idx < req->AppNumInClusters; idx++) {
		cmd[cmInd++] = (uint8_t) (req->AppInClusterList[idx] & 0xFF);
		cmd[cmInd++] = (uint8_t) ((req->AppInClusterList[idx] >> 8) & 0xFF);
	}
	cmd[cmInd++] = req->AppNumOutClusters;
	for (idx = 0; idx < req->AppNumOutClusters; idx++) {
		cmd[cmInd++] = (uint8_t) (req->AppOutClusterList[idx] & 0xFF);
		cmd[cmInd++] = (uint8_t) ((req->AppOutClusterList[idx] >> 8) & 0xFF);
	}
	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_AF),
	MT_AF_REGISTER, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

uint8_t afDataRequest(DataRequestFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 10 + req->Len;
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = (uint8_t) (req->DstAddr & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->DstAddr >> 8) & 0xFF);
	cmd[cmInd++] = req->DstEndpoint;
	cmd[cmInd++] = req->SrcEndpoint;
	cmd[cmInd++] = (uint8_t) (req->ClusterID & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->ClusterID >> 8) & 0xFF);
	cmd[cmInd++] = req->TransID;
	cmd[cmInd++] = req->Options;
	cmd[cmInd++] = req->Radius;
	cmd[cmInd++] = req->Len;
	for (idx = 0; idx < req->Len; idx++) {
		cmd[cmInd++] = req->Data[idx];

	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_AF),
	MT_AF_DATA_REQUEST, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

uint8_t afDataRequestExt(DataRequestExtFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 20 + req->Len;
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = req->DstAddrMode;
	memcpy((cmd + cmInd), req->DstAddr, 8);
	cmInd += 8;
	cmd[cmInd++] = req->DstEndpoint;
	cmd[cmInd++] = (uint8_t) (req->DstPanID & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->DstPanID >> 8) & 0xFF);
	cmd[cmInd++] = req->SrcEndpoint;
	cmd[cmInd++] = (uint8_t) (req->ClusterId & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->ClusterId >> 8) & 0xFF);
	cmd[cmInd++] = req->TransId;
	cmd[cmInd++] = req->Options;
	cmd[cmInd++] = req->Radius;
	cmd[cmInd++] = (uint8_t) (req->Len & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Len >> 8) & 0xFF);
	for (idx = 0; idx < req->Len; idx++) {
		cmd[cmInd++] = req->Data[idx];
	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_AF),
	MT_AF_DATA_REQUEST_EXT, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

uint8_t afDataRequestSrcRtg(DataRequestSrcRtgFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 11 + (req->RelayCount * 2) + req->Len;
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = (uint8_t) (req->DstAddr & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->DstAddr >> 8) & 0xFF);
	cmd[cmInd++] = req->DstEndpoint;
	cmd[cmInd++] = req->SrcEndpoint;
	cmd[cmInd++] = (uint8_t) (req->ClusterID & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->ClusterID >> 8) & 0xFF);
	cmd[cmInd++] = req->TransID;
	cmd[cmInd++] = req->Options;
	cmd[cmInd++] = req->Radius;
	cmd[cmInd++] = req->RelayCount;
	for (idx = 0; idx < req->RelayCount; idx++) {
		cmd[cmInd++] = (uint8_t) (req->RelayList[idx] & 0xFF);
		cmd[cmInd++] = (uint8_t) ((req->RelayList[idx] >> 8) & 0xFF);
	}
	cmd[cmInd++] = req->Len;
	for (idx = 0; idx < req->Len; idx++) {
		cmd[cmInd++] = req->Data[idx];
	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_AF),
	MT_AF_DATA_REQUEST_SRC_RTG, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

uint8_t afInterPanCtl(InterPanCtlFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 1 + req->Command;
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = req->Command;
	for (idx = 0; idx < req->Command; idx++) {
		cmd[cmInd++] = req->Data[idx];
	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_AF),
	MT_AF_INTER_PAN_CTL, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

uint8_t afDataStore(DataStoreFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 3 + req->Length;
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = (uint8_t) (req->Index & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Index >> 8) & 0xFF);
	cmd[cmInd++] = req->Length;
	for (idx = 0; idx < req->Length; idx++) {
		cmd[cmInd++] = req->Data[idx];
	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_AF),
	MT_AF_DATA_STORE, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

static void processDataConfirm(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtAfCbs.pfnAfDataConfirm) {
		uint8_t msgIdx = 2;
		DataConfirmFormat_t rsp;
		if (rpcLen < 3) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];
		rsp.Endpoint = rpcBuff[msgIdx++];
		rsp.TransId = rpcBuff[msgIdx++];

		mtAfCbs.pfnAfDataConfirm(&rsp);
	}
}

static void processIncomingMsg(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtAfCbs.pfnAfIncomingMsg) {
		uint8_t msgIdx = 2;
		IncomingMsgFormat_t rsp;
		if (rpcLen < 17) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.GroupId = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.ClusterId = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.SrcEndpoint = rpcBuff[msgIdx++];
		rsp.DstEndpoint = rpcBuff[msgIdx++];
		rsp.WasVroadcast = rpcBuff[msgIdx++];
		rsp.LinkQuality = rpcBuff[msgIdx++];
		rsp.SecurityUse = rpcBuff[msgIdx++];
		rsp.TimeStamp = 0;
		uint8_t i;
		for (i = 0; i < 4; i++)
			rsp.TimeStamp |= ((uint32_t) rpcBuff[msgIdx++]) << (i * 8);
		rsp.TransSeqNum = rpcBuff[msgIdx++];
		rsp.Len = rpcBuff[msgIdx++];
		if (rpcLen > 17) {
			uint32_t i;
			for (i = 0; i < rsp.Len; i++) {
				rsp.Data[i] = rpcBuff[msgIdx++];
			}
		}
		mtAfCbs.pfnAfIncomingMsg(&rsp);
	}
}

static void processIncomingMsgExt(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtAfCbs.pfnAfIncomingMsgExt) {
		uint8_t msgIdx = 2;
		IncomingMsgExtFormat_t rsp;
		if (rpcLen < 27) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.GroupId = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.ClusterId = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.SrcAddrMode = rpcBuff[msgIdx++];
		rsp.SrcAddr = 0;
		uint8_t i;
		for (i = 0; i < 8; i++) {
			if ((rsp.SrcAddrMode == 2 && i < 2) || rsp.SrcAddrMode == 3) {
				rsp.SrcAddr |= ((uint64_t) rpcBuff[msgIdx]) << (i * 8);
			}
			msgIdx++;
		}

		rsp.SrcEndpoint = rpcBuff[msgIdx++];
		rsp.SrcPanId = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.DstEndpoint = rpcBuff[msgIdx++];
		rsp.WasVroadcast = rpcBuff[msgIdx++];
		rsp.LinkQuality = rpcBuff[msgIdx++];
		rsp.SecurityUse = rpcBuff[msgIdx++];
		rsp.TimeStamp = 0;
		for (i = 0; i < 4; i++)
			rsp.TimeStamp |= ((uint32_t) rpcBuff[msgIdx++]) << (i * 8);
		rsp.TransSeqNum = rpcBuff[msgIdx++];
		rsp.Len = rpcBuff[msgIdx++];
		uint32_t ind;
		for (ind = 0; ind < rsp.Len; ind++) {
			rsp.Data[ind] = rpcBuff[msgIdx++];
		}

		mtAfCbs.pfnAfIncomingMsgExt(&rsp);
	}
}

uint8_t afDataRetrieve(DataRetrieveFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 7;
	uint8_t cmd[cmdLen];

	memcpy((cmd + cmInd), req->TimeStamp, 4);
	cmInd += 4;
	cmd[cmInd++] = (uint8_t) (req->Index & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Index >> 8) & 0xFF);
	cmd[cmInd++] = req->Length;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_AF),
	MT_AF_DATA_RETRIEVE, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

static void processDataRetrieveSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtAfCbs.pfnAfDataRetrieveSrsp) {
		uint8_t msgIdx = 2;
		DataRetrieveSrspFormat_t rsp;
		if (rpcLen < 2) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];
		rsp.Length = rpcBuff[msgIdx++];
		if (rpcLen > 2) {
			uint32_t i;
			for (i = 0; i < rsp.Length; i++) {
				rsp.Data[i] = rpcBuff[msgIdx++];
			}
		}
		mtAfCbs.pfnAfDataRetrieveSrsp(&rsp);
	}
}

uint8_t afApsfConfigSet(ApsfConfigSetFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 3;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->Endpoint;
	cmd[cmInd++] = req->FrameDelay;
	cmd[cmInd++] = req->WindowSize;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_AF),
	MT_AF_APSF_CONFIG_SET, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

static void processReflectError(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtAfCbs.pfnAfReflectError) {
		uint8_t msgIdx = 2;
		ReflectErrorFormat_t rsp;
		if (rpcLen < 6) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];
		rsp.Endpoint = rpcBuff[msgIdx++];
		rsp.TransId = rpcBuff[msgIdx++];
		rsp.DstAddrMode = rpcBuff[msgIdx++];
		rsp.DstAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		mtAfCbs.pfnAfReflectError(&rsp);
	}
}

/*********************************************************************
 * @fn      afRegisterCallbacks
 *
 * @brief
 *
 * @param
 *
 * @return
 */
void afRegisterCallbacks(mtAfCb_t cbs) {
	memcpy(&mtAfCbs, &cbs, sizeof(mtAfCb_t));
}

/*************************************************************************************************
 * @fn      afProcess()
 *
 * @brief   read and process the RPC Af message from the ZB SoC
 *
 * @param   rpcLen has the size of the frame: cmd0 + cmd1 + payload + FCS
 *
 * @return
 *************************************************************************************************/
void afProcess(uint8_t *rpcBuff, uint8_t rpcLen) {
	dbg_print(PRINT_LEVEL_VERBOSE, "afProcess: processing CMD0:%x, CMD1:%x\n", rpcBuff[0], rpcBuff[1]);

	//process the synchronous SRSP from SREQ
	if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP) {
		processSrsp(rpcBuff, rpcLen);
	}
	else {
		//Read CMD1 and processes the specific SREQ
		switch (rpcBuff[1]) {
			case MT_AF_DATA_CONFIRM:
				dbg_print(PRINT_LEVEL_VERBOSE, "afProcess: MT_AF_DATA_CONFIRM\n");
				processDataConfirm(rpcBuff, rpcLen);
				break;
			case MT_AF_INCOMING_MSG:
				dbg_print(PRINT_LEVEL_VERBOSE, "afProcess: MT_AF_INCOMING_MSG\n");
				processIncomingMsg(rpcBuff, rpcLen);
				break;
			case MT_AF_INCOMING_MSG_EXT:
				dbg_print(PRINT_LEVEL_VERBOSE, "afProcess: MT_AF_INCOMING_MSG_EXT\n");
				processIncomingMsgExt(rpcBuff, rpcLen);
				break;
			case MT_AF_REFLECT_ERROR:
				dbg_print(PRINT_LEVEL_VERBOSE, "afProcess: MT_AF_REFLECT_ERROR\n");
				processReflectError(rpcBuff, rpcLen);
				break;
			default:
				dbg_print(PRINT_LEVEL_WARNING, "processRpcAf: CMD0:%x, CMD1:%x, not handled\n", rpcBuff[0], rpcBuff[1]);
				break;
		}
	}
}

/*********************************************************************
 * @fn      processSrsp
 *
 * @brief  Generic function for processing the SRSP and copying it to
 *         local buffer for SREQ function to deal with
 *
 * @param
 *
 * @return
 */
static void processSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	//copies sresp to local buffer
	memcpy(srspRpcBuff, rpcBuff, rpcLen);
	//srspRpcLen = rpcLen;
	switch (rpcBuff[1]) {
		case MT_AF_DATA_RETRIEVE:
			dbg_print(PRINT_LEVEL_VERBOSE, "afProcess: MT_AF_DATA_RETRIEVE\n");
			processDataRetrieveSrsp(rpcBuff, rpcLen);
			break;
		default:
			dbg_print(PRINT_LEVEL_INFO, "processSrsp: unsupported message [%x:%x]\n", rpcBuff[0], rpcBuff[1]);
			break;
	}

}

