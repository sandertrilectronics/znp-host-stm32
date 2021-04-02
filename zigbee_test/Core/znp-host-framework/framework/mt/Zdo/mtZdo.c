/*
 * mtZdo.c
 *
 * This module contains the API for the MT ZDO Interface
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

#include "mtZdo.h"
#include "mtSys.h"
#include "mtParser.h"
#include "rpc.h"
#include "dbgPrint.h"

/*********************************************************************
 * MACROS
 */
#define STARTDELAY 0

/*********************************************************************
 * LOCAL VARIABLES
 */
static mtZdoCb_t mtZdoCbs;
extern uint8_t srspRpcBuff[RPC_MAX_LEN];
extern uint8_t srspRpcLen;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void processSrsp(uint8_t *rpcBuff, uint8_t rpcLen);
static void processStateChange(uint8_t *rpcBuff, uint8_t rpcLen);
static void processNwkAddrRsp(uint8_t *rpcBuff, uint8_t rpcLen);

/*********************************************************************
 * @fn      processStateChange
 *
 * @brief  receives and decodes the ZDO State Change Ind msg
 *
 * @param   uint8_t *rpcBuff
 *
 * @return  none
 */
static void processStateChange(uint8_t *rpcBuff, uint8_t rpcLen)
{

	uint8_t zdoState = rpcBuff[2];
	//passes the state to the callback function
	if (mtZdoCbs.pfnmtZdoStateChangeInd)
	{
		mtZdoCbs.pfnmtZdoStateChangeInd(zdoState);
	}
}

/*********************************************************************
 * @fn      zdoNwkAddrReq
 *
 * @brief   Send ZDO_NWK_ADDR_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoNwkAddrReq(NwkAddrReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 10;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		memcpy((cmd + cmInd), req->IEEEAddress, 8);
		cmInd += 8;
		cmd[cmInd++] = req->ReqType;
		cmd[cmInd++] = req->StartIndex;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_NWK_ADDR_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoIeeeAddrReq
 *
 * @brief   Send ZDO_IEEE_ADDR_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoIeeeAddrReq(IeeeAddrReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 4;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->ShortAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->ShortAddr >> 8) & 0xFF);
		cmd[cmInd++] = req->ReqType;
		cmd[cmInd++] = req->StartIndex;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_IEEE_ADDR_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoNodeDescReq
 *
 * @brief   Send ZDO_NODE_DESC_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoNodeDescReq(NodeDescReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 4;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = (uint8_t)(req->NwkAddrOfInterest & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->NwkAddrOfInterest >> 8) & 0xFF);

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_NODE_DESC_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoPowerDescReq
 *
 * @brief   Send ZDO_POWER_DESC_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoPowerDescReq(PowerDescReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 4;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = (uint8_t)(req->NwkAddrOfInterest & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->NwkAddrOfInterest >> 8) & 0xFF);

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_POWER_DESC_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoSimpleDescReq
 *
 * @brief   Send ZDO_SIMPLE_DESC_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoSimpleDescReq(SimpleDescReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 5;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = (uint8_t)(req->NwkAddrOfInterest & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->NwkAddrOfInterest >> 8) & 0xFF);
		cmd[cmInd++] = req->Endpoint;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_SIMPLE_DESC_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoActiveEpReq
 *
 * @brief   Send ZDO_ACTIVE_EP_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoActiveEpReq(ActiveEpReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 4;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = (uint8_t)(req->NwkAddrOfInterest & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->NwkAddrOfInterest >> 8) & 0xFF);

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_ACTIVE_EP_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMatchDescReq
 *
 * @brief   Send ZDO_MATCH_DESC_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMatchDescReq(MatchDescReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 8 + (req->NumInClusters * 2) + (req->NumOutClusters * 2);
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		int idx;

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = (uint8_t)(req->NwkAddrOfInterest & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->NwkAddrOfInterest >> 8) & 0xFF);
		cmd[cmInd++] = (uint8_t)(req->ProfileID & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->ProfileID >> 8) & 0xFF);
		cmd[cmInd++] = req->NumInClusters;
		for (idx = 0; idx < req->NumInClusters; idx++)
		{
			cmd[cmInd++] = (uint8_t)(req->InClusterList[idx] & 0xFF);
			cmd[cmInd++] = (uint8_t)((req->InClusterList[idx] >> 8) & 0xFF);
		}
		cmd[cmInd++] = req->NumOutClusters;
		for (idx = 0; idx < req->NumOutClusters; idx++)
		{
			cmd[cmInd++] = (uint8_t)(req->OutClusterList[idx] & 0xFF);
			cmd[cmInd++] = (uint8_t)((req->OutClusterList[idx] >> 8) & 0xFF);
		}

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MATCH_DESC_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoComplexDescReq
 *
 * @brief   Send ZDO_COMPLEX_DESC_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoComplexDescReq(ComplexDescReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 4;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = (uint8_t)(req->NwkAddrOfInterest & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->NwkAddrOfInterest >> 8) & 0xFF);

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_COMPLEX_DESC_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoUserDescReq
 *
 * @brief   Send ZDO_USER_DESC_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoUserDescReq(UserDescReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 4;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = (uint8_t)(req->NwkAddrOfInterest & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->NwkAddrOfInterest >> 8) & 0xFF);

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_USER_DESC_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoDeviceAnnce
 *
 * @brief   Send ZDO_DEVICE_ANNCE_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoDeviceAnnce(DeviceAnnceFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 11;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->NWKAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->NWKAddr >> 8) & 0xFF);
		memcpy((cmd + cmInd), req->IEEEAddr, 8);
		cmInd += 8;
		cmd[cmInd++] = req->Capabilities;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_DEVICE_ANNCE, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoUserDescSet
 *
 * @brief   Send ZDO_USER_DESC_SET to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoUserDescSet(UserDescSetFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 5 + req->Len;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		int idx;

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = (uint8_t)(req->NwkAddrOfInterest & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->NwkAddrOfInterest >> 8) & 0xFF);
		cmd[cmInd++] = req->Len;
		for (idx = 0; idx < req->Len; idx++)
		{
			cmd[cmInd++] = req->UserDescriptor[idx];
		}

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_USER_DESC_SET, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoServerDiscReq
 *
 * @brief   Send ZDO_SERVER_DISC_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoServerDiscReq(ServerDiscReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 2;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->ServerMask & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->ServerMask >> 8) & 0xFF);

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_SERVER_DISC_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoEndDeviceBindReq
 *
 * @brief   Send ZDO_END_DEVICE_BIND_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoEndDeviceBindReq(EndDeviceBindReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 17 + (req->NumInClusters * 2) + (req->NumOutClusters * 2);
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		int idx;

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = (uint8_t)(req->LocalCoordinator & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->LocalCoordinator >> 8) & 0xFF);
		memcpy((cmd + cmInd), req->CoordinatorIEEE, 8);
		cmInd += 8;
		cmd[cmInd++] = req->EndPoint;
		cmd[cmInd++] = (uint8_t)(req->ProfileID & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->ProfileID >> 8) & 0xFF);
		cmd[cmInd++] = req->NumInClusters;
		for (idx = 0; idx < req->NumInClusters; idx++)
		{
			cmd[cmInd++] = (uint8_t)(req->InClusterList[idx] & 0xFF);
			cmd[cmInd++] = (uint8_t)((req->InClusterList[idx] >> 8) & 0xFF);
		}
		cmd[cmInd++] = req->NumOutClusters;
		for (idx = 0; idx < req->NumOutClusters; idx++)
		{
			cmd[cmInd++] = (uint8_t)(req->OutClusterList[idx] & 0xFF);
			cmd[cmInd++] = (uint8_t)((req->OutClusterList[idx] >> 8) & 0xFF);
		}

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_END_DEVICE_BIND_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoBindReq
 *
 * @brief   Send ZDO__BIND_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoBindReq(BindReqFormat_t *req)
{
	uint8_t status;
	uint8_t addrmd = (req->DstAddrMode == 3 ? 8 : 2);
	uint8_t cmInd = 0;
	uint8_t endP = (req->DstAddrMode == 3 ? 1 : 0);
	uint32_t cmdLen = 14 + addrmd + endP;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		memcpy((cmd + cmInd), req->SrcAddress, 8);
		cmInd += 8;
		cmd[cmInd++] = req->SrcEndpoint;
		cmd[cmInd++] = (uint8_t)(req->ClusterID & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->ClusterID >> 8) & 0xFF);
		cmd[cmInd++] = req->DstAddrMode;
		memcpy((cmd + cmInd), req->DstAddress, addrmd);
		cmInd += addrmd;
		if (endP)
			cmd[cmInd++] = req->DstEndpoint;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_BIND_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoUnbindReq
 *
 * @brief   Send ZDO_UNBIND_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoUnbindReq(UnbindReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint8_t addrmd = (req->DstAddrMode == 3 ? 8 : 2);
	uint8_t endP = (req->DstAddrMode == 3 ? 1 : 0);
	uint32_t cmdLen = 16;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		memcpy((cmd + cmInd), req->SrcAddress, 8);
		cmInd += 8;
		cmd[cmInd++] = req->SrcEndpoint;
		cmd[cmInd++] = (uint8_t)(req->ClusterID & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->ClusterID >> 8) & 0xFF);
		cmd[cmInd++] = req->DstAddrMode;
		memcpy((cmd + cmInd), req->DstAddress, addrmd);
		cmInd += addrmd;
		if (endP)
			cmd[cmInd++] = req->DstEndpoint;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_UNBIND_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMgmtNwkDiscReq
 *
 * @brief   Send ZDO_MGMT_NWK_DISC_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMgmtNwkDiscReq(MgmtNwkDiscReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 8;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		memcpy((cmd + cmInd), req->ScanChannels, 4);
		cmInd += 4;
		cmd[cmInd++] = req->ScanDuration;
		cmd[cmInd++] = req->StartIndex;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MGMT_NWK_DISC_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMgmtLqiReq
 *
 * @brief   Send ZDO_MGMT_LQI_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMgmtLqiReq(MgmtLqiReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 3;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = req->StartIndex;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MGMT_LQI_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMgmtRtgReq
 *
 * @brief   Send ZDO_MGMT_RTG_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMgmtRtgReq(MgmtRtgReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 3;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = req->StartIndex;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MGMT_RTG_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMgmtBindReq
 *
 * @brief   Send ZDO_MGMT_BIND_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMgmtBindReq(MgmtBindReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 3;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = req->StartIndex;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MGMT_BIND_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMgmtLeaveReq
 *
 * @brief   Send ZDO_MGMT_LEAVE_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMgmtLeaveReq(MgmtLeaveReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 11;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		memcpy((cmd + cmInd), req->DeviceAddr, 8);
		cmInd += 8;
		cmd[cmInd++] = req->RemoveChildre_Rejoin;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MGMT_LEAVE_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMgmtDirectJoinReq
 *
 * @brief   Send ZDO_MGMT_DIRECT_JOIN_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMgmtDirectJoinReq(MgmtDirectJoinReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 11;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		memcpy((cmd + cmInd), req->DeviceAddr, 8);
		cmInd += 8;
		cmd[cmInd++] = req->CapInfo;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MGMT_DIRECT_JOIN_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMgmtPermitJoinReq
 *
 * @brief   Send ZDO_MGMT_PERMIT_JOIN_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMgmtPermitJoinReq(MgmtPermitJoinReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 5;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = req->AddrMode;
		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = req->Duration;
		cmd[cmInd++] = req->TCSignificance;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MGMT_PERMIT_JOIN_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMgmtNwkUpdateReq
 *
 * @brief   Send ZDO_MGMT_NWK_UPDATE_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMgmtNwkUpdateReq(MgmtNwkUpdateReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 11;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->DstAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->DstAddr >> 8) & 0xFF);
		cmd[cmInd++] = req->DstAddrMode;
		memcpy((cmd + cmInd), req->ChannelMask, 4);
		cmInd += 4;
		cmd[cmInd++] = req->ScanDuration;
		cmd[cmInd++] = req->ScanCount;
		cmd[cmInd++] = (uint8_t)(req->NwkManagerAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->NwkManagerAddr >> 8) & 0xFF);

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MGMT_NWK_UPDATE_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoStartupFromApp
 *
 * @brief   Send ZDO_STARTUP_FROM_APP_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoStartupFromApp(StartupFromAppFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 2;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = LO_UINT16(req->StartDelay);
		cmd[cmInd++] = HI_UINT16(req->StartDelay);
		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_STARTUP_FROM_APP, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoAutoFindDestination
 *
 * @brief   Send ZDO_AUTO_FIND_DESTINATION_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoAutoFindDestination(AutoFindDestinationFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 1;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = req->Endpoint;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_AUTO_FIND_DESTINATION, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoSetLinkKey
 *
 * @brief   Send ZDO_SET_LINK_KEY to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoSetLinkKey(SetLinkKeyFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 26;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->ShortAddr & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->ShortAddr >> 8) & 0xFF);
		memcpy((cmd + cmInd), req->IEEEaddr, 8);
		cmInd += 8;
		memcpy((cmd + cmInd), req->LinkKeyData, 16);
		cmInd += 16;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_SET_LINK_KEY, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoRemoveLinkKey
 *
 * @brief   Send ZDO_REMOVE_LINK_KEY to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoRemoveLinkKey(RemoveLinkKeyFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 8;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		memcpy((cmd + cmInd), req->IEEEaddr, 8);
		cmInd += 8;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_REMOVE_LINK_KEY, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoGetLinkKey
 *
 * @brief   Send ZDO_GET_LINK_KEY to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoGetLinkKey(GetLinkKeyFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 8;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		memcpy((cmd + cmInd), req->IEEEaddr, 8);
		cmInd += 8;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_GET_LINK_KEY, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoNwkDiscoveryReq
 *
 * @brief   Send ZDO_NWK_DISC_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoNwkDiscoveryReq(NwkDiscoveryReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 5;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		memcpy((cmd + cmInd), req->ScanChannels, 4);
		cmInd += 4;
		cmd[cmInd++] = req->ScanDuration;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_NWK_DISCOVERY_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoJoinReq
 *
 * @brief   Send ZDO_JOIN_REQ to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoJoinReq(JoinReqFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 15;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = req->LogicalChannel;
		cmd[cmInd++] = (uint8_t)(req->PanID & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->PanID >> 8) & 0xFF);
		memcpy((cmd + cmInd), req->ExtendedPanID, 8);
		cmInd += 8;
		cmd[cmInd++] = (uint8_t)(req->ChosenParent & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->ChosenParent >> 8) & 0xFF);
		cmd[cmInd++] = req->ParentDepth;
		cmd[cmInd++] = req->StackProfile;

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_JOIN_REQ, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMsgCbRegister
 *
 * @brief   Send ZDO_MSG_CB_REGISTER to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMsgCbRegister(MsgCbRegisterFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 2;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->ClusterID & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->ClusterID >> 8) & 0xFF);

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MSG_CB_REGISTER, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      zdoMsgCbRemove
 *
 * @brief   Send ZDO_MSG_CB_REMOVE to ZNP
 *
 * @param    req - Pointer to outgoing command structure
 *
 * @return   status
 */
uint8_t zdoMsgCbRemove(MsgCbRemoveFormat_t *req)
{
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 2;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[cmInd++] = (uint8_t)(req->ClusterID & 0xFF);
		cmd[cmInd++] = (uint8_t)((req->ClusterID >> 8) & 0xFF);

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_MSG_CB_REMOVE, cmd, cmdLen);

		if (status == MT_RPC_SUCCESS)
		{
			rpcWaitMqClientMsg(50);
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*********************************************************************
 * @fn      processGetLinkKey
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processGetLinkKey(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoGetLinkKey)
	{
		uint8_t msgIdx = 2;
		GetLinkKeySrspFormat_t rsp;
		if (rpcLen < 25)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];
		rsp.IEEEAddr = 0;
		uint8_t i;
		for (i = 0; i < 8; i++)
			rsp.IEEEAddr |= ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);
		memcpy(rsp.LinkKeyData, &rpcBuff[msgIdx], 16);
		msgIdx += 16;

		mtZdoCbs.pfnZdoGetLinkKey(&rsp);
	}
}

/*********************************************************************
 * @fn      processNwkAddrRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processNwkAddrRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoNwkAddrRsp)
	{
		uint8_t msgIdx = 2;
		NwkAddrRspFormat_t rsp;
		if (rpcLen < 13)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];
		rsp.IEEEAddr = 0;
		uint8_t i;
		for (i = 0; i < 8; i++)
			rsp.IEEEAddr |= ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.StartIndex = rpcBuff[msgIdx++];
		rsp.NumAssocDev = rpcBuff[msgIdx++];
		if (rpcLen > 13)
		{
			uint32_t i;
			for (i = 0; i < rsp.NumAssocDev; i++)
			{
				rsp.AssocDevList[i] = BUILD_UINT16(rpcBuff[msgIdx],
				        rpcBuff[msgIdx + 1]);
				msgIdx += 2;
			}
		}
		mtZdoCbs.pfnZdoNwkAddrRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processIeeeAddrRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processIeeeAddrRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoIeeeAddrRsp)
	{
		uint8_t msgIdx = 2;
		IeeeAddrRspFormat_t rsp;
		if (rpcLen < 13)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];
		rsp.IEEEAddr = 0;
		uint8_t i;
		for (i = 0; i < 8; i++)
			rsp.IEEEAddr |= ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.StartIndex = rpcBuff[msgIdx++];
		rsp.NumAssocDev = rpcBuff[msgIdx++];
		rsp.StartIndex = (rsp.NumAssocDev == 0 ? 0 : rsp.StartIndex);
		if (rpcLen > 13)
		{
			uint32_t i;
			for (i = 0; i < rsp.NumAssocDev; i++)
			{
				rsp.AssocDevList[i] = BUILD_UINT16(rpcBuff[msgIdx],
				        rpcBuff[msgIdx + 1]);
				msgIdx += 2;
			}
		}
		mtZdoCbs.pfnZdoIeeeAddrRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processNodeDescRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processNodeDescRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoNodeDescRsp)
	{
		uint8_t msgIdx = 2;
		NodeDescRspFormat_t rsp;
		if (rpcLen < 18)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.LoTy_ComDescAv_UsrDesAv = rpcBuff[msgIdx++];
		rsp.APSFlg_FrqBnd = rpcBuff[msgIdx++];
		rsp.MACCapFlg = rpcBuff[msgIdx++];
		rsp.ManufacturerCode = BUILD_UINT16(rpcBuff[msgIdx],
		        rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.MaxBufferSize = rpcBuff[msgIdx++];
		rsp.MaxTransferSize = BUILD_UINT16(rpcBuff[msgIdx],
		        rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.ServerMask = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.MaxOutTransferSize = BUILD_UINT16(rpcBuff[msgIdx],
		        rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.DescriptorCapabilities = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoNodeDescRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processPowerDescRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processPowerDescRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoPowerDescRsp)
	{
		uint8_t msgIdx = 2;
		PowerDescRspFormat_t rsp;
		if (rpcLen < 7)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.CurrntPwrMode_AvalPwrSrcs = rpcBuff[msgIdx++];
		rsp.CurrntPwrSrc_CurrntPwrSrcLvl = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoPowerDescRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processSimpleDescRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processSimpleDescRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoSimpleDescRsp)
	{
		uint8_t msgIdx = 2;
		SimpleDescRspFormat_t rsp;
		if (rpcLen < 6)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Len = rpcBuff[msgIdx++];
		if (rpcLen > 6)
		{
			rsp.Endpoint = rpcBuff[msgIdx++];
			rsp.ProfileID = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
			msgIdx += 2;
			rsp.DeviceID = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
			msgIdx += 2;
			rsp.DeviceVersion = rpcBuff[msgIdx++];
			rsp.NumInClusters = rpcBuff[msgIdx++];
			uint32_t i;
			for (i = 0; i < rsp.NumInClusters; i++)
			{
				rsp.InClusterList[i] = BUILD_UINT16(rpcBuff[msgIdx],
				        rpcBuff[msgIdx + 1]);
				msgIdx += 2;
			}
			rsp.NumOutClusters = rpcBuff[msgIdx++];
			for (i = 0; i < rsp.NumOutClusters; i++)
			{
				rsp.OutClusterList[i] = BUILD_UINT16(rpcBuff[msgIdx],
				        rpcBuff[msgIdx + 1]);
				msgIdx += 2;
			}
		}
		mtZdoCbs.pfnZdoSimpleDescRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processActiveEpRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processActiveEpRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoActiveEpRsp)
	{
		uint8_t msgIdx = 2;
		ActiveEpRspFormat_t rsp;
		if (rpcLen < 6)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.ActiveEPCount = rpcBuff[msgIdx++];
		if (rpcLen > 6)
		{
			uint32_t i;
			for (i = 0; i < rsp.ActiveEPCount; i++)
			{
				rsp.ActiveEPList[i] = rpcBuff[msgIdx++];
			}
		}
		mtZdoCbs.pfnZdoActiveEpRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processMatchDescRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processMatchDescRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoMatchDescRsp)
	{
		uint8_t msgIdx = 2;
		MatchDescRspFormat_t rsp;
		if (rpcLen < 6)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.MatchLength = rpcBuff[msgIdx++];
		if (rpcLen > 6)
		{
			uint32_t i;
			for (i = 0; i < rsp.MatchLength; i++)
			{
				rsp.MatchList[i] = rpcBuff[msgIdx++];
			}
		}
		mtZdoCbs.pfnZdoMatchDescRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processComplexDescRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processComplexDescRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoComplexDescRsp)
	{
		uint8_t msgIdx = 2;
		ComplexDescRspFormat_t rsp;
		if (rpcLen < 6)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.ComplexLength = rpcBuff[msgIdx++];
		if (rpcLen > 6)
		{
			uint32_t i;
			for (i = 0; i < rsp.ComplexLength; i++)
			{
				rsp.ComplexList[i] = rpcBuff[msgIdx++];
			}
		}
		mtZdoCbs.pfnZdoComplexDescRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processUserDescRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processUserDescRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoUserDescRsp)
	{
		uint8_t msgIdx = 2;
		UserDescRspFormat_t rsp;
		if (rpcLen < 6)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Len = rpcBuff[msgIdx++];
		if (rpcLen > 6)
		{
			uint32_t i;
			for (i = 0; i < rsp.Len; i++)
			{
				rsp.CUserDescriptor[i] = rpcBuff[msgIdx++];
			}
		}
		mtZdoCbs.pfnZdoUserDescRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processUserDescConf
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processUserDescConf(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoUserDescConf)
	{
		uint8_t msgIdx = 2;
		UserDescConfFormat_t rsp;
		if (rpcLen < 5)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		mtZdoCbs.pfnZdoUserDescConf(&rsp);
	}
}

/*********************************************************************
 * @fn      processServerDiscRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processServerDiscRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoServerDiscRsp)
	{
		uint8_t msgIdx = 2;
		ServerDiscRspFormat_t rsp;
		if (rpcLen < 5)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.ServerMask = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		mtZdoCbs.pfnZdoServerDiscRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processEndDeviceBindRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processEndDeviceBindRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoEndDeviceBindRsp)
	{
		uint8_t msgIdx = 2;
		EndDeviceBindRspFormat_t rsp;
		if (rpcLen < 3)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoEndDeviceBindRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processBindRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processBindRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoBindRsp)
	{
		uint8_t msgIdx = 2;
		BindRspFormat_t rsp;
		if (rpcLen < 3)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoBindRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processUnbindRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processUnbindRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoUnbindRsp)
	{
		uint8_t msgIdx = 2;
		UnbindRspFormat_t rsp;
		if (rpcLen < 3)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoUnbindRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processMgmtNwkDiscRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processMgmtNwkDiscRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoMgmtNwkDiscRsp)
	{
		uint8_t msgIdx = 2;
		MgmtNwkDiscRspFormat_t rsp;
		if (rpcLen < 6)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.NetworkCount = rpcBuff[msgIdx++];
		rsp.StartIndex = rpcBuff[msgIdx++];
		rsp.NetworkListCount = rpcBuff[msgIdx++];
		if (rpcLen > 6)
		{
			uint32_t i;
			for (i = 0; i < rsp.NetworkListCount; i++)
			{
				rsp.NetworkList[i].PanID = 0;
				uint8_t ind;
				for (ind = 0; ind < 8; ind++)
					rsp.NetworkList[i].PanID |= ((uint64_t) rpcBuff[msgIdx++])
					        << (ind * 8);
				rsp.NetworkList[i].LogicalChannel = rpcBuff[msgIdx++];
				rsp.NetworkList[i].StackProf_ZigVer = rpcBuff[msgIdx++];
				rsp.NetworkList[i].BeacOrd_SupFramOrd = rpcBuff[msgIdx++];
				rsp.NetworkList[i].PermitJoin = rpcBuff[msgIdx++];
			}
		}
		mtZdoCbs.pfnZdoMgmtNwkDiscRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processMgmtLqiRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processMgmtLqiRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoMgmtLqiRsp)
	{
		uint8_t msgIdx = 2;
		MgmtLqiRspFormat_t rsp;
		if (rpcLen < 6)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.NeighborTableEntries = rpcBuff[msgIdx++];
		rsp.StartIndex = rpcBuff[msgIdx++];
		rsp.NeighborLqiListCount = rpcBuff[msgIdx++];
		if (rpcLen > 6)
		{
			uint32_t i;
			for (i = 0; i < rsp.NeighborLqiListCount; i++)
			{

				rsp.NeighborLqiList[i].ExtendedPanID = 0;
				uint8_t ind;
				for (ind = 0; ind < 8; ind++)
					rsp.NeighborLqiList[i].ExtendedPanID |=
					        ((uint64_t) rpcBuff[msgIdx++]) << (ind * 8);
				rsp.NeighborLqiList[i].ExtendedAddress = 0;
				for (ind = 0; ind < 8; ind++)
					rsp.NeighborLqiList[i].ExtendedAddress |=
					        ((uint64_t) rpcBuff[msgIdx++]) << (ind * 8);
				rsp.NeighborLqiList[i].NetworkAddress = BUILD_UINT16(
				        rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
				msgIdx += 2;
				rsp.NeighborLqiList[i].DevTyp_RxOnWhenIdle_Relat =
				        rpcBuff[msgIdx++];
				rsp.NeighborLqiList[i].PermitJoining = rpcBuff[msgIdx++];
				rsp.NeighborLqiList[i].Depth = rpcBuff[msgIdx++];
				rsp.NeighborLqiList[i].LQI = rpcBuff[msgIdx++];

			}
		}
		MgmtLqiRspFormat_t *copyy = &rsp;
		mtZdoCbs.pfnZdoMgmtLqiRsp(copyy);
	}
}

/*********************************************************************
 * @fn      processMgmtRtgRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processMgmtRtgRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoMgmtRtgRsp)
	{
		uint8_t msgIdx = 2;
		MgmtRtgRspFormat_t rsp;
		if (rpcLen < 6)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.RoutingTableEntries = rpcBuff[msgIdx++];
		rsp.StartIndex = rpcBuff[msgIdx++];
		rsp.RoutingTableListCount = rpcBuff[msgIdx++];
		if (rpcLen > 6)
		{
			uint32_t i;
			for (i = 0; i < rsp.RoutingTableListCount; i++)
			{
				rsp.RoutingTableList[i].DstAddr = BUILD_UINT16(rpcBuff[msgIdx],
				        rpcBuff[msgIdx + 1]);
				msgIdx += 2;
				rsp.RoutingTableList[i].Status = rpcBuff[msgIdx++];
				rsp.RoutingTableList[i].NextHop = BUILD_UINT16(rpcBuff[msgIdx],
				        rpcBuff[msgIdx + 1]);
				msgIdx += 2;
			}
		}
		mtZdoCbs.pfnZdoMgmtRtgRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processMgmtBindRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processMgmtBindRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoMgmtBindRsp)
	{
		uint8_t msgIdx = 2;
		MgmtBindRspFormat_t rsp;
		if (rpcLen < 6)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];
		rsp.BindingTableEntries = rpcBuff[msgIdx++];
		rsp.StartIndex = rpcBuff[msgIdx++];
		rsp.BindingTableListCount = rpcBuff[msgIdx++];
		if (rpcLen > 6)
		{
			uint32_t i;
			for (i = 0; i < rsp.BindingTableListCount; i++)
			{
				rsp.BindingTableList[i].SrcIEEEAddr = 0;
				uint8_t i;
				for (i = 0; i < 8; i++)
					rsp.BindingTableList[i].SrcIEEEAddr |=
					        ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);
				rsp.BindingTableList[i].SrcEndpoint = rpcBuff[msgIdx++];
				rsp.BindingTableList[i].ClusterID = rpcBuff[msgIdx++];
				rsp.BindingTableList[i].DstAddrMode = rpcBuff[msgIdx++];
				rsp.BindingTableList[i].DstIEEEAddr = 0;
				for (i = 0; i < 8; i++)
					rsp.BindingTableList[i].DstIEEEAddr |=
					        ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);
				rsp.BindingTableList[i].DstEndpoint = rpcBuff[msgIdx++];
			}
		}
		mtZdoCbs.pfnZdoMgmtBindRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processMgmtLeaveRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processMgmtLeaveRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoMgmtLeaveRsp)
	{
		uint8_t msgIdx = 2;
		MgmtLeaveRspFormat_t rsp;
		if (rpcLen < 3)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoMgmtLeaveRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processMgmtDirectJoinRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processMgmtDirectJoinRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoMgmtDirectJoinRsp)
	{
		uint8_t msgIdx = 2;
		MgmtDirectJoinRspFormat_t rsp;
		if (rpcLen < 3)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoMgmtDirectJoinRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processMgmtPermitJoinRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processMgmtPermitJoinRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoMgmtPermitJoinRsp)
	{
		uint8_t msgIdx = 2;
		MgmtPermitJoinRspFormat_t rsp;
		if (rpcLen < 3)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoMgmtPermitJoinRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processEndDeviceAnnceInd
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processEndDeviceAnnceInd(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoEndDeviceAnnceInd)
	{
		uint8_t msgIdx = 2;
		EndDeviceAnnceIndFormat_t rsp;
		if (rpcLen < 13)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.IEEEAddr = 0;
		uint8_t i;
		for (i = 0; i < 8; i++)
			rsp.IEEEAddr |= ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);
		rsp.Capabilities = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoEndDeviceAnnceInd(&rsp);
	}
}

/*********************************************************************
 * @fn      processMatchDescRspSent
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processMatchDescRspSent(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoMatchDescRspSent)
	{
		uint8_t msgIdx = 2;
		MatchDescRspSentFormat_t rsp;
		if (rpcLen < 4)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.NumInClusters = rpcBuff[msgIdx++];
		uint32_t i;
		for (i = 0; i < rsp.NumInClusters; i++)
		{
			rsp.InClusterList[i] = BUILD_UINT16(rpcBuff[msgIdx],
			        rpcBuff[msgIdx + 1]);
			msgIdx += 2;
		}
		rsp.NumOutClusters = rpcBuff[msgIdx++];
		for (i = 0; i < rsp.NumOutClusters; i++)
		{
			rsp.OutClusterList[i] = BUILD_UINT16(rpcBuff[msgIdx],
			        rpcBuff[msgIdx + 1]);
			msgIdx += 2;
		}

		mtZdoCbs.pfnZdoMatchDescRspSent(&rsp);
	}
}

/*********************************************************************
 * @fn      processStatusErrorRsp
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processStatusErrorRsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoStatusErrorRsp)
	{
		uint8_t msgIdx = 2;
		StatusErrorRspFormat_t rsp;
		if (rpcLen < 3)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoStatusErrorRsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processSrcRtgInd
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processSrcRtgInd(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoSrcRtgInd)
	{
		uint8_t msgIdx = 2;
		SrcRtgIndFormat_t rsp;
		if (rpcLen < 4)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.DstAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.RelayCount = rpcBuff[msgIdx++];
		uint32_t i;
		for (i = 0; i < rsp.RelayCount; i++)
		{
			rsp.RelayList[i] = BUILD_UINT16(rpcBuff[msgIdx],
			        rpcBuff[msgIdx + 1]);
			msgIdx += 2;
		}

		mtZdoCbs.pfnZdoSrcRtgInd(&rsp);
	}
}
/*********************************************************************
 * @fn      processBeaconNotifyInd
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processBeaconNotifyInd(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoBeaconNotifyInd)
	{
		uint8_t msgIdx = 2;
		BeaconNotifyIndFormat_t rsp;
		if (rpcLen < 1)
		{
			printf("MT_RPC_ERR_LENGTH\n");
		}
		printf("rpcLen = %d\n", rpcLen);

		rsp.BeaconCount = rpcBuff[msgIdx++];
		if (rpcLen > 1)
		{
			uint32_t i;
			for (i = 0; i < rsp.BeaconCount; i++)
			{
				rsp.BeaconList[i].SrcAddr = BUILD_UINT16(rpcBuff[msgIdx],
				        rpcBuff[msgIdx + 1]);
				msgIdx += 2;
				rsp.BeaconList[i].PanId = BUILD_UINT16(rpcBuff[msgIdx],
				        rpcBuff[msgIdx + 1]);
				msgIdx += 2;
				rsp.BeaconList[i].LogicalChannel = rpcBuff[msgIdx++];
				rsp.BeaconList[i].PermitJoining = rpcBuff[msgIdx++];
				rsp.BeaconList[i].RouterCap = rpcBuff[msgIdx++];
				rsp.BeaconList[i].DevCap = rpcBuff[msgIdx++];
				rsp.BeaconList[i].ProtocolVer = rpcBuff[msgIdx++];
				rsp.BeaconList[i].StackProf = rpcBuff[msgIdx++];
				rsp.BeaconList[i].Lqi = rpcBuff[msgIdx++];
				rsp.BeaconList[i].Depth = rpcBuff[msgIdx++];
				rsp.BeaconList[i].UpdateId = rpcBuff[msgIdx++];

				rsp.BeaconList[i].ExtendedPanId = 0;
				uint8_t ind;
				for (ind = 0; ind < 8; ind++)
					rsp.BeaconList[i].ExtendedPanId |=
					        ((uint64_t) rpcBuff[msgIdx++]) << (ind * 8);

			}
		}
		mtZdoCbs.pfnZdoBeaconNotifyInd(&rsp);
	}
}

/*********************************************************************
 * @fn      processJoinCnf
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processJoinCnf(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoJoinCnf)
	{
		uint8_t msgIdx = 2;
		JoinCnfFormat_t rsp;
		if (rpcLen < 5)
		{
			printf("MT_RPC_ERR_LENGTH\n");
		}
		printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];
		rsp.DevAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.ParentAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		mtZdoCbs.pfnZdoJoinCnf(&rsp);
	}
}

/*********************************************************************
 * @fn      processNwkDiscoveryCnf
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processNwkDiscoveryCnf(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoNwkDiscoveryCnf)
	{
		uint8_t msgIdx = 2;
		NwkDiscoveryCnfFormat_t rsp;
		if (rpcLen < 1)
		{
			printf("MT_RPC_ERR_LENGTH\n");
		}
		printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoNwkDiscoveryCnf(&rsp);
	}
}
/*********************************************************************
 * @fn      processLeaveInd
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processLeaveInd(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoLeaveInd)
	{
		uint8_t msgIdx = 2;
		LeaveIndFormat_t rsp;
		if (rpcLen < 13)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		
		rsp.ExtAddr = 0;		
		uint8_t i;
		for (i = 0; i < 8; i++)
		{
			rsp.ExtAddr |= ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);
		}
		rsp.Request = rpcBuff[msgIdx++];
		rsp.Remove = rpcBuff[msgIdx++];
		rsp.Rejoin = rpcBuff[msgIdx++];

		mtZdoCbs.pfnZdoLeaveInd(&rsp);
	}
}

/*********************************************************************
 * @fn      processMsgCbIncoming
 *
 * @brief   processes incoming command from ZNP
 *
 * @param    rpcBuff - Buffer from rpc layer, contains command data
 * @param    rpcLen - Length of rpcBuff
 *
 * @return
 */
static void processMsgCbIncoming(uint8_t *rpcBuff, uint8_t rpcLen)
{
	if (mtZdoCbs.pfnZdoMsgCbIncoming)
	{
		uint8_t msgIdx = 2;
		MsgCbIncomingFormat_t rsp;
		if (rpcLen < 9)
		{
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SrcAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.WasBroadcast = rpcBuff[msgIdx++];
		rsp.ClusterID = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.SecurityUse = rpcBuff[msgIdx++];
		rsp.SeqNum = rpcBuff[msgIdx++];
		rsp.MacDstAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		rsp.Status = rpcBuff[msgIdx++];
		rsp.ExtAddr = 0;
		uint8_t i;
		for (i = 0; i < 8; i++)
		{
			rsp.ExtAddr |= ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);
		}
		rsp.NwkAddr = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		rsp.NotUsed = rpcBuff[msgIdx];
		
		
		mtZdoCbs.pfnZdoMsgCbIncoming(&rsp);
	}
}

/*********************************************************************
 * @fn      zdoInit
 *
 * @brief  Sends the ZD0_startup_from_App command to start the network
 *
 * @param   none
 *
 * @return  none
 */
uint8_t zdoInit(void)
{
	uint8_t status;
	// build the buffer
	uint32_t cmdLen = 2;
	uint8_t *cmd = malloc(cmdLen);

	if (cmd)
	{

		cmd[0] = LO_UINT16(STARTDELAY);
		cmd[1] = HI_UINT16(STARTDELAY);

		status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_ZDO),
		MT_ZDO_STARTUP_FROM_APP, cmd, cmdLen);

		//read the SREQ from the queue
		if (status == MT_RPC_SUCCESS)
		{
			//rpcSendFrame will block on the SRSP's, which will be
			//pushed to the front of the queue
			rpcWaitMqClientMsg(50);

			//set status to status of srsp
			status = srspRpcBuff[2];
		}

		free(cmd);
		return status;
	}
	else
	{
		dbg_print(PRINT_LEVEL_WARNING, "Memory for cmd was not allocated\n");
		return 1;
	}
}

/*************************************************************************************************
 * @fn      zdoProcess()
 *
 * @brief   read and process the RPC ZDO message from the ZB SoC
 *
 * @param   none
 *
 * @return  length of current Rx Buffer
 ***********************************************************************************************/
void zdoProcess(uint8_t *rpcBuff, uint8_t rpcLen)
{
	dbg_print(PRINT_LEVEL_VERBOSE, "zdoProcess: processing CMD0:%x, CMD1:%x\n",
	        rpcBuff[0], rpcBuff[1]);

	//process the synchronous SRSP from SREQ
	if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP)
	{
		processSrsp(rpcBuff, rpcLen);
	}
	else
	{
		//Read CMD1 and processes the specific SREQ
		switch (rpcBuff[1])
		{
		case MT_ZDO_STATE_CHANGE_IND:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_STATE_CHANGE_IND\n");
			processStateChange(rpcBuff, rpcLen);
			break;
		case MT_ZDO_NWK_ADDR_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE, "zdoProcess: MT_ZDO_NWK_ADDR_RSP\n");
			processNwkAddrRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_IEEE_ADDR_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_IEEE_ADDR_RSP\n");
			processIeeeAddrRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_NODE_DESC_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_NODE_DESC_RSP\n");
			processNodeDescRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_POWER_DESC_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_POWER_DESC_RSP\n");
			processPowerDescRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_SIMPLE_DESC_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_SIMPLE_DESC_RSP\n");
			processSimpleDescRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_ACTIVE_EP_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_ACTIVE_EP_RSP\n");
			processActiveEpRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_MATCH_DESC_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_MATCH_DESC_RSP\n");
			processMatchDescRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_COMPLEX_DESC_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_COMPLEX_DESC_RSP\n");
			processComplexDescRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_USER_DESC_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_USER_DESC_RSP\n");
			processUserDescRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_USER_DESC_CONF:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_USER_DESC_CONF\n");
			processUserDescConf(rpcBuff, rpcLen);
			break;
		case MT_ZDO_SERVER_DISC_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_SERVER_DISC_RSP\n");
			processServerDiscRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_END_DEVICE_BIND_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_END_DEVICE_BIND_RSP\n");
			processEndDeviceBindRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_BIND_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE, "zdoProcess: MT_ZDO_BIND_RSP\n");
			processBindRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_UNBIND_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE, "zdoProcess: MT_ZDO_UNBIND_RSP\n");
			processUnbindRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_MGMT_NWK_DISC_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_MGMT_NWK_DISC_RSP\n");
			processMgmtNwkDiscRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_MGMT_LQI_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE, "zdoProcess: MT_ZDO_MGMT_LQI_RSP\n");
			processMgmtLqiRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_MGMT_RTG_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE, "zdoProcess: MT_ZDO_MGMT_RTG_RSP\n");
			processMgmtRtgRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_MGMT_BIND_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_MGMT_BIND_RSP\n");
			processMgmtBindRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_MGMT_LEAVE_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_MGMT_LEAVE_RSP\n");
			processMgmtLeaveRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_MGMT_DIRECT_JOIN_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_MGMT_DIRECT_JOIN_RSP\n");
			processMgmtDirectJoinRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_MGMT_PERMIT_JOIN_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_MGMT_PERMIT_JOIN_RSP\n");
			processMgmtPermitJoinRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_END_DEVICE_ANNCE_IND:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_END_DEVICE_ANNCE_IND\n");
			processEndDeviceAnnceInd(rpcBuff, rpcLen);
			break;
		case MT_ZDO_MATCH_DESC_RSP_SENT:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_MATCH_DESC_RSP_SENT\n");
			processMatchDescRspSent(rpcBuff, rpcLen);
			break;
		case MT_ZDO_STATUS_ERROR_RSP:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_STATUS_ERROR_RSP\n");
			processStatusErrorRsp(rpcBuff, rpcLen);
			break;
		case MT_ZDO_SRC_RTG_IND:
			dbg_print(PRINT_LEVEL_VERBOSE, "zdoProcess: MT_ZDO_SRC_RTG_IND\n");
			processSrcRtgInd(rpcBuff, rpcLen);
			break;
		case MT_ZDO_BEACON_NOTIFY_IND:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_BEACON_NOTIFY_IND\n");
			processBeaconNotifyInd(rpcBuff, rpcLen);
			break;
		case MT_ZDO_JOIN_CNF:
			dbg_print(PRINT_LEVEL_VERBOSE, "zdoProcess: MT_ZDO_JOIN_CNF\n");
			processJoinCnf(rpcBuff, rpcLen);
			break;
		case MT_ZDO_NWK_DISCOVERY_CNF:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_NWK_DISCOVERY_CNF\n");
			processNwkDiscoveryCnf(rpcBuff, rpcLen);
			break;
		case MT_ZDO_LEAVE_IND:
			dbg_print(PRINT_LEVEL_VERBOSE, "zdoProcess: MT_ZDO_LEAVE_IND\n");
			processLeaveInd(rpcBuff, rpcLen);
			break;
		case MT_ZDO_MSG_CB_INCOMING:
			dbg_print(PRINT_LEVEL_VERBOSE,
			        "zdoProcess: MT_ZDO_MSG_CB_INCOMING\n");
			processMsgCbIncoming(rpcBuff, rpcLen);
			break;

		default:
			dbg_print(PRINT_LEVEL_WARNING,
			        "zdoProcess: CMD0:%x, CMD1:%x, not handled\n", rpcBuff[0],
			        rpcBuff[1]);
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
static void processSrsp(uint8_t *rpcBuff, uint8_t rpcLen)
{
	//copies sresp to local buffer
	memcpy(srspRpcBuff, rpcBuff, rpcLen);
	//srspRpcLen = rpcLen;
	switch (rpcBuff[1])
	{
	case MT_ZDO_GET_LINK_KEY:
		dbg_print(PRINT_LEVEL_VERBOSE, "zdoProcess: MT_ZDO_GET_LINK_KEY\n");
		processGetLinkKey(rpcBuff, rpcLen);
		break;
	default:
		dbg_print(PRINT_LEVEL_INFO, "processSrsp: unsupported message\n");
		break;
	}
}

/*********************************************************************
 * @fn      zbRegisterZdoCallbacks
 *
 * @brief
 *
 * @param
 *
 * @return
 */
void zdoRegisterCallbacks(mtZdoCb_t cbs)
{
	memcpy(&mtZdoCbs, &cbs, sizeof(mtZdoCb_t));
}

