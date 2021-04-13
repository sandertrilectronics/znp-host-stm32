/*
 * mtSapi.c
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

#include "mtSapi.h"
#include "mtSys.h"
#include "mtParser.h"
#include "rpc.h"

#include "dbgPrint.h"

/*********************************************************************
 * LOCAL VARIABLES
 */
static mtSapiCb_t mtSapiCbs;
extern uint8_t srspRpcBuff[RPC_MAX_LEN];
extern uint8_t srspRpcLen;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void processSrsp(uint8_t *rpcBuff, uint8_t rpcLen);
static void processStartCnf(uint8_t *rpcBuff, uint8_t rpcLen);
static void processBindCnf(uint8_t *rpcBuff, uint8_t rpcLen);
static void processAllowBindCnf(uint8_t *rpcBuff, uint8_t rpcLen);
static void processSendDataCnf(uint8_t *rpcBuff, uint8_t rpcLen);
static void processFindDeviceCnf(uint8_t *rpcBuff, uint8_t rpcLen);
static void processReceiveDataInd(uint8_t *rpcBuff, uint8_t rpcLen);

/*********************************************************************
 * API FUNCTIONS
 */

/******************************************************************************
 * @fn          zbSystemReset
 *
 * @brief       The zbSystemReset function reboots the ZigBee Stack.  The
 *              zbSystemReset function can be called after a call to
 *              zbWriteConfiguration to restart Z-Stack with the updated
 *              configuration.
 *
 * @param       none
 *
 * @return      none
 */
uint8_t zbSystemReset(void) {

	rpcSendFrame((MT_RPC_CMD_AREQ | MT_RPC_SYS_SAPI), MT_SAPI_SYS_RESET, NULL, 0);

	return SUCCESS;
}

/*********************************************************************
 * @fn      zbAppRegisterReq
 *
 * @brief   This command enables the application processor to register its application with the ZNP
 *           device.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t zbAppRegisterReq(AppRegisterReqFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 9 + (req->InputCommandsNum * 2) + (req->OutputCommandsNum * 2);
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = req->AppEndpoint;
	cmd[cmInd++] = (uint8_t) (req->AppProfileId & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->AppProfileId >> 8) & 0xFF);
	cmd[cmInd++] = (uint8_t) (req->DeviceId & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->DeviceId >> 8) & 0xFF);
	cmd[cmInd++] = req->DeviceVersion;
	cmd[cmInd++] = req->Unused;
	cmd[cmInd++] = req->InputCommandsNum;
	for (idx = 0; idx < req->InputCommandsNum; idx++) {
		cmd[cmInd++] = (uint8_t) (req->InputCommandsList[idx] & 0xFF);
		cmd[cmInd++] = (uint8_t) ((req->InputCommandsList[idx] >> 8) & 0xFF);
	}
	cmd[cmInd++] = req->OutputCommandsNum;
	for (idx = 0; idx < req->OutputCommandsNum; idx++) {
		cmd[cmInd++] = (uint8_t) (req->OutputCommandsList[idx] & 0xFF);
		cmd[cmInd++] = (uint8_t) ((req->OutputCommandsList[idx] >> 8) & 0xFF);
	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SAPI),
	MT_SAPI_APP_REGISTER_REQ, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      zbStartReq
 *
 * @brief   This command starts the ZigBee stack in the ZNP device. When the ZigBee stack
 *           starts, the device reads the programmed configuration parameters and operates accordingly.
 *
 * @param    - 
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t zbStartReq() {
	uint8_t status;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SAPI),
	MT_SAPI_START_REQ, NULL, 0);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      zbPermitJoiningReq
 *
 * @brief   This command is used to control the joining permissions and thus allow or disallow new devices
 *           from joining the network. By default, permit joining is always on.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t zbPermitJoiningReq(PermitJoiningReqFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 3;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = (uint8_t) (req->Destination & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Destination >> 8) & 0xFF);
	cmd[cmInd++] = req->Timeout;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SAPI),
	MT_SAPI_PERMIT_JOINING_REQ, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      zbBindDevice
 *
 * @brief   This command is used to create or delete a ‘binding’ to another device on the network.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t zbBindDevice(BindDeviceFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 11;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->Create;
	cmd[cmInd++] = (uint8_t) (req->CommandId & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->CommandId >> 8) & 0xFF);
	memcpy((cmd + cmInd), req->DstIeee, 8);
	cmInd += 8;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SAPI),
	MT_SAPI_BIND_DEVICE, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      zbAllowBind
 *
 * @brief   This command is issued by the ZNP device to return the results from a
 *           ZB_BIND_DEVICE command.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t zbAllowBind(AllowBindFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 1;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->Timeout;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SAPI),
	MT_SAPI_ALLOW_BIND, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      zbSendDataReq
 *
 * @brief   This command initiates transmission of data to another device in the network. This command can
 *           only be issued after the application processor has registered its application using the
 *           ZB_APP_REGISTER_REQUEST and the device has successfully created or joined a network.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t zbSendDataReq(SendDataReqFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 8 + req->Len;
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = (uint8_t) (req->Destination & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Destination >> 8) & 0xFF);
	cmd[cmInd++] = (uint8_t) (req->CommandId & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->CommandId >> 8) & 0xFF);
	cmd[cmInd++] = req->Handle;
	cmd[cmInd++] = req->Ack;
	cmd[cmInd++] = req->Radius;
	cmd[cmInd++] = req->Len;
	for (idx = 0; idx < req->Len; idx++) {
		cmd[cmInd++] = req->Data[idx];
	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SAPI),
	MT_SAPI_SEND_DATA_REQ, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      zbFindDeviceReq
 *
 * @brief   This command is used to determine the short address for a device in the network.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t zbFindDeviceReq(FindDeviceReqFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 8;
	uint8_t cmd[cmdLen];

	memcpy((cmd + cmInd), req->SearchKey, 8);
	cmInd += 8;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SAPI),
	MT_SAPI_FIND_DEVICE_REQ, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      zbWriteConfiguration
 *
 * @brief   This command is used to write a configuration parameter to the ZNP device.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t zbWriteConfiguration(WriteConfigurationFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 3 + req->Len;
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = req->ConfigId;
	cmd[cmInd++] = req->Len;
	for (idx = 0; idx < req->Len; idx++) {
		cmd[cmInd++] = req->Value[idx];
	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SAPI),
	MT_SAPI_WRITE_CONFIGURATION, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      zbGetDeviceInfo
 *
 * @brief   This command retrieves a Device Information Property.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t zbGetDeviceInfo(GetDeviceInfoFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 1;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->Param;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SAPI),
	MT_SAPI_GET_DEVICE_INFO, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      zbReadConfiguration
 *
 * @brief   This command is used to read the value of a configuration parameter from the ZNP
 *           device.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t zbReadConfiguration(ReadConfigurationFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 1;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->ConfigId;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SAPI),
	MT_SAPI_READ_CONFIGURATION, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processReadConfigurationSrsp
 *
 * @brief   This function is trigered by the ZNP after a zbReadConfiguration call.
 *           Parses the incoming buffer to a command specific structure which
 *           is then passed to its respective callback.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of incoming buffer.
 *
 * @return   
 */
static void processReadConfigurationSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSapiCbs.pfnSapiReadConfigurationSrsp) {
		uint8_t msgIdx = 2;
		ReadConfigurationSrspFormat_t rsp;
		if (rpcLen < 3) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];
		rsp.ConfigId = rpcBuff[msgIdx++];
		rsp.Len = rpcBuff[msgIdx++];
		if (rpcLen > 3) {
			uint32_t i;
			for (i = 0; i < rsp.Len; i++) {
				rsp.Value[i] = rpcBuff[msgIdx++];
			}
		}
		mtSapiCbs.pfnSapiReadConfigurationSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processGetDeviceInfoSrsp
 *
 * @brief   This function is trigered by the ZNP after a zbGetDeviceInfo call.
 *           Parses the incoming buffer to a command specific structure which
 *           is then passed to its respective callback.
 *
 * @param   rpcBuff - Incoming buffer
 * @param   rpcLen - Length of incoming buffer.
 *
 * @return   
 */
static void processGetDeviceInfoSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSapiCbs.pfnSapiGetDeviceInfoSrsp) {
		uint8_t msgIdx = 2;
		GetDeviceInfoSrspFormat_t rsp;
		if (rpcLen < 9) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Param = rpcBuff[msgIdx++];
		uint8_t i;
		for (i = 0; i < 8; i++) {
			rsp.Value[i] = rpcBuff[msgIdx++];
		}

		mtSapiCbs.pfnSapiGetDeviceInfoSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      processFindDeviceCnf
 *
 * @brief   This function is trigered by the ZNP after a zbFindDevice call.
 *           Parses the incoming buffer to a command specific structure which
 *           is then passed to its respective callback.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of incoming buffer.
 *
 * @return   
 */
static void processFindDeviceCnf(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSapiCbs.pfnSapiFindDeviceCnf) {
		uint8_t msgIdx = 2;
		FindDeviceCnfFormat_t rsp;
		if (rpcLen < 11) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.SearchKey = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Result = 0;
		uint8_t i;
		for (i = 0; i < 8; i++)
			rsp.Result |= ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);

		mtSapiCbs.pfnSapiFindDeviceCnf(&rsp);
	}
}

/*********************************************************************
 * @fn      processSendDataCnf
 *
 * @brief   This function is trigered by the ZNP after a zbSendData call.
 *           Parses the incoming buffer to a command specific structure which
 *           is then passed to its respective callback.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of incoming buffer.
 *
 * @return   
 */
static void processSendDataCnf(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSapiCbs.pfnSapiSendDataCnf) {
		uint8_t msgIdx = 2;
		SendDataCnfFormat_t rsp;
		if (rpcLen < 2) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Handle = rpcBuff[msgIdx++];
		rsp.Status = rpcBuff[msgIdx++];

		mtSapiCbs.pfnSapiSendDataCnf(&rsp);
	}
}

/*********************************************************************
 * @fn      processReceiveDataInd
 *
 * @brief   This callback is called asynchronously by the ZNP device when it has received a packet
 *           Parses the incoming buffer to a command specific structure which
 *           is then passed to its respective callback.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of incoming buffer.
 *
 * @return   
 */
static void processReceiveDataInd(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSapiCbs.pfnSapiReceiveDataInd) {
		uint8_t msgIdx = 2;
		ReceiveDataIndFormat_t rsp;
		if (rpcLen < 6) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Source = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Command = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Len = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		if (rpcLen > 6) {
			uint32_t i;
			for (i = 0; i < rsp.Len; i++) {
				rsp.Data[i] = rpcBuff[msgIdx++];
			}
		}
		mtSapiCbs.pfnSapiReceiveDataInd(&rsp);
	}
}

/*********************************************************************
 * @fn      processAllowBindCnf
 *
 * @brief   This command is issued by the ZNP device when it responds to a bind request from a
 *           remote device.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of incoming buffer.
 *
 * @return   
 */
static void processAllowBindCnf(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSapiCbs.pfnSapiAllowBindCnf) {
		uint8_t msgIdx = 2;
		AllowBindCnfFormat_t rsp;
		if (rpcLen < 2) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Source = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		mtSapiCbs.pfnSapiAllowBindCnf(&rsp);
	}
}

/*********************************************************************
 * @fn      processBindCnf
 *
 * @brief   This command is issued by the ZNP device to return the results from a
 *           ZB_BIND_DEVICE command.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of incoming buffer.
 *
 * @return   
 */
static void processBindCnf(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSapiCbs.pfnSapiBindCnf) {
		uint8_t msgIdx = 2;
		BindCnfFormat_t rsp;
		if (rpcLen < 3) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.CommandId = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;
		rsp.Status = rpcBuff[msgIdx++];

		mtSapiCbs.pfnSapiBindCnf(&rsp);
	}
}

/*********************************************************************
 * @fn      processStartCnf
 *
 * @brief   This command is issued by the ZNP device to return the results from a
 *           ZB_START_REQUEST command.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of incoming buffer.
 *
 * @return   
 */
static void processStartCnf(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSapiCbs.pfnSapiStartCnf) {
		uint8_t msgIdx = 2;
		StartCnfFormat_t rsp;
		if (rpcLen < 1) {
			printf("MT_RPC_ERR_LENGTH\n");
		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];

		mtSapiCbs.pfnSapiStartCnf(&rsp);
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
		case MT_SAPI_READ_CONFIGURATION:
			dbg_print(PRINT_LEVEL_VERBOSE, "sapiProcess: MT_SAPI_READ_CONFIGURATION\n");
			processReadConfigurationSrsp(rpcBuff, rpcLen);
			break;
		case MT_SAPI_GET_DEVICE_INFO:
			dbg_print(PRINT_LEVEL_VERBOSE, "sapiProcess: MT_SAPI_GET_DEVICE_INFO\n");
			processGetDeviceInfoSrsp(rpcBuff, rpcLen);
			break;
		default:
			dbg_print(PRINT_LEVEL_INFO, "processSrsp: unsupported message  [%x:%x]\n", rpcBuff[0], rpcBuff[1]);
			break;
	}

}
/*************************************************************************************************
 * @fn      sapiProcess()
 *
 * @brief   read and process the RPC ZDO message from the ZB SoC
 *
 * @param   none
 *
 * @return  length of current Rx Buffer
 ***********************************************************************************************/
void sapiProcess(uint8_t *rpcBuff, uint8_t rpcLen) {
	dbg_print(PRINT_LEVEL_VERBOSE, "sapiProcess: processing CMD0:%x, CMD1:%x\n", rpcBuff[0], rpcBuff[1]);

//process the synchronous SRSP from SREQ
	if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP) {
		processSrsp(rpcBuff, rpcLen);
	}
	else {
		//Read CMD1 and processes the specific SREQ
		switch (rpcBuff[1]) {
			case MT_SAPI_FIND_DEVICE_CNF:
				dbg_print(PRINT_LEVEL_VERBOSE, "sapiProcess: MT_SAPI_FIND_DEVICE_CNF\n");
				processFindDeviceCnf(rpcBuff, rpcLen);
				break;
			case MT_SAPI_SEND_DATA_CNF:
				dbg_print(PRINT_LEVEL_VERBOSE, "sapiProcess: MT_SAPI_SEND_DATA_CNF\n");
				processSendDataCnf(rpcBuff, rpcLen);
				break;
			case MT_SAPI_RECEIVE_DATA_IND:
				dbg_print(PRINT_LEVEL_VERBOSE, "sapiProcess: MT_SAPI_RECEIVE_DATA_IND\n");
				processReceiveDataInd(rpcBuff, rpcLen);
				break;
			case MT_SAPI_ALLOW_BIND_CNF:
				dbg_print(PRINT_LEVEL_VERBOSE, "sapiProcess: MT_SAPI_ALLOW_BIND_CNF\n");
				processAllowBindCnf(rpcBuff, rpcLen);
				break;
			case MT_SAPI_BIND_CNF:
				dbg_print(PRINT_LEVEL_VERBOSE, "sapiProcess: MT_SAPI_BIND_CNF\n");
				processBindCnf(rpcBuff, rpcLen);
				break;
			case MT_SAPI_START_CNF:
				dbg_print(PRINT_LEVEL_VERBOSE, "sapiProcess: MT_SAPI_START_CNF\n");
				processStartCnf(rpcBuff, rpcLen);
				break;

			default:
				dbg_print(PRINT_LEVEL_INFO, "sapiProcess: CMD0:%x, CMD1:%x, not handled\n", rpcBuff[0], rpcBuff[1]);
				break;
		}
	}
}

/*********************************************************************
 * @fn      sapiRegisterCallbacks
 *
 * @brief Register the sapi callbacks
 *
 * @param cbs - callback structure for mtSapi
 *
 * @return
 */
void sapiRegisterCallbacks(mtSapiCb_t cbs) {
	memcpy(&mtSapiCbs, &cbs, sizeof(mtSapiCb_t));
}

