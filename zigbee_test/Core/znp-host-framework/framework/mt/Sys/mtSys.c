/*
 * mtSys.c
 *
 * This module contains the API for the MT SYS Interface.
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

#include "mtSys.h"
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
static mtSysCb_t mtSysCbs;
extern uint8_t srspRpcBuff[RPC_MAX_LEN];
extern uint8_t srspRpcLen;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void processSrsp(uint8_t *rpcBuff, uint8_t rpcLen);
static void processResetInd(uint8_t *rpcBuff, uint8_t rpcLen);

/*********************************************************************
 * @fn      sysPing
 *
 * @brief   This command issues PING requests to verify if a device is active and check
 *           the capability of the device.
 *
 * @param    -
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysPing() {
	uint8_t status;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_PING, NULL, 0);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processPingSrsp
 *
 * @brief   This function is trigered by the ZNP after a sysPing call.
 *           Parses the incoming buffer to a command specific structure which
 *           is then passed to its respective callback.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of incoming buffer.
 *
 */
static void processPingSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysPingSrsp) {
		uint8_t msgIdx = 2;
		PingSrspFormat_t rsp;
		if (rpcLen < 2) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Capabilities = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		mtSysCbs.pfnSysPingSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysSetExtAddr
 *
 * @brief   This command is used to set the extended address of the device.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysSetExtAddr(SetExtAddrFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 8;
	uint8_t cmd[cmdLen];

	memcpy((cmd + cmInd), req->ExtAddr, 8);
	cmInd += 8;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_SET_EXTADDR, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      sysGetExtAddr
 *
 * @brief   This command requests the ZNP device to respond with its extended IEEE address.
 *
 * @param    -
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysGetExtAddr() {
	uint8_t status;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_GET_EXTADDR, NULL, 0);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processGetExtAddrSrsp
 *
 * @brief   This Function is trigered after a call to sysGetExtAddr. Gets a buffer with IEEE address and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processGetExtAddrSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysGetExtAddrSrsp) {
		uint8_t msgIdx = 2;
		GetExtAddrSrspFormat_t rsp;
		if (rpcLen < 8) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.ExtAddr = 0;
		uint8_t i;
		for (i = 0; i < 8; i++)
			rsp.ExtAddr |= ((uint64_t) rpcBuff[msgIdx++]) << (i * 8);

		mtSysCbs.pfnSysGetExtAddrSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysRamRead
 *
 * @brief   This command requests to read a specific section of Ram on the ZNP.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysRamRead(RamReadFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 3;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = (uint8_t) (req->Address & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Address >> 8) & 0xFF);
	cmd[cmInd++] = req->Len;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_RAM_READ, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processRamReadSrsp
 *
 * @brief   This Function is trigered after a call to sysRamRead.
 *           Parses the incoming buffer to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processRamReadSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysRamReadSrsp) {
		uint8_t msgIdx = 2;
		RamReadSrspFormat_t rsp;
		if (rpcLen < 2) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];
		rsp.Len = rpcBuff[msgIdx++];
		if (rpcLen > 2) {
			uint32_t i;
			for (i = 0; i < rsp.Len; i++) {
				rsp.Value[i] = rpcBuff[msgIdx++];
			}
		}
		mtSysCbs.pfnSysRamReadSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysRamWrite
 *
 * @brief   This command requests to write to a specific section of Ram on the ZNP.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysRamWrite(RamWriteFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 4 + req->Len;
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = (uint8_t) (req->Address & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Address >> 8) & 0xFF);
	cmd[cmInd++] = req->Len;
	for (idx = 0; idx < req->Len; idx++) {
		cmd[cmInd++] = req->Value[idx];
	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_RAM_WRITE, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      sysResetReq
 *
 * @brief   This command resets the ZNP device.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysResetReq(ResetReqFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 1;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->Type;

	status = rpcSendFrame((MT_RPC_CMD_AREQ | MT_RPC_SYS_SYS),
	MT_SYS_RESET_REQ, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}
	return status;
}

/*********************************************************************
 * @fn      processResetInd
 *
 * @brief   This Function is trigered after a call to sysResetReq.
 *           Parses the incoming buffer to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processResetInd(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysResetInd) {
		uint8_t msgIdx = 2;
		ResetIndFormat_t rsp;
		if (rpcLen < 6) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Reason = rpcBuff[msgIdx++];
		rsp.TransportRev = rpcBuff[msgIdx++];
		rsp.ProductId = rpcBuff[msgIdx++];
		rsp.MajorRel = rpcBuff[msgIdx++];
		rsp.MinorRel = rpcBuff[msgIdx++];
		rsp.HwRev = rpcBuff[msgIdx++];

		mtSysCbs.pfnSysResetInd(&rsp);
	}
}

/*********************************************************************
 * @fn      sysVersion
 *
 * @brief   This command is issued by the host processor to request for the
 *           ZNP software version information.
 *
 * @param    -
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysVersion() {
	uint8_t status;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_VERSION, NULL, 0);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processVersionSrsp
 *
 * @brief   This Function is trigered after a call to sysVersion.
 *           Parses the incoming buffer to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processVersionSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysVersionSrsp) {
		uint8_t msgIdx = 2;
		VersionSrspFormat_t rsp;
		if (rpcLen < 5) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.TransportRev = rpcBuff[msgIdx++];
		rsp.Product = rpcBuff[msgIdx++];
		rsp.MajorRel = rpcBuff[msgIdx++];
		rsp.MinorRel = rpcBuff[msgIdx++];
		rsp.MaintRel = rpcBuff[msgIdx++];

		mtSysCbs.pfnSysVersionSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysOsalNvRead
 *
 * @brief   This command is used to read data values from an item
 *           stored in NV memory.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysOsalNvRead(OsalNvReadFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 3;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = (uint8_t) (req->Id & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Id >> 8) & 0xFF);
	cmd[cmInd++] = req->Offset;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_OSAL_NV_READ, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processOsalNvReadSrsp
 *
 * @brief   This Function is trigered after a call to sysOsalNvRead.
 *           Parses the incoming buffer to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processOsalNvReadSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysOsalNvReadSrsp) {
		uint8_t msgIdx = 2;
		OsalNvReadSrspFormat_t rsp;
		if (rpcLen < 2) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Status = rpcBuff[msgIdx++];
		rsp.Len = rpcBuff[msgIdx++];
		if (rpcLen > 2) {
			uint32_t i;
			for (i = 0; i < rsp.Len; i++) {
				rsp.Value[i] = rpcBuff[msgIdx++];
			}
		}
		mtSysCbs.pfnSysOsalNvReadSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysOsalNvWrite
 *
 * @brief   This command is used to write data values to NV memory.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysOsalNvWrite(OsalNvWriteFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 4 + req->Len;
	uint8_t cmd[cmdLen];
	int idx;

	cmd[cmInd++] = (uint8_t) (req->Id & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Id >> 8) & 0xFF);
	cmd[cmInd++] = req->Offset;
	cmd[cmInd++] = req->Len;
	for (idx = 0; idx < req->Len; idx++) {
		cmd[cmInd++] = req->Value[idx];
	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_OSAL_NV_WRITE, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      sysOsalNvItemInit
 *
 * @brief   This command is used by the application processor to create
 *           and initialize an item in the ZNP.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysOsalNvItemInit(OsalNvItemInitFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 5 + req->InitLen;
	uint8_t cmd[cmdLen];

	int idx;

	cmd[cmInd++] = (uint8_t) (req->Id & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Id >> 8) & 0xFF);
	cmd[cmInd++] = (uint8_t) (req->ItemLen & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->ItemLen >> 8) & 0xFF);
	cmd[cmInd++] = req->InitLen;
	for (idx = 0; idx < req->InitLen; idx++) {
		cmd[cmInd++] = req->InitData[idx];
	}

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_OSAL_NV_ITEM_INIT, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      sysOsalNvDelete
 *
 * @brief   This command is used by the application processor to delete
 *           an item from NV memory
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysOsalNvDelete(OsalNvDeleteFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 4;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = (uint8_t) (req->Id & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Id >> 8) & 0xFF);
	cmd[cmInd++] = (uint8_t) (req->ItemLen & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->ItemLen >> 8) & 0xFF);

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_OSAL_NV_DELETE, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      sysOsalNvLength
 *
 * @brief   This command is used by the host processor to
 *           get the length of an item in the NV memory.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysOsalNvLength(OsalNvLengthFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 2;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = (uint8_t) (req->Id & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Id >> 8) & 0xFF);

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_OSAL_NV_LENGTH, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processOsalNvLengthSrsp
 *
 * @brief   This Function is trigered after a call to sysOsalNvLength.
 *           Parses the incoming buffer to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processOsalNvLengthSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysOsalNvLengthSrsp) {
		uint8_t msgIdx = 2;
		OsalNvLengthSrspFormat_t rsp;
		if (rpcLen < 2) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.ItemLen = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		mtSysCbs.pfnSysOsalNvLengthSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysOsalStartTimer
 *
 * @brief   This command starts a timer event. The event will expire after
 *           the indicated amount of time and a notification
 *           will be sent back.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysOsalStartTimer(OsalStartTimerFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 3;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->Id;
	cmd[cmInd++] = (uint8_t) (req->Timeout & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Timeout >> 8) & 0xFF);

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_OSAL_START_TIMER, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      sysOsalStopTimer
 *
 * @brief   This command stops a timer event.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysOsalStopTimer(OsalStopTimerFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 1;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->Id;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_OSAL_STOP_TIMER, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processOsalTimerExpired
 *
 * @brief   This callback is sent by the ZNP to indicate that a specific
 *           timer has been expired.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processOsalTimerExpired(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysOsalTimerExpired) {
		uint8_t msgIdx = 2;
		OsalTimerExpiredFormat_t rsp;
		if (rpcLen < 1) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Id = rpcBuff[msgIdx++];

		mtSysCbs.pfnSysOsalTimerExpired(&rsp);
	}
}

/*********************************************************************
 * @fn      sysStackTune
 *
 * @brief   This command tunes intricate or arcane settings at runtime.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysStackTune(StackTuneFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 2;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->Operation;
	cmd[cmInd++] = req->Value;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_STACK_TUNE, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processStackTuneSrsp
 *
 * @brief   This Function is trigered after a call to sysStackTune.
 *           Parses the incoming buffer to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processStackTuneSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysStackTuneSrsp) {
		uint8_t msgIdx = 2;
		StackTuneSrspFormat_t rsp;
		if (rpcLen < 1) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Value = rpcBuff[msgIdx++];

		mtSysCbs.pfnSysStackTuneSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysAdcRead
 *
 * @brief   This commands reads the value from the ADC.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysAdcRead(AdcReadFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 2;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->Channel;
	cmd[cmInd++] = req->Resolution;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_ADC_READ, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processAdcReadSrsp
 *
 * @brief   This Function is trigered after a call to sysAdcRead.
 *           Parses the incoming buffer to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processAdcReadSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysAdcReadSrsp) {
		uint8_t msgIdx = 2;
		AdcReadSrspFormat_t rsp;
		if (rpcLen < 2) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Value = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		mtSysCbs.pfnSysAdcReadSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysGpio
 *
 * @brief   Command controls the GPIO pins.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysGpio(GpioFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 2;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->Operation;
	cmd[cmInd++] = req->Value;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_GPIO, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processGpioSrsp
 *
 * @brief   This Function is trigered after a call to sysGpio.
 *           Parses the incoming buffer to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming Buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processGpioSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysGpioSrsp) {
		uint8_t msgIdx = 2;
		GpioSrspFormat_t rsp;
		if (rpcLen < 1) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Value = rpcBuff[msgIdx++];

		mtSysCbs.pfnSysGpioSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysRandom
 *
 * @brief   This command is used to get a random 16-bit number.
 *
 * @param    -
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysRandom() {
	uint8_t status;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_RANDOM, NULL, 0);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processRandomSrsp
 *
 * @brief   This Function is trigered after a call to sysRandom.
 *           Parses the incoming buffer to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming Buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processRandomSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysRandomSrsp) {
		uint8_t msgIdx = 2;
		RandomSrspFormat_t rsp;
		if (rpcLen < 2) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.Value = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		mtSysCbs.pfnSysRandomSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysSetTime
 *
 * @brief   This command sets the target system date and time.
 *           The time can bespecified in seconds since 00:00:00 on January 1,
 *           2000 or in parsed date/time components.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysSetTime(SetTimeFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 11;
	uint8_t cmd[cmdLen];

	memcpy((cmd + cmInd), req->UTCTime, 4);
	cmInd += 4;
	cmd[cmInd++] = req->Hour;
	cmd[cmInd++] = req->Minute;
	cmd[cmInd++] = req->Second;
	cmd[cmInd++] = req->Month;
	cmd[cmInd++] = req->Day;
	cmd[cmInd++] = (uint8_t) (req->Year & 0xFF);
	cmd[cmInd++] = (uint8_t) ((req->Year >> 8) & 0xFF);

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_SET_TIME, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      sysGetTime
 *
 * @brief   This command gets the target system date and time.
 *
 * @param    -
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysGetTime() {
	uint8_t status;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_GET_TIME, NULL, 0);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processGetTimeSrsp
 *
 * @brief   This Function is trigered after a call to sysGetTime.
 *           Parses the incoming buffer to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming Buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processGetTimeSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysGetTimeSrsp) {
		uint8_t msgIdx = 2;
		GetTimeSrspFormat_t rsp;
		if (rpcLen < 11) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.UTCTime = 0;
		uint8_t i;
		for (i = 0; i < 4; i++)
			rsp.UTCTime |= ((uint32_t) rpcBuff[msgIdx++]) << (i * 8);
		rsp.Hour = rpcBuff[msgIdx++];
		rsp.Minute = rpcBuff[msgIdx++];
		rsp.Second = rpcBuff[msgIdx++];
		rsp.Month = rpcBuff[msgIdx++];
		rsp.Day = rpcBuff[msgIdx++];
		rsp.Year = BUILD_UINT16(rpcBuff[msgIdx], rpcBuff[msgIdx + 1]);
		msgIdx += 2;

		mtSysCbs.pfnSysGetTimeSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysSetTxPower
 *
 * @brief   This command sets the target system radio transmit power.
 *
 * @param   req - Pointer to command specific structure.
 *
 * @return   status, either Success (0) or Failure (1).
 */
uint8_t sysSetTxPower(SetTxPowerFormat_t *req) {
	uint8_t status;
	uint8_t cmInd = 0;
	uint32_t cmdLen = 1;
	uint8_t cmd[cmdLen];

	cmd[cmInd++] = req->TxPower;

	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SYS),
	MT_SYS_SET_TX_POWER, cmd, cmdLen);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}

/*********************************************************************
 * @fn      processSetTxPowerSrsp
 *
 * @brief   This Function is trigered after a call to sysSetTxPower.
 *           Parses the incoming TX power to a command specific structure
 *           and passes it to its respective callback function.
 *
 * @param   rpcBuff - Incoming Buffer.
 * @param   rpcLen - Length of buffer.
 *
 */
static void processSetTxPowerSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	if (mtSysCbs.pfnSysSetTxPowerSrsp) {
		uint8_t msgIdx = 2;
		SetTxPowerSrspFormat_t rsp;
		if (rpcLen < 1) {
			printf("MT_RPC_ERR_LENGTH\n");

		}
		//printf("rpcLen = %d\n", rpcLen);

		rsp.TxPower = rpcBuff[msgIdx++];

		mtSysCbs.pfnSysSetTxPowerSrsp(&rsp);
	}
}

/*********************************************************************
 * @fn      sysRegisterCallbacks
 *
 * @brief
 *
 * @param
 *
 */
void sysRegisterCallbacks(mtSysCb_t cbs) {
	memcpy(&mtSysCbs, &cbs, sizeof(mtSysCb_t));
}

/*********************************************************************
 * @fn      processSrsp
 *
 * @brief  Generic function for processing the SRSP and copying it to
 *         local buffer for SREQ function to deal with
 *
 * @param
 *

 */
static void processSrsp(uint8_t *rpcBuff, uint8_t rpcLen) {
	//copies sresp to local buffer
	memcpy(srspRpcBuff, rpcBuff, rpcLen);
	//srspRpcLen = rpcLen;
	switch (rpcBuff[1]) {
		case MT_SYS_PING:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_PING\n");
			processPingSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_GET_EXTADDR:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_GET_EXTADDR\n");
			processGetExtAddrSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_RAM_READ:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_RAM_READ\n");
			processRamReadSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_VERSION:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_VERSION\n");
			processVersionSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_OSAL_NV_READ:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_OSAL_NV_READ\n");
			processOsalNvReadSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_OSAL_NV_WRITE:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_OSAL_NV_WRITE\n");
			break;
		case MT_SYS_OSAL_NV_LENGTH:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_OSAL_NV_LENGTH\n");
			processOsalNvLengthSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_STACK_TUNE:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_STACK_TUNE\n");
			processStackTuneSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_ADC_READ:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_ADC_READ\n");
			processAdcReadSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_GPIO:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_GPIO\n");
			processGpioSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_RANDOM:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_RANDOM\n");
			processRandomSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_GET_TIME:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_GET_TIME\n");
			processGetTimeSrsp(rpcBuff, rpcLen);
			break;
		case MT_SYS_SET_TX_POWER:
			dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_SET_TX_POWER\n");
			processSetTxPowerSrsp(rpcBuff, rpcLen);
			break;
		default:
			dbg_print(PRINT_LEVEL_INFO, "processSrsp: unsupported message\n");
			break;
	}

}

/*************************************************************************************************
 * @fn      sysProcess()
 *
 * @brief   read and process the RPC Sys message from the ZB SoC
 *
 * @param   rpcLen has the size of the frame: cmd0 + cmd1 + payload + FCS
 *

 *************************************************************************************************/
void sysProcess(uint8_t *rpcBuff, uint8_t rpcLen) {
	dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: processing CMD0:%x, CMD1:%x\n", rpcBuff[0], rpcBuff[1]);

	//process the synchronous SRSP from SREQ
	if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP) {
		processSrsp(rpcBuff, rpcLen);
	}
	else {
		//Read CMD1 and processes the specific SREQ
		switch (rpcBuff[1]) {
			case MT_SYS_RESET_IND:
				dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_RESET_IND\n");
				processResetInd(rpcBuff, rpcLen);
				break;
			case MT_SYS_OSAL_TIMER_EXPIRED:
				dbg_print(PRINT_LEVEL_VERBOSE, "sysProcess: MT_SYS_OSAL_TIMER_EXPIRED\n");
				processOsalTimerExpired(rpcBuff, rpcLen);
				break;
			default:
				dbg_print(PRINT_LEVEL_WARNING, "processRpcSys: CMD0:%x, CMD1:%x, not handled\n", rpcBuff[0], rpcBuff[1]);
				break;
		}
	}
}

