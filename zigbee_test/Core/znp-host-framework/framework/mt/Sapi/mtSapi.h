/*
 * mtSapi.h
 *
 * This module contains the API for the MT ZDO Interface.
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
#ifndef ZBMTSAPI_H
#define ZBMTSAPI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "mtAf.h"

/***************************************************************************************************
 * SAPI COMMANDS
 ***************************************************************************************************/

// SAPI MT Command Identifiers
/* AREQ from Host */
#define MT_SAPI_SYS_RESET                   0x09

/* SREQ/SRSP */
#define MT_SAPI_START_REQ                   0x00
#define MT_SAPI_BIND_DEVICE             0x01
#define MT_SAPI_ALLOW_BIND              0x02
#define MT_SAPI_SEND_DATA_REQ               0x03
#define MT_SAPI_READ_CONFIGURATION                0x04
#define MT_SAPI_WRITE_CONFIGURATION               0x05
#define MT_SAPI_GET_DEVICE_INFO            0x06
#define MT_SAPI_FIND_DEVICE_REQ                0x07
#define MT_SAPI_PERMIT_JOINING_REQ               0x08
#define MT_SAPI_APP_REGISTER_REQ            0x0a

/* AREQ to host */
#define MT_SAPI_START_CNF                   0x80
#define MT_SAPI_BIND_CNF                    0x81
#define MT_SAPI_ALLOW_BIND_CNF              0x82
#define MT_SAPI_SEND_DATA_CNF               0x83
#define MT_SAPI_FIND_DEVICE_CNF                0x85
#define MT_SAPI_RECEIVE_DATA_IND                0x87

typedef struct
{
	uint8_t AppEndpoint;
	uint16_t AppProfileId;
	uint16_t DeviceId;
	uint8_t DeviceVersion;
	uint8_t Unused;
	uint8_t InputCommandsNum;
	uint16_t InputCommandsList[255];
	uint8_t OutputCommandsNum;
	uint16_t OutputCommandsList[255];
}AppRegisterReqFormat_t;

typedef struct
{
	uint16_t Destination;
	uint8_t Timeout;
}PermitJoiningReqFormat_t;

typedef struct
{
	uint8_t Create;
	uint16_t CommandId;
	uint8_t DstIeee[8];
}BindDeviceFormat_t;

typedef struct
{
	uint8_t Timeout;
}AllowBindFormat_t;

typedef struct
{
	uint16_t Destination;
	uint16_t CommandId;
	uint8_t Handle;
	uint8_t Ack;
	uint8_t Radius;
	uint8_t Len;
	uint8_t Data[99];
}SendDataReqFormat_t;

typedef struct
{
	uint8_t SearchKey[8];
}FindDeviceReqFormat_t;

typedef struct
{
	uint8_t ConfigId;
	uint8_t Len;
	uint8_t Value[128];
}WriteConfigurationFormat_t;

typedef struct
{
	uint8_t Param;
}GetDeviceInfoFormat_t;

typedef struct
{
	uint8_t ConfigId;
}ReadConfigurationFormat_t;

typedef struct
{
	uint8_t Status;
	uint8_t ConfigId;
	uint8_t Len;
	uint8_t Value[128];
}ReadConfigurationSrspFormat_t;

typedef struct
{
	uint8_t Param;
	uint8_t Value[8];
}GetDeviceInfoSrspFormat_t;

typedef struct
{
	uint16_t SearchKey;
	uint64_t Result;
}FindDeviceCnfFormat_t;

typedef struct
{
	uint8_t Handle;
	uint8_t Status;
}SendDataCnfFormat_t;

typedef struct
{
	uint16_t Source;
	uint16_t Command;
	uint16_t Len;
	uint8_t Data[84];
}ReceiveDataIndFormat_t;

typedef struct
{
	uint16_t Source;
}AllowBindCnfFormat_t;

typedef struct
{
	uint16_t CommandId;
	uint8_t Status;
}BindCnfFormat_t;

typedef struct
{
	uint8_t Status;
}StartCnfFormat_t;

typedef uint8_t (*mtSapiStub_t)(void);
typedef uint8_t (*mtSapiReadConfigurationSrspCb_t)(ReadConfigurationSrspFormat_t *msg);
typedef uint8_t (*mtSapiGetDeviceInfoSrspCb_t)(GetDeviceInfoSrspFormat_t *msg);
typedef uint8_t (*mtSapiFindDeviceCnfCb_t)(FindDeviceCnfFormat_t *msg);
typedef uint8_t (*mtSapiSendDataCnfCb_t)(SendDataCnfFormat_t *msg);
typedef uint8_t (*mtSapiReceiveDataIndCb_t)(ReceiveDataIndFormat_t *msg);
typedef uint8_t (*mtSapiAllowBindCnfCb_t)(AllowBindCnfFormat_t *msg);
typedef uint8_t (*mtSapiBindCnfCb_t)(BindCnfFormat_t *msg);
typedef uint8_t (*mtSapiStartCnfCb_t)(StartCnfFormat_t *msg);

typedef struct
{
	mtSapiReadConfigurationSrspCb_t pfnSapiReadConfigurationSrsp; //MT_SAPI_READ_CONFIGURATION
	mtSapiGetDeviceInfoSrspCb_t pfnSapiGetDeviceInfoSrsp;//MT_SAPI_GET_DEVICE_INFO
	mtSapiFindDeviceCnfCb_t pfnSapiFindDeviceCnf;//MT_SAPI_FIND_DEVICE_CNF
	mtSapiSendDataCnfCb_t pfnSapiSendDataCnf;//MT_SAPI_SEND_DATA_CNF
	mtSapiReceiveDataIndCb_t pfnSapiReceiveDataInd;//MT_SAPI_RECEIVE_DATA_IND
	mtSapiAllowBindCnfCb_t pfnSapiAllowBindCnf;//MT_SAPI_ALLOW_BIND_CNF
	mtSapiBindCnfCb_t pfnSapiBindCnf;//MT_SAPI_BIND_CNF
	mtSapiStartCnfCb_t pfnSapiStartCnf;//MT_SAPI_START_CNF

}mtSapiCb_t;

void sapiRegisterCallbacks(mtSapiCb_t cbs);
void sapiProcess(uint8_t *rpcBuff, uint8_t rpcLen);
uint8_t zbSystemReset ( void );
uint8_t zbAppRegisterReq(AppRegisterReqFormat_t *req);
uint8_t zbStartReq(void);
uint8_t zbPermitJoiningReq(PermitJoiningReqFormat_t *req);
uint8_t zbBindDevice(BindDeviceFormat_t *req);
uint8_t zbAllowBind(AllowBindFormat_t *req);
uint8_t zbSendDataReq(SendDataReqFormat_t *req);
uint8_t zbFindDeviceReq(FindDeviceReqFormat_t *req);
uint8_t zbWriteConfiguration(WriteConfigurationFormat_t *req);
uint8_t zbGetDeviceInfo(GetDeviceInfoFormat_t *req);
uint8_t zbReadConfiguration(ReadConfigurationFormat_t *req);

#ifdef __cplusplus
}
#endif

#endif /* ZBMTSAPI_H */

