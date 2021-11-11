/*
 * mtAf.h
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
#ifndef ZBMTAF_H
#define ZBMTAF_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef uint16_t cId_t;
// Simple Description Format Structure

enum
{
	AddrNotPresent = 0,
	AddrGroup = 1,
	Addr16Bit = 2,
	Addr64Bit = 3,
	AddrBroadcast = 15
};

typedef enum
{
	afAddrNotPresent = AddrNotPresent,
	afAddr16Bit = Addr16Bit,
	afAddr64Bit = Addr64Bit,
	afAddrGroup = AddrGroup,
	afAddrBroadcast = AddrBroadcast
} afAddrMode_t;

#define Z_EXTADDR_LEN   8
typedef uint8_t ZLongAddr_t[Z_EXTADDR_LEN];

typedef struct
{
	union
	{
		uint16_t shortAddr;
		ZLongAddr_t extAddr;
	} addr;
	afAddrMode_t addrMode;
	uint8_t endPoint;
	uint16_t panId; // used for the INTER_PAN feature
} afAddrType_t;

// Generalized MSG Command Format
typedef struct
{
	uint8_t TransSeqNumber;
	uint16_t DataLength; // Number of bytes in TransData
	uint8_t *Data;
} afMSGCommandFormat_t;

typedef struct
{
	uint16_t groupId; /* Message's group ID - 0 if not set */
	uint16_t clusterId; /* Message's cluster ID */
	afAddrType_t srcAddr; /* Source Address, if endpoint is STUBAPS_INTER_PAN_EP,
	 it's an InterPAN message */
	uint16_t macDestAddr; /* MAC header destination short address */
	uint8_t endPoint; /* destination endpoint */
	uint8_t wasBroadcast; /* TRUE if network destination was a broadcast address */
	uint8_t LinkQuality; /* The link quality of the received data frame */
	uint8_t correlation; /* The raw correlation value of the received data frame */
	int8_t rssi; /* The received RF power in units dBm */
	uint8_t SecurityUse; /* deprecated */
	uint32_t timestamp; /* receipt timestamp from MAC */
	uint8_t nwkSeqNum; /* network header frame sequence number */
	afMSGCommandFormat_t cmd; /* Application Data */
	uint16_t macSrcAddr; /* MAC header source short address */
} afIncomingMSGPacket_t;

/***************************************************************************************************
 * AF COMMANDS
 ***************************************************************************************************/

/* SREQ/SRSP */
#define MT_AF_REGISTER                       0x00
#define MT_AF_DATA_REQUEST                   0x01  /* AREQ optional, but no AREQ response. */
#define MT_AF_DATA_REQUEST_EXT               0x02  /* AREQ optional, but no AREQ response. */
#define MT_AF_DATA_REQUEST_SRC_RTG            0x03

#define MT_AF_INTER_PAN_CTL                  0x10
#define MT_AF_DATA_STORE                     0x11
#define MT_AF_DATA_RETRIEVE                  0x12
#define MT_AF_APSF_CONFIG_SET                0x13

/* AREQ to host */
#define MT_AF_DATA_CONFIRM                   0x80
#define MT_AF_INCOMING_MSG                   0x81
#define MT_AF_INCOMING_MSG_EXT               0x82
#define MT_AF_REFLECT_ERROR                  0x83

#define afStatus_SUCCESS                     0x00
#define afStatus_FAILED                      0x01
#define afStatus_INVALID_PARAMETER           0x02
#define afStatus_MEM_FAIL                    0x10
#define afStatus_NO_ROUTE                    0xCD
#define afStatus_DUPLICATE									 0xB8

typedef struct
{
	uint8_t EndPoint;
	uint16_t AppProfId;
	uint16_t AppDeviceId;
	uint8_t AppDevVer;
	uint8_t LatencyReq;
	uint8_t AppNumInClusters;
	uint16_t AppInClusterList[16];
	uint8_t AppNumOutClusters;
	uint16_t AppOutClusterList[16];
} RegisterFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t DstEndpoint;
	uint8_t SrcEndpoint;
	uint16_t ClusterID;
	uint8_t TransID;
	uint8_t Options;
	uint8_t Radius;
	uint8_t Len;
	uint8_t Data[128];
} DataRequestFormat_t;

typedef struct
{
	uint8_t DstAddrMode;
	uint8_t DstAddr[8];
	uint8_t DstEndpoint;
	uint16_t DstPanID;
	uint8_t SrcEndpoint;
	uint16_t ClusterId;
	uint8_t TransId;
	uint8_t Options;
	uint8_t Radius;
	uint16_t Len;
	uint8_t Data[230];
} DataRequestExtFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t DstEndpoint;
	uint8_t SrcEndpoint;
	uint16_t ClusterID;
	uint8_t TransID;
	uint8_t Options;
	uint8_t Radius;
	uint8_t RelayCount;
	uint16_t RelayList[255];
	uint8_t Len;
	uint8_t Data[128];
} DataRequestSrcRtgFormat_t;

typedef struct
{
	uint8_t Command;
	uint8_t Data[3];
} InterPanCtlFormat_t;

typedef struct
{
	uint16_t Index;
	uint8_t Length;
	uint8_t Data[247];
} DataStoreFormat_t;

typedef struct
{
	uint8_t Status;
	uint8_t Endpoint;
	uint8_t TransId;
} DataConfirmFormat_t;

typedef struct
{
	uint16_t GroupId;
	uint16_t ClusterId;
	uint16_t SrcAddr;
	uint8_t SrcEndpoint;
	uint8_t DstEndpoint;
	uint8_t WasVroadcast;
	uint8_t LinkQuality;
	uint8_t SecurityUse;
	uint32_t TimeStamp;
	uint8_t TransSeqNum;
	uint8_t Len;
	uint8_t Data[99];
} IncomingMsgFormat_t;

typedef struct
{
	uint16_t GroupId;
	uint16_t ClusterId;
	uint8_t SrcAddrMode;
	uint64_t SrcAddr;
	uint8_t SrcEndpoint;
	uint16_t SrcPanId;
	uint8_t DstEndpoint;
	uint8_t WasVroadcast;
	uint8_t LinkQuality;
	uint8_t SecurityUse;
	uint32_t TimeStamp;
	uint8_t TransSeqNum;
	uint8_t Len;
	uint8_t Data[99];
} IncomingMsgExtFormat_t;

typedef struct
{
	uint8_t TimeStamp[4];
	uint16_t Index;
	uint8_t Length;
} DataRetrieveFormat_t;

typedef struct
{
	uint8_t Status;
	uint8_t Length;
	uint8_t Data[248];
} DataRetrieveSrspFormat_t;

typedef struct
{
	uint8_t success;	
} RegisterSrspFormat_t;

typedef struct
{
	uint8_t success;	
} DataRequestSrspFormat_t;

typedef struct
{
	uint8_t Endpoint;
	uint8_t FrameDelay;
	uint8_t WindowSize;
} ApsfConfigSetFormat_t;

typedef struct
{
	uint8_t Status;
	uint8_t Endpoint;
	uint8_t TransId;
	uint8_t DstAddrMode;
	uint16_t DstAddr;
} ReflectErrorFormat_t;

typedef uint8_t (*mtAfDataConfirmCb_t)(DataConfirmFormat_t *msg);
typedef uint8_t (*mtAfIncomingMsgCb_t)(IncomingMsgFormat_t *msg);
typedef uint8_t (*mtAfIncomingMsgExt_t)(IncomingMsgExtFormat_t *msg);
typedef uint8_t (*mtAfRegisterSrspCb_t)(RegisterSrspFormat_t *msg);
typedef uint8_t (*mtAfDataRequestSrspCb_t)(DataRequestSrspFormat_t *msg);
typedef uint8_t (*mtAfDataRetrieveSrspCb_t)(DataRetrieveSrspFormat_t *msg);
typedef uint8_t (*mtAfReflectErrorCb_t)(ReflectErrorFormat_t *msg);

typedef struct
{
	mtAfDataConfirmCb_t pfnAfDataConfirm;				//MT_AF_DATA_CONFIRM
	mtAfIncomingMsgCb_t pfnAfIncomingMsg;				//MT_AF_INCOMING_MSG
	mtAfIncomingMsgExt_t pfnAfIncomingMsgExt;			//MT_AF_INCOMING_MSG_EXT
	mtAfRegisterSrspCb_t pfnAfRegisterSrsp;				//MT_AF_REGISTER
	mtAfDataRequestSrspCb_t pfnAfDataReqeuestSrsp;		//MT_AF_DATA_REQUEST
	mtAfDataRetrieveSrspCb_t pfnAfDataRetrieveSrsp;		//MT_AF_DATA_RETRIEVE
	mtAfReflectErrorCb_t pfnAfReflectError;				//MT_AF_REFLECT_ERROR
} mtAfCb_t;

void afRegisterCallbacks(mtAfCb_t cbs);
void afProcess(uint8_t *rpcBuff, uint8_t rpcLen);
uint8_t afRegister(RegisterFormat_t *req);
uint8_t afDataRequest(DataRequestFormat_t *req);
uint8_t afDataRequestExt(DataRequestExtFormat_t *req);
uint8_t afDataRequestSrcRtg(DataRequestSrcRtgFormat_t *req);
uint8_t afInterPanCtl(InterPanCtlFormat_t *req);
uint8_t afDataStore(DataStoreFormat_t *req);
uint8_t afDataRetrieve(DataRetrieveFormat_t *req);
uint8_t afApsfConfigSet(ApsfConfigSetFormat_t *req);

//uint8_t afRegisterExtended(SimpleDescriptionFormat_t *simpleDesc);
//uint8_t afDataRequest(afAddrType_t *dstAddr, uint8_t srcEP, uint16_t cID,
//uint16_t len, uint8_t *buf, uint8_t transID, uint8_t options,
//uint8_t radius);

#ifdef __cplusplus
}
#endif

#endif /* ZBMTAF_H */
