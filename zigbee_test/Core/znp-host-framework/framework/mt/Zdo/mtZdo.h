/*
 * mtZdo.h
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
#ifndef ZBMTZDO_H
#define ZBMTZDO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/***************************************************************************************************
 * ZDO COMMANDS
 ***************************************************************************************************/

/* SREQ/SRSP */
#define MT_ZDO_NWK_ADDR_REQ                  0x00
#define MT_ZDO_IEEE_ADDR_REQ                 0x01
#define MT_ZDO_NODE_DESC_REQ                 0x02
#define MT_ZDO_POWER_DESC_REQ                0x03
#define MT_ZDO_SIMPLE_DESC_REQ               0x04
#define MT_ZDO_ACTIVE_EP_REQ                 0x05
#define MT_ZDO_MATCH_DESC_REQ                0x06
#define MT_ZDO_COMPLEX_DESC_REQ              0x07
#define MT_ZDO_USER_DESC_REQ                 0x08
#define MT_ZDO_DEVICE_ANNCE                  0x0A
#define MT_ZDO_USER_DESC_SET                 0x0B
#define MT_ZDO_SERVER_DISC_REQ               0x0C
#define MT_ZDO_END_DEVICE_BIND_REQ           0x20
#define MT_ZDO_BIND_REQ                      0x21
#define MT_ZDO_UNBIND_REQ                    0x22

#define MT_ZDO_SET_LINK_KEY                  0x23
#define MT_ZDO_REMOVE_LINK_KEY               0x24
#define MT_ZDO_GET_LINK_KEY                  0x25
#define MT_ZDO_NWK_DISCOVERY_REQ             0x26
#define MT_ZDO_JOIN_REQ                      0x27

#define MT_ZDO_MGMT_NWK_DISC_REQ              0x30
#define MT_ZDO_MGMT_LQI_REQ                  0x31
#define MT_ZDO_MGMT_RTG_REQ                  0x32
#define MT_ZDO_MGMT_BIND_REQ                 0x33
#define MT_ZDO_MGMT_LEAVE_REQ                0x34
#define MT_ZDO_MGMT_DIRECT_JOIN_REQ          0x35
#define MT_ZDO_MGMT_PERMIT_JOIN_REQ          0x36
#define MT_ZDO_MGMT_NWK_UPDATE_REQ           0x37

/* AREQ optional, but no AREQ response. */
#define MT_ZDO_MSG_CB_REGISTER               0x3E
#define MT_ZDO_MSG_CB_REMOVE                 0x3F
#define MT_ZDO_STARTUP_FROM_APP              0x40

/* AREQ from host */
#define MT_ZDO_AUTO_FIND_DESTINATION     0x41

/* AREQ to host */
#define MT_ZDO_AREQ_TO_HOST                0x80 /* Mark the start of the ZDO CId AREQs to host. */
#define MT_ZDO_NWK_ADDR_RSP                0x80
#define MT_ZDO_IEEE_ADDR_RSP               0x81
#define MT_ZDO_NODE_DESC_RSP               0x82
#define MT_ZDO_POWER_DESC_RSP              0x83
#define MT_ZDO_SIMPLE_DESC_RSP             0x84
#define MT_ZDO_ACTIVE_EP_RSP               0x85
#define MT_ZDO_MATCH_DESC_RSP              0x86

#define MT_ZDO_COMPLEX_DESC_RSP            0x90
#define MT_ZDO_USER_DESC_RSP               0x91
//                                         0x92 */ ((uint8)Discovery_Cache_req | 0x80)
#define MT_ZDO_USER_DESC_CONF              0x94
#define MT_ZDO_SERVER_DISC_RSP             0x95

#define MT_ZDO_END_DEVICE_BIND_RSP         0xA0
#define MT_ZDO_BIND_RSP                    0xA1
#define MT_ZDO_UNBIND_RSP                  0xA2

#define MT_ZDO_MGMT_NWK_DISC_RSP           0xB0
#define MT_ZDO_MGMT_LQI_RSP                0xB1
#define MT_ZDO_MGMT_RTG_RSP                0xB2
#define MT_ZDO_MGMT_BIND_RSP               0xB3
#define MT_ZDO_MGMT_LEAVE_RSP              0xB4
#define MT_ZDO_MGMT_DIRECT_JOIN_RSP        0xB5
#define MT_ZDO_MGMT_PERMIT_JOIN_RSP        0xB6

//                                        /* 0xB8 */ ((uint8)Mgmt_NWK_Update_req | 0x80)

#define MT_ZDO_STATE_CHANGE_IND              0xC0
#define MT_ZDO_END_DEVICE_ANNCE_IND          0xC1
#define MT_ZDO_MATCH_DESC_RSP_SENT           0xC2
#define MT_ZDO_STATUS_ERROR_RSP              0xC3
#define MT_ZDO_SRC_RTG_IND                   0xC4
#define MT_ZDO_BEACON_NOTIFY_IND             0xC5
#define MT_ZDO_JOIN_CNF                      0xC6
#define MT_ZDO_NWK_DISCOVERY_CNF             0xC7
#define MT_ZDO_CONCENTRATOR_IND_CB           0xC8
#define MT_ZDO_LEAVE_IND                     0xC9

#define MT_ZDO_MSG_CB_INCOMING               0xFF

// Some arbitrarily chosen value for a default error status msg.
//#define MtZdoDef_rsp                         0x0040

/*ZDO Status Responses Definitions for ZDO Startup from App*/
#define RESTORED_NETWORK 0x00
#define NEW_NETWORK 0x01
#define LEAVEANDNOTSTARTED 0x02

/*MACROS*/
#define SUCCESS 0x00
#define FAILURE 0x01
#define MAX_MTU 0x0C
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
#define BREAK_UINT32(var, ByteNum) \
                (uint8_t)((uint32_t)(((var)>>((ByteNum) * 8)) & 0x00FF))

/*typedefs*/

typedef struct
{
	uint64_t PanID;
	uint8_t LogicalChannel;
	uint8_t StackProf_ZigVer;
	uint8_t BeacOrd_SupFramOrd;
	uint8_t PermitJoin;
} NetworkListItemFormat_t;

typedef struct
{
	uint64_t ExtendedPanID;
	uint64_t ExtendedAddress;
	uint16_t NetworkAddress;
	uint8_t DevTyp_RxOnWhenIdle_Relat;
	uint8_t PermitJoining;
	uint8_t Depth;
	uint8_t LQI;
} NeighborLqiListItemFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t Status;
	uint16_t NextHop;
} RoutingTableListItemFormat_t;

typedef struct
{
	uint64_t SrcIEEEAddr;
	uint8_t SrcEndpoint;
	uint8_t ClusterID;
	uint8_t DstAddrMode;
	uint64_t DstIEEEAddr;
	uint8_t DstEndpoint;
} BindingTableListItemFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint16_t PanId;
	uint8_t LogicalChannel;
	uint8_t PermitJoining;
	uint8_t RouterCap;
	uint8_t DevCap;
	uint8_t ProtocolVer;
	uint8_t StackProf;
	uint8_t Lqi;
	uint8_t Depth;
	uint8_t UpdateId;
	uint64_t ExtendedPanId;
} BeaconListItemFormat_t;

typedef struct
{
	uint8_t IEEEAddress[8];
	uint8_t ReqType;
	uint8_t StartIndex;
} NwkAddrReqFormat_t;

typedef struct
{
	uint16_t ShortAddr;
	uint8_t ReqType;
	uint8_t StartIndex;
} IeeeAddrReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint16_t NwkAddrOfInterest;
} NodeDescReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint16_t NwkAddrOfInterest;
} PowerDescReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint16_t NwkAddrOfInterest;
	uint8_t Endpoint;
} SimpleDescReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint16_t NwkAddrOfInterest;
} ActiveEpReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint16_t NwkAddrOfInterest;
	uint16_t ProfileID;
	uint8_t NumInClusters;
	uint16_t InClusterList[16];
	uint8_t NumOutClusters;
	uint16_t OutClusterList[16];
} MatchDescReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint16_t NwkAddrOfInterest;
} ComplexDescReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint16_t NwkAddrOfInterest;
} UserDescReqFormat_t;

typedef struct
{
	uint16_t NWKAddr;
	uint8_t IEEEAddr[8];
	uint8_t Capabilities;
} DeviceAnnceFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint16_t NwkAddrOfInterest;
	uint8_t Len;
	uint8_t UserDescriptor[16];
} UserDescSetFormat_t;

typedef struct
{
	uint16_t ServerMask;
} ServerDiscReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint16_t LocalCoordinator;
	uint8_t CoordinatorIEEE[8];
	uint8_t EndPoint;
	uint16_t ProfileID;
	uint8_t NumInClusters;
	uint16_t InClusterList[16];
	uint8_t NumOutClusters;
	uint16_t OutClusterList[16];
} EndDeviceBindReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t SrcAddress[8];
	uint8_t SrcEndpoint;
	uint16_t ClusterID;
	uint8_t DstAddrMode;
	uint8_t DstAddress[8];
	uint8_t DstEndpoint;
} BindReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t SrcAddress[8];
	uint8_t SrcEndpoint;
	uint16_t ClusterID;
	uint8_t DstAddrMode;
	uint8_t DstAddress[8];
	uint8_t DstEndpoint;
} UnbindReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t ScanChannels[4];
	uint8_t ScanDuration;
	uint8_t StartIndex;
} MgmtNwkDiscReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t StartIndex;
} MgmtLqiReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t StartIndex;
} MgmtRtgReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t StartIndex;
} MgmtBindReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t DeviceAddr[8];
	uint8_t RemoveChildre_Rejoin;
} MgmtLeaveReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t DeviceAddr[8];
	uint8_t CapInfo;
} MgmtDirectJoinReqFormat_t;

typedef struct
{
	uint8_t AddrMode;
	uint16_t DstAddr;
	uint8_t Duration;
	uint8_t TCSignificance;
} MgmtPermitJoinReqFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t DstAddrMode;
	uint8_t ChannelMask[4];
	uint8_t ScanDuration;
	uint8_t ScanCount;
	uint16_t NwkManagerAddr;
} MgmtNwkUpdateReqFormat_t;

typedef struct
{
	uint16_t StartDelay;
} StartupFromAppFormat_t;

typedef struct
{
	uint8_t Endpoint;
} AutoFindDestinationFormat_t;

typedef struct
{
	uint16_t ShortAddr;
	uint8_t IEEEaddr[8];
	uint8_t LinkKeyData[16];
} SetLinkKeyFormat_t;

typedef struct
{
	uint8_t IEEEaddr[8];
} RemoveLinkKeyFormat_t;

typedef struct
{
	uint8_t IEEEaddr[8];
} GetLinkKeyFormat_t;

typedef struct
{
	uint8_t Status;
	uint64_t IEEEAddr;
	uint8_t LinkKeyData[16];
} GetLinkKeySrspFormat_t;

typedef struct
{
	uint8_t ScanChannels[4];
	uint8_t ScanDuration;
} NwkDiscoveryReqFormat_t;

typedef struct
{
	uint8_t LogicalChannel;
	uint16_t PanID;
	uint8_t ExtendedPanID[8];
	uint16_t ChosenParent;
	uint8_t ParentDepth;
	uint8_t StackProfile;
} JoinReqFormat_t;

typedef struct
{
	uint16_t ClusterID;
} MsgCbRegisterFormat_t;

typedef struct
{
	uint16_t ClusterID;
} MsgCbRemoveFormat_t;

typedef struct
{
	uint8_t Status;
	uint64_t IEEEAddr;
	uint16_t NwkAddr;
	uint8_t StartIndex;
	uint8_t NumAssocDev;
	uint16_t AssocDevList[70];
} NwkAddrRspFormat_t;

typedef struct
{
	uint8_t Status;
	uint64_t IEEEAddr;
	uint16_t NwkAddr;
	uint8_t StartIndex;
	uint8_t NumAssocDev;
	uint16_t AssocDevList[70];
} IeeeAddrRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint16_t NwkAddr;
	uint8_t LoTy_ComDescAv_UsrDesAv;
	uint8_t APSFlg_FrqBnd;
	uint8_t MACCapFlg;
	uint16_t ManufacturerCode;
	uint8_t MaxBufferSize;
	uint16_t MaxTransferSize;
	uint16_t ServerMask;
	uint16_t MaxOutTransferSize;
	uint8_t DescriptorCapabilities;
} NodeDescRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint16_t NwkAddr;
	uint8_t CurrntPwrMode_AvalPwrSrcs;
	uint8_t CurrntPwrSrc_CurrntPwrSrcLvl;
} PowerDescRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint16_t NwkAddr;
	uint8_t Len;
	uint8_t Endpoint;
	uint16_t ProfileID;
	uint16_t DeviceID;
	uint8_t DeviceVersion;
	uint8_t NumInClusters;
	uint16_t InClusterList[16];
	uint8_t NumOutClusters;
	uint16_t OutClusterList[16];
} SimpleDescRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint16_t NwkAddr;
	uint8_t ActiveEPCount;
	uint8_t ActiveEPList[77];
} ActiveEpRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint16_t NwkAddr;
	uint8_t MatchLength;
	uint8_t MatchList[77];
} MatchDescRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint16_t NwkAddr;
	uint8_t ComplexLength;
	uint8_t ComplexList[77];
} ComplexDescRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint16_t NwkAddr;
	uint8_t Len;
	uint8_t CUserDescriptor[77];
} UserDescRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint16_t NwkAddr;
} UserDescConfFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint16_t ServerMask;
} ServerDiscRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
} EndDeviceBindRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
} BindRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
} UnbindRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint8_t NetworkCount;
	uint8_t StartIndex;
	uint8_t NetworkListCount;
	NetworkListItemFormat_t NetworkList[72];
} MgmtNwkDiscRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint8_t NeighborTableEntries;
	uint8_t StartIndex;
	uint8_t NeighborLqiListCount;
	NeighborLqiListItemFormat_t NeighborLqiList[66];
} MgmtLqiRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint8_t RoutingTableEntries;
	uint8_t StartIndex;
	uint8_t RoutingTableListCount;
	RoutingTableListItemFormat_t RoutingTableList[75];
} MgmtRtgRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
	uint8_t BindingTableEntries;
	uint8_t StartIndex;
	uint8_t BindingTableListCount;
	BindingTableListItemFormat_t BindingTableList[75];
} MgmtBindRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
} MgmtLeaveRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
} MgmtDirectJoinRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
} MgmtPermitJoinRspFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint16_t NwkAddr;
	uint64_t IEEEAddr;
	uint8_t Capabilities;
} EndDeviceAnnceIndFormat_t;

typedef struct
{
	uint16_t NwkAddr;
	uint8_t NumInClusters;
	uint16_t InClusterList[16];
	uint8_t NumOutClusters;
	uint16_t OutClusterList[16];
} MatchDescRspSentFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t Status;
} StatusErrorRspFormat_t;

typedef struct
{
	uint16_t DstAddr;
	uint8_t RelayCount;
	uint16_t RelayList[255];
} SrcRtgIndFormat_t;

typedef struct
{
	uint8_t BeaconCount;
	BeaconListItemFormat_t BeaconList[21];
} BeaconNotifyIndFormat_t;

typedef struct
{
	uint8_t Status;
	uint16_t DevAddr;
	uint16_t ParentAddr;
} JoinCnfFormat_t;

typedef struct
{
	uint8_t Status;
} NwkDiscoveryCnfFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint64_t ExtAddr;
	uint8_t Request;
	uint8_t Remove;
	uint8_t Rejoin;
} LeaveIndFormat_t;

typedef struct
{
	uint16_t SrcAddr;
	uint8_t WasBroadcast;
	uint16_t ClusterID;
	uint8_t SecurityUse;
	uint8_t SeqNum;
	uint16_t MacDstAddr;
	uint8_t Status;
	uint64_t ExtAddr;
	uint16_t NwkAddr;
	uint8_t NotUsed;
} MsgCbIncomingFormat_t;

//sets the types of response of State Change Ind
typedef enum
{
	DEV_HOLD, // Initialized - not started automatically
	DEV_INIT, // Initialized - not connected to anything
	DEV_NWK_DISC, // Discovering PAN's to join
	DEV_NWK_JOINING, // Joining a PAN
	DEV_NWK_REJOIN, // ReJoining a PAN, only for end devices
	DEV_END_DEVICE_UNAUTH, // Joined but not yet authenticated by trust center
	DEV_END_DEVICE, // Started as device after authentication
	DEV_ROUTER, // Device joined, authenticated and is a router
	DEV_COORD_STARTING, // Started as Zigbee Coordinator
	DEV_ZB_COORD, // Started as Zigbee Coordinator
	DEV_NWK_ORPHAN // Device has lost information about its parent..
} devStates_t;

typedef uint8_t (*mtZdoStateChangeIndCb_t)(uint8_t zdoState);
typedef uint8_t (*mtZdoGetLinkKeyCb_t)(GetLinkKeySrspFormat_t *msg);
typedef uint8_t (*mtZdoNwkAddrRspCb_t)(NwkAddrRspFormat_t *msg);
typedef uint8_t (*mtZdoIeeeAddrRspCb_t)(IeeeAddrRspFormat_t *msg);
typedef uint8_t (*mtZdoNodeDescRspCb_t)(NodeDescRspFormat_t *msg);
typedef uint8_t (*mtZdoPowerDescRspCb_t)(PowerDescRspFormat_t *msg);
typedef uint8_t (*mtZdoSimpleDescRspCb_t)(SimpleDescRspFormat_t *msg);
typedef uint8_t (*mtZdoActiveEpRspCb_t)(ActiveEpRspFormat_t *msg);
typedef uint8_t (*mtZdoMatchDescRspCb_t)(MatchDescRspFormat_t *msg);
typedef uint8_t (*mtZdoComplexDescRspCb_t)(ComplexDescRspFormat_t *msg);
typedef uint8_t (*mtZdoUserDescRspCb_t)(UserDescRspFormat_t *msg);
typedef uint8_t (*mtZdoUserDescConfCb_t)(UserDescConfFormat_t *msg);
typedef uint8_t (*mtZdoServerDiscRspCb_t)(ServerDiscRspFormat_t *msg);
typedef uint8_t (*mtZdoEndDeviceBindRspCb_t)(EndDeviceBindRspFormat_t *msg);
typedef uint8_t (*mtZdoBindRspCb_t)(BindRspFormat_t *msg);
typedef uint8_t (*mtZdoUnbindRspCb_t)(UnbindRspFormat_t *msg);
typedef uint8_t (*mtZdoMgmtNwkDiscRspCb_t)(MgmtNwkDiscRspFormat_t *msg);
typedef uint8_t (*mtZdoMgmtLqiRspCb_t)(MgmtLqiRspFormat_t *msg);
typedef uint8_t (*mtZdoMgmtRtgRspCb_t)(MgmtRtgRspFormat_t *msg);
typedef uint8_t (*mtZdoMgmtBindRspCb_t)(MgmtBindRspFormat_t *msg);
typedef uint8_t (*mtZdoMgmtLeaveRspCb_t)(MgmtLeaveRspFormat_t *msg);
typedef uint8_t (*mtZdoMgmtDirectJoinRspCb_t)(MgmtDirectJoinRspFormat_t *msg);
typedef uint8_t (*mtZdoMgmtPermitJoinRspCb_t)(MgmtPermitJoinRspFormat_t *msg);
typedef uint8_t (*mtZdoEndDeviceAnnceIndCb_t)(EndDeviceAnnceIndFormat_t *msg);
typedef uint8_t (*mtZdoMatchDescRspSentCb_t)(MatchDescRspSentFormat_t *msg);
typedef uint8_t (*mtZdoStatusErrorRspCb_t)(StatusErrorRspFormat_t *msg);
typedef uint8_t (*mtZdoSrcRtgIndCb_t)(SrcRtgIndFormat_t *msg);
typedef uint8_t (*mtZdoBeaconNotifyIndCb_t)(BeaconNotifyIndFormat_t *msg);
typedef uint8_t (*mtZdoJoinCnfCb_t)(JoinCnfFormat_t *msg);
typedef uint8_t (*mtZdoNwkDiscoveryCnfCb_t)(NwkDiscoveryCnfFormat_t *msg);
typedef uint8_t (*mtZdoLeaveIndCb_t)(LeaveIndFormat_t *msg);
typedef uint8_t (*mtZdoMsgCbIncomingCb_t)(MsgCbIncomingFormat_t *msg);

typedef uint8_t (*mtZdoStub_t)(void);

typedef struct
{
	mtZdoNwkAddrRspCb_t pfnZdoNwkAddrRsp; // MT_ZDO_NWK_ADDR_RSP                0x80
	mtZdoIeeeAddrRspCb_t pfnZdoIeeeAddrRsp; // MT_ZDO_IEEE_ADDR_RSP               0x81
	mtZdoNodeDescRspCb_t pfnZdoNodeDescRsp; // MT_ZDO_NODE_DESC_RSP               0x82
	mtZdoPowerDescRspCb_t pfnZdoPowerDescRsp; // MT_ZDO_POWER_DESC_RSP              0x83
	mtZdoSimpleDescRspCb_t pfnZdoSimpleDescRsp; // MT_ZDO_SIMPLE_DESC_RSP             0x84
	mtZdoActiveEpRspCb_t pfnZdoActiveEpRsp; // MT_ZDO_ACTIVE_EP_RSP               0x85
	mtZdoMatchDescRspCb_t pfnZdoMatchDescRsp; // MT_ZDO_MATCH_DESC_RSP              0x86
	mtZdoComplexDescRspCb_t pfnZdoComplexDescRsp; // MT_ZDO_COMPLEX_DESC_RSP            0x87
	mtZdoUserDescRspCb_t pfnZdoUserDescRsp; // MT_ZDO_USER_DESC_RSP               0x88
	mtZdoUserDescConfCb_t pfnZdoUserDescConf; // MT_ZDO_USER_DESC_CONF              0x89
	mtZdoServerDiscRspCb_t pfnZdoServerDiscRsp; // MT_ZDO_SERVER_DISC_RSP             0x8A
	mtZdoEndDeviceBindRspCb_t pfnZdoEndDeviceBindRsp; // MT_ZDO_END_DEVICE_BIND_RSP         0xA0
	mtZdoBindRspCb_t pfnZdoBindRsp;   // MT_ZDO_BIND_RSP                    0xA1
	mtZdoUnbindRspCb_t pfnZdoUnbindRsp; // MT_ZDO_UNBIND_RSP                  0xA2
	mtZdoMgmtNwkDiscRspCb_t pfnZdoMgmtNwkDiscRsp; // MT_ZDO_MGMT_NWK_DISC_RSP           0xB0
	mtZdoMgmtLqiRspCb_t pfnZdoMgmtLqiRsp; // MT_ZDO_MGMT_LQI_RSP                0xB1
	mtZdoMgmtRtgRspCb_t pfnZdoMgmtRtgRsp; // MT_ZDO_MGMT_RTG_RSP                0xB2
	mtZdoMgmtBindRspCb_t pfnZdoMgmtBindRsp; // MT_ZDO_MGMT_BIND_RSP               0xB3
	mtZdoMgmtLeaveRspCb_t pfnZdoMgmtLeaveRsp; // MT_ZDO_MGMT_LEAVE_RSP              0xB4
	mtZdoMgmtDirectJoinRspCb_t pfnZdoMgmtDirectJoinRsp; // MT_ZDO_MGMT_DIRECT_JOIN_RSP        0xB5
	mtZdoMgmtPermitJoinRspCb_t pfnZdoMgmtPermitJoinRsp; // MT_ZDO_MGMT_PERMIT_JOIN_RSP        0xB6
	mtZdoStateChangeIndCb_t pfnmtZdoStateChangeInd;    //MT_ZDO_STATE_CHANGE_IND
	mtZdoEndDeviceAnnceIndCb_t pfnZdoEndDeviceAnnceInd; //MT_ZDO_END_DEVICE_ANNCE_IND
	mtZdoSrcRtgIndCb_t pfnZdoSrcRtgInd;                 //MT_ZDO_SRC_RTG_IND
	mtZdoBeaconNotifyIndCb_t pfnZdoBeaconNotifyInd;	//MT_ZDO_BEACON_NOTIFY_IND
	mtZdoJoinCnfCb_t pfnZdoJoinCnf;				        //MT_ZDO_JOIN_CNF
	mtZdoNwkDiscoveryCnfCb_t pfnZdoNwkDiscoveryCnf;	//MT_ZDO_NWK_DISCOVERY_CNF
	mtZdoStub_t pfnZdoConcentratorInd;              //MT_ZDO_CONCENTRATOR_IND_CB
	mtZdoLeaveIndCb_t pfnZdoLeaveInd;                   //MT_ZDO_LEAVE_IND
	mtZdoStatusErrorRspCb_t pfnZdoStatusErrorRsp; //MT_ZDO_STATUS_ERROR_RSP             0xC3
	mtZdoMatchDescRspSentCb_t pfnZdoMatchDescRspSent; //MT_ZDO_MATCH_DESC_RSP_SENT          0xC2
	mtZdoMsgCbIncomingCb_t pfnZdoMsgCbIncoming;
	mtZdoGetLinkKeyCb_t pfnZdoGetLinkKey;
} mtZdoCb_t;

void zdoRegisterCallbacks(mtZdoCb_t cbs);
uint8_t zdoInit(void);
uint8_t zdoNwkAddrReq(NwkAddrReqFormat_t *req);
uint8_t zdoIeeeAddrReq(IeeeAddrReqFormat_t *req);
uint8_t zdoNodeDescReq(NodeDescReqFormat_t *req);
uint8_t zdoPowerDescReq(PowerDescReqFormat_t *req);
uint8_t zdoSimpleDescReq(SimpleDescReqFormat_t *req);
uint8_t zdoActiveEpReq(ActiveEpReqFormat_t *req);
uint8_t zdoMatchDescReq(MatchDescReqFormat_t *req);
uint8_t zdoComplexDescReq(ComplexDescReqFormat_t *req);
uint8_t zdoUserDescReq(UserDescReqFormat_t *req);
uint8_t zdoDeviceAnnce(DeviceAnnceFormat_t *req);
uint8_t zdoUserDescSet(UserDescSetFormat_t *req);
uint8_t zdoServerDiscReq(ServerDiscReqFormat_t *req);
uint8_t zdoEndDeviceBindReq(EndDeviceBindReqFormat_t *req);
uint8_t zdoBindReq(BindReqFormat_t *req);
uint8_t zdoUnbindReq(UnbindReqFormat_t *req);
uint8_t zdoMgmtNwkDiscReq(MgmtNwkDiscReqFormat_t *req);
uint8_t zdoMgmtLqiReq(MgmtLqiReqFormat_t *req);
uint8_t zdoMgmtRtgReq(MgmtRtgReqFormat_t *req);
uint8_t zdoMgmtBindReq(MgmtBindReqFormat_t *req);
uint8_t zdoMgmtLeaveReq(MgmtLeaveReqFormat_t *req);
uint8_t zdoMgmtDirectJoinReq(MgmtDirectJoinReqFormat_t *req);
uint8_t zdoMgmtPermitJoinReq(MgmtPermitJoinReqFormat_t *req);
uint8_t zdoMgmtNwkUpdateReq(MgmtNwkUpdateReqFormat_t *req);
uint8_t zdoStartupFromApp(StartupFromAppFormat_t *req);
uint8_t zdoAutoFindDestination(AutoFindDestinationFormat_t *req);
uint8_t zdoSetLinkKey(SetLinkKeyFormat_t *req);
uint8_t zdoRemoveLinkKey(RemoveLinkKeyFormat_t *req);
uint8_t zdoGetLinkKey(GetLinkKeyFormat_t *req);
uint8_t zdoNwkDiscoveryReq(NwkDiscoveryReqFormat_t *req);
uint8_t zdoJoinReq(JoinReqFormat_t *req);
uint8_t zdoMsgCbRegister(MsgCbRegisterFormat_t *req);
uint8_t zdoMsgCbRemove(MsgCbRemoveFormat_t *req);

void zdoProcess(uint8_t *rpcBuff, uint8_t rpcLen);

#ifdef __cplusplus
}
#endif

#endif /* ZBMTZDO_H */

