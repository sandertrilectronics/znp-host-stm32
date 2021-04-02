/*
 * mtsys.h
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
#ifndef ZBMTSYS_H
#define ZBMTSYS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/***************************************************************************************************
 * SYS COMMANDS
 ***************************************************************************************************/

/* AREQ from host */
#define MT_SYS_RESET_REQ                     0x00

/* SREQ/SRSP */
#define MT_SYS_PING                          0x01
#define MT_SYS_VERSION                       0x02
#define MT_SYS_SET_EXTADDR                   0x03
#define MT_SYS_GET_EXTADDR                   0x04
#define MT_SYS_RAM_READ                      0x05
#define MT_SYS_RAM_WRITE                     0x06
#define MT_SYS_OSAL_NV_ITEM_INIT             0x07
#define MT_SYS_OSAL_NV_READ                  0x08
#define MT_SYS_OSAL_NV_WRITE                 0x09
#define MT_SYS_OSAL_START_TIMER              0x0A
#define MT_SYS_OSAL_STOP_TIMER               0x0B
#define MT_SYS_RANDOM                        0x0C
#define MT_SYS_ADC_READ                      0x0D
#define MT_SYS_GPIO                          0x0E
#define MT_SYS_STACK_TUNE                    0x0F
#define MT_SYS_SET_TIME                      0x10
#define MT_SYS_GET_TIME                      0x11
#define MT_SYS_OSAL_NV_DELETE                0x12
#define MT_SYS_OSAL_NV_LENGTH                0x13
#define MT_SYS_SET_TX_POWER                  0x14

/* AREQ to host */
#define MT_SYS_RESET_IND                     0x80
#define MT_SYS_OSAL_TIMER_EXPIRED            0x81

// OSAL NV item IDs
#define ZCD_NV_EXTADDR                    0x0001
#define ZCD_NV_BOOTCOUNTER                0x0002
#define ZCD_NV_STARTUP_OPTION             0x0003
#define ZCD_NV_START_DELAY                0x0004

// NWK Layer NV item IDs
#define ZCD_NV_NIB                        0x0021
#define ZCD_NV_DEVICE_LIST                0x0022
#define ZCD_NV_ADDRMGR                    0x0023
#define ZCD_NV_POLL_RATE                  0x0024
#define ZCD_NV_QUEUED_POLL_RATE           0x0025
#define ZCD_NV_RESPONSE_POLL_RATE         0x0026
#define ZCD_NV_REJOIN_POLL_RATE           0x0027
#define ZCD_NV_DATA_RETRIES               0x0028
#define ZCD_NV_POLL_FAILURE_RETRIES       0x0029
#define ZCD_NV_STACK_PROFILE              0x002A
#define ZCD_NV_INDIRECT_MSG_TIMEOUT       0x002B
#define ZCD_NV_ROUTE_EXPIRY_TIME          0x002C
#define ZCD_NV_EXTENDED_PAN_ID            0x002D
#define ZCD_NV_BCAST_RETRIES              0x002E
#define ZCD_NV_PASSIVE_ACK_TIMEOUT        0x002F
#define ZCD_NV_BCAST_DELIVERY_TIME        0x0030
#define ZCD_NV_NWK_MODE                   0x0031
#define ZCD_NV_CONCENTRATOR_ENABLE        0x0032
#define ZCD_NV_CONCENTRATOR_DISCOVERY     0x0033
#define ZCD_NV_CONCENTRATOR_RADIUS        0x0034
#define ZCD_NV_CONCENTRATOR_RC            0x0036
#define ZCD_NV_NWK_MGR_MODE               0x0037
#define ZCD_NV_SRC_RTG_EXPIRY_TIME        0x0038
#define ZCD_NV_ROUTE_DISCOVERY_TIME       0x0039
#define ZCD_NV_NWK_ACTIVE_KEY_INFO        0x003A
#define ZCD_NV_NWK_ALTERN_KEY_INFO        0x003B
#define ZCD_NV_ROUTER_OFF_ASSOC_CLEANUP   0x003C
#define ZCD_NV_NWK_LEAVE_REQ_ALLOWED      0x003D
#define ZCD_NV_NWK_CHILD_AGE_ENABLE       0x003E
#define ZCD_NV_DEVICE_LIST_KA_TIMEOUT     0x003F

// APS Layer NV item IDs
#define ZCD_NV_BINDING_TABLE              0x0041
#define ZCD_NV_GROUP_TABLE                0x0042
#define ZCD_NV_APS_FRAME_RETRIES          0x0043
#define ZCD_NV_APS_ACK_WAIT_DURATION      0x0044
#define ZCD_NV_APS_ACK_WAIT_MULTIPLIER    0x0045
#define ZCD_NV_BINDING_TIME               0x0046
#define ZCD_NV_APS_USE_EXT_PANID          0x0047
#define ZCD_NV_APS_USE_INSECURE_JOIN      0x0048
#define ZCD_NV_COMMISSIONED_NWK_ADDR      0x0049

#define ZCD_NV_APS_NONMEMBER_RADIUS       0x004B     // Multicast non_member radius#define ZCD_NV_APS_LINK_KEY_TABLE         0x004C
#define ZCD_NV_APS_DUPREJ_TIMEOUT_INC     0x004D
#define ZCD_NV_APS_DUPREJ_TIMEOUT_COUNT   0x004E
#define ZCD_NV_APS_DUPREJ_TABLE_SIZE      0x004F

// Security NV Item IDs
#define ZCD_NV_SECURITY_LEVEL             0x0061
#define ZCD_NV_PRECFGKEY                  0x0062
#define ZCD_NV_PRECFGKEYS_ENABLE          0x0063
#define ZCD_NV_SECURITY_MODE              0x0064
#define ZCD_NV_SECURE_PERMIT_JOIN         0x0065
#define ZCD_NV_APS_LINK_KEY_TYPE          0x0066
#define ZCD_NV_APS_ALLOW_R19_SECURITY     0x0067

#define ZCD_NV_IMPLICIT_CERTIFICATE       0x0069
#define ZCD_NV_DEVICE_PRIVATE_KEY         0x006A
#define ZCD_NV_CA_PUBLIC_KEY              0x006B

#define ZCD_NV_USE_DEFAULT_TCLK           0x006D
#define ZCD_NV_TRUSTCENTER_ADDR           0x006E
#define ZCD_NV_RNG_COUNTER                0x006F
#define ZCD_NV_RANDOM_SEED                0x0070

// ZDO NV Item IDs
#define ZCD_NV_USERDESC                   0x0081
#define ZCD_NV_NWKKEY                     0x0082
#define ZCD_NV_PANID                      0x0083
#define ZCD_NV_CHANLIST                   0x0084
#define ZCD_NV_LEAVE_CTRL                 0x0085
#define ZCD_NV_SCAN_DURATION              0x0086
#define ZCD_NV_LOGICAL_TYPE               0x0087
#define ZCD_NV_NWKMGR_MIN_TX              0x0088
#define ZCD_NV_NWKMGR_ADDR                0x0089

#define ZCD_NV_ZDO_DIRECT_CB              0x008F

// ZCL NV item IDs
#define ZCD_NV_SCENE_TABLE                0x0091
#define ZCD_NV_MIN_FREE_NWK_ADDR          0x0092
#define ZCD_NV_MAX_FREE_NWK_ADDR          0x0093
#define ZCD_NV_MIN_FREE_GRP_ID            0x0094
#define ZCD_NV_MAX_FREE_GRP_ID            0x0095
#define ZCD_NV_MIN_GRP_IDS                0x0096
#define ZCD_NV_MAX_GRP_IDS                0x0097

// Non-standard NV item IDs
#define ZCD_NV_SAPI_ENDPOINT              0x00A1

// NV Items Reserved for Commissioning Cluster Startup Attribute Set (SAS):
// 0x00B1 - 0x00BF: Parameters related to APS and NWK layers
// 0x00C1 - 0x00CF: Parameters related to Security
// 0x00D1 - 0x00DF: Current key parameters
#define ZCD_NV_SAS_SHORT_ADDR             0x00B1
#define ZCD_NV_SAS_EXT_PANID              0x00B2
#define ZCD_NV_SAS_PANID                  0x00B3
#define ZCD_NV_SAS_CHANNEL_MASK           0x00B4
#define ZCD_NV_SAS_PROTOCOL_VER           0x00B5
#define ZCD_NV_SAS_STACK_PROFILE          0x00B6
#define ZCD_NV_SAS_STARTUP_CTRL           0x00B7

#define ZCD_NV_SAS_TC_ADDR                0x00C1
#define ZCD_NV_SAS_TC_MASTER_KEY          0x00C2
#define ZCD_NV_SAS_NWK_KEY                0x00C3
#define ZCD_NV_SAS_USE_INSEC_JOIN         0x00C4
#define ZCD_NV_SAS_PRECFG_LINK_KEY        0x00C5
#define ZCD_NV_SAS_NWK_KEY_SEQ_NUM        0x00C6
#define ZCD_NV_SAS_NWK_KEY_TYPE           0x00C7
#define ZCD_NV_SAS_NWK_MGR_ADDR           0x00C8

#define ZCD_NV_SAS_CURR_TC_MASTER_KEY     0x00D1
#define ZCD_NV_SAS_CURR_NWK_KEY           0x00D2
#define ZCD_NV_SAS_CURR_PRECFG_LINK_KEY   0x00D3

// NV Items Reserved for Trust Center Link Key Table entries
// 0x0101 - 0x01FF
#define ZCD_NV_TCLK_TABLE_START           0x0101
#define ZCD_NV_TCLK_TABLE_END             0x01FF

// NV Items Reserved for APS Link Key Table entries
// 0x0201 - 0x02FF
#define ZCD_NV_APS_LINK_KEY_DATA_START    0x0201     // APS key data#define ZCD_NV_APS_LINK_KEY_DATA_END      0x02FF

// NV Items Reserved for Master Key Table entries
// 0x0301 - 0x03FF
#define ZCD_NV_MASTER_KEY_DATA_START      0x0301     // Master key data#define ZCD_NV_MASTER_KEY_DATA_END        0x03FF

// NV Items Reserved for applications (user applications)
// 0x0401 � 0x0FFF

// ZCD_NV_STARTUP_OPTION values
//   These are bit weighted - you can OR these together.
//   Setting one of these bits will set their associated NV items
//   to code initialized values.
#define ZCD_STARTOPT_DEFAULT_CONFIG_STATE  0x01
#define ZCD_STARTOPT_DEFAULT_NETWORK_STATE 0x02
#define ZCD_STARTOPT_AUTO_START            0x04
#define ZCD_STARTOPT_CLEAR_CONFIG   ZCD_STARTOPT_DEFAULT_CONFIG_STATE
#define ZCD_STARTOPT_CLEAR_STATE    ZCD_STARTOPT_DEFAULT_NETWORK_STATE

// ZCD_LOGICAL_TYPE values
#define DEVICETYPE_COORDINATOR 0x00
#define DEVICETYPE_ROUTER 0x01
#define DEVICETYPE_ENDDEVICE 0x02

#define ZCL_KE_IMPLICIT_CERTIFICATE_LEN    48
#define ZCL_KE_CA_PUBLIC_KEY_LEN           22
#define ZCL_KE_DEVICE_PRIVATE_KEY_LEN      21

typedef struct
{
	uint16_t Capabilities;
} PingSrspFormat_t;

typedef struct
{
	uint8_t ExtAddr[8];
} SetExtAddrFormat_t;

typedef struct
{
	uint64_t ExtAddr;
} GetExtAddrSrspFormat_t;

typedef struct
{
	uint16_t Address;
	uint8_t Len;
} RamReadFormat_t;

typedef struct
{
	uint8_t Status;
	uint8_t Len;
	uint8_t Value[128];
} RamReadSrspFormat_t;

typedef struct
{
	uint16_t Address;
	uint8_t Len;
	uint8_t Value[128];
} RamWriteFormat_t;

typedef struct
{
	uint8_t Type;
} ResetReqFormat_t;

typedef struct
{
	uint8_t Reason;
	uint8_t TransportRev;
	uint8_t ProductId;
	uint8_t MajorRel;
	uint8_t MinorRel;
	uint8_t HwRev;
} ResetIndFormat_t;

typedef struct
{
	uint8_t TransportRev;
	uint8_t Product;
	uint8_t MajorRel;
	uint8_t MinorRel;
	uint8_t MaintRel;
} VersionSrspFormat_t;

typedef struct
{
	uint16_t Id;
	uint8_t Offset;
} OsalNvReadFormat_t;

typedef struct
{
	uint8_t Status;
	uint8_t Len;
	uint8_t Value[248];
} OsalNvReadSrspFormat_t;

typedef struct
{
	uint16_t Id;
	uint8_t Offset;
	uint8_t Len;
	uint8_t Value[246];
} OsalNvWriteFormat_t;

typedef struct
{
	uint16_t Id;
	uint16_t ItemLen;
	uint8_t InitLen;
	uint8_t InitData[245];
} OsalNvItemInitFormat_t;

typedef struct
{
	uint16_t Id;
	uint16_t ItemLen;
} OsalNvDeleteFormat_t;

typedef struct
{
	uint16_t Id;
} OsalNvLengthFormat_t;

typedef struct
{
	uint16_t ItemLen;
} OsalNvLengthSrspFormat_t;

typedef struct
{
	uint8_t Id;
	uint16_t Timeout;
} OsalStartTimerFormat_t;

typedef struct
{
	uint8_t Id;
} OsalStopTimerFormat_t;

typedef struct
{
	uint8_t Id;
} OsalTimerExpiredFormat_t;

typedef struct
{
	uint8_t Operation;
	uint8_t Value;
} StackTuneFormat_t;

typedef struct
{
	uint8_t Value;
} StackTuneSrspFormat_t;

typedef struct
{
	uint8_t Channel;
	uint8_t Resolution;
} AdcReadFormat_t;

typedef struct
{
	uint16_t Value;
} AdcReadSrspFormat_t;

typedef struct
{
	uint8_t Operation;
	uint8_t Value;
} GpioFormat_t;

typedef struct
{
	uint8_t Value;
} GpioSrspFormat_t;

typedef struct
{
	uint16_t Value;
} RandomSrspFormat_t;

typedef struct
{
	uint8_t UTCTime[4];
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t Month;
	uint8_t Day;
	uint16_t Year;
} SetTimeFormat_t;

typedef struct
{
	uint32_t UTCTime;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t Month;
	uint8_t Day;
	uint16_t Year;
} GetTimeSrspFormat_t;

typedef struct
{
	uint8_t TxPower;
} SetTxPowerFormat_t;

typedef struct
{
	uint8_t TxPower;
} SetTxPowerSrspFormat_t;

//typedef uint8_t (*mtSysResetInd_t)(uint8_t resetReason, uint8_t version[5]);

typedef uint8_t (*mtSysPingSrspCb_t)(PingSrspFormat_t *msg);
typedef uint8_t (*mtSysGetExtAddrSrspCb_t)(GetExtAddrSrspFormat_t *msg);
typedef uint8_t (*mtSysRamReadSrspCb_t)(RamReadSrspFormat_t *msg);
typedef uint8_t (*mtSysResetIndCb_t)(ResetIndFormat_t *msg);
typedef uint8_t (*mtSysVersionSrspCb_t)(VersionSrspFormat_t *msg);
typedef uint8_t (*mtSysOsalNvReadSrspCb_t)(OsalNvReadSrspFormat_t *msg);
typedef uint8_t (*mtSysOsalNvLengthSrspCb_t)(OsalNvLengthSrspFormat_t *msg);
typedef uint8_t (*mtSysOsalTimerExpiredCb_t)(OsalTimerExpiredFormat_t *msg);
typedef uint8_t (*mtSysStackTuneSrspCb_t)(StackTuneSrspFormat_t *msg);
typedef uint8_t (*mtSysAdcReadSrspCb_t)(AdcReadSrspFormat_t *msg);
typedef uint8_t (*mtSysGpioSrspCb_t)(GpioSrspFormat_t *msg);
typedef uint8_t (*mtSysRandomSrspCb_t)(RandomSrspFormat_t *msg);
typedef uint8_t (*mtSysGetTimeSrspCb_t)(GetTimeSrspFormat_t *msg);
typedef uint8_t (*mtSysSetTxPowerSrspCb_t)(SetTxPowerSrspFormat_t *msg);

typedef uint8_t (*mtSysStub_t)(void);

typedef struct
{
	//mtSysResetInd_t pfnSysResetInd; //MT_SYS_RESET_IND                     0x80
	mtSysPingSrspCb_t pfnSysPingSrsp;
	mtSysGetExtAddrSrspCb_t pfnSysGetExtAddrSrsp;
	mtSysRamReadSrspCb_t pfnSysRamReadSrsp;
	mtSysResetIndCb_t pfnSysResetInd;
	mtSysVersionSrspCb_t pfnSysVersionSrsp;
	mtSysOsalNvReadSrspCb_t pfnSysOsalNvReadSrsp;
	mtSysOsalNvLengthSrspCb_t pfnSysOsalNvLengthSrsp;
	mtSysOsalTimerExpiredCb_t pfnSysOsalTimerExpired;
	mtSysStackTuneSrspCb_t pfnSysStackTuneSrsp;
	mtSysAdcReadSrspCb_t pfnSysAdcReadSrsp;
	mtSysGpioSrspCb_t pfnSysGpioSrsp;
	mtSysRandomSrspCb_t pfnSysRandomSrsp;
	mtSysGetTimeSrspCb_t pfnSysGetTimeSrsp;
	mtSysSetTxPowerSrspCb_t pfnSysSetTxPowerSrsp;
} mtSysCb_t;

/*MACROS*/
#define SUCCESS 0x00
#define FAILURE 0x01
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
#define BREAK_UINT32(var, ByteNum) \
                (uint8_t)((uint32_t)(((var)>>((ByteNum) * 8)) & 0x00FF))

void sysRegisterCallbacks(mtSysCb_t cbs);
void sysProcess(uint8_t *rpcBuff, uint8_t rpcLen);
//uint8_t sysNvWrite(uint16_t NvItemId, uint8_t offset, uint8_t *data,
//		uint8_t dataLen);
//uint8_t sysNvRead(uint16_t NvItemId, uint8_t offset, uint8_t *data,
//		uint8_t dataLen);
//uint8_t sysGetExtAddr(uint8_t ieee[8]);
uint8_t sysPing(void);
uint8_t sysSetExtAddr(SetExtAddrFormat_t *req);
uint8_t sysGetExtAddr(void);
uint8_t sysRamRead(RamReadFormat_t *req);
uint8_t sysRamWrite(RamWriteFormat_t *req);
uint8_t sysResetReq(ResetReqFormat_t *req);
uint8_t sysVersion(void);
uint8_t sysOsalNvRead(OsalNvReadFormat_t *req);
uint8_t sysOsalNvWrite(OsalNvWriteFormat_t *req);
uint8_t sysOsalNvItemInit(OsalNvItemInitFormat_t *req);
uint8_t sysOsalNvDelete(OsalNvDeleteFormat_t *req);
uint8_t sysOsalNvLength(OsalNvLengthFormat_t *req);
uint8_t sysOsalStartTimer(OsalStartTimerFormat_t *req);
uint8_t sysOsalStopTimer(OsalStopTimerFormat_t *req);
uint8_t sysStackTune(StackTuneFormat_t *req);
uint8_t sysAdcRead(AdcReadFormat_t *req);
uint8_t sysGpio(GpioFormat_t *req);
uint8_t sysRandom(void);
uint8_t sysSetTime(SetTimeFormat_t *req);
uint8_t sysGetTime(void);
uint8_t sysSetTxPower(SetTxPowerFormat_t *req);

uint8_t sysReset(uint8_t resetType);

#ifdef __cplusplus
}
#endif

#endif /* ZBMTSYS_H */
