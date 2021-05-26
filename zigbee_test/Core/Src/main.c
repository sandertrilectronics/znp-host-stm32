/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "rpc.h"
#include "mtSys.h"
#include "mtZdo.h"
#include "mtAf.h"
#include "mtAppCfg.h"
#include "mtSys.h"
#include "mtParser.h"
#include "mtUtil.h"
#include "rpcTransport.h"
#include "dbgPrint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static SemaphoreHandle_t dbg_sem;

void log_print(const char *fmt, ...) {
	// small local working buffer
	static char working_buffer[256];

	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
		// take semaphore
		if (xSemaphoreTake(dbg_sem, 1000) == pdFALSE)
			return;

		// append parameters
		va_list args;
		va_start(args, fmt);
		vsnprintf(working_buffer, 256, fmt, args);
		va_end(args);

		// send data
		HAL_UART_Transmit(&huart2, (char*) working_buffer, strlen(working_buffer), 100);

		// Give semaphore back
		xSemaphoreGive(dbg_sem);
	}
	else {
		// append parameters
		va_list args;
		va_start(args, fmt);
		vsnprintf(working_buffer, 256, fmt, args);
		va_end(args);

		// send data
		HAL_UART_Transmit(&huart2, (char*) working_buffer, strlen(working_buffer), 100);
	}
}

/********************************************************************
 * START OF SYS CALL BACK FUNCTIONS
 */
static uint8_t mtSysResetIndCb(ResetIndFormat_t *msg) {
	log_print("ZNP Version: %d.%d.%d\n", msg->MajorRel, msg->MinorRel, msg->HwRev);
	return 0;
}

static uint8_t mtVersionIndCb(VersionSrspFormat_t *msg) {
	log_print("Version: %d %d %d %d %d %d\n", msg->MaintRel, msg->MajorRel, msg->MinorRel, msg->Product, msg->TransportRev);
	return 0;
}

static mtSysCb_t mtSysCb = { NULL, NULL, NULL, mtSysResetIndCb, mtVersionIndCb, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };
/********************************************************************
 * START OF ZDO CALL BACK FUNCTIONS
 */

/********************************************************************
 * @fn     Callback function for ZDO State Change Indication
 * @brief  receives the AREQ status and specifies the change ZDO state
 *
 * @param  uint8 zdoState
 *
 * @return SUCCESS or FAILURE
 */
static uint8_t mtZdoStateChangeIndCb(uint8_t newDevState) {
	switch (newDevState) {
		case DEV_HOLD:
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: Initialized - not started automatically\n");
			break;
		case DEV_INIT:
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: Initialized - not connected to anything\n");
			break;
		case DEV_NWK_DISC:
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: Discovering PAN's to join\n");
			log_print("Network Discovering\n");
			break;
		case DEV_NWK_JOINING:
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: Joining a PAN\n");
			log_print("Network Joining\n");
			break;
		case DEV_NWK_REJOIN:
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: ReJoining a PAN, only for end devices\n");
			log_print("Network Rejoining\n");
			break;
		case DEV_END_DEVICE_UNAUTH:
			log_print("Network Authenticating\n");
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: Joined but not yet authenticated by trust center\n");
			break;
		case DEV_END_DEVICE:
			log_print("Network Joined\n");
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: Started as device after authentication\n");
			break;
		case DEV_ROUTER:
			log_print("Network Joined\n");
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: Device joined, authenticated and is a router\n");
			break;
		case DEV_COORD_STARTING:
			log_print("Network Starting\n");
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: Starting as Zigbee Coordinator...\n");
			break;
		case DEV_ZB_COORD:
			log_print("Network Started\n");
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: Started as Zigbee Coordinator\n");
			break;
		case DEV_NWK_ORPHAN:
			log_print("Network Orphaned\n");
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: Device has lost information about its parent\n");
			break;
		default:
			dbg_print(PRINT_LEVEL_INFO, "mtZdoStateChangeIndCb: unknown state");
			break;
	}

	//devState = (devStates_t) newDevState;

	return SUCCESS;
}

static uint8_t mtZdoSimpleDescRspCb(SimpleDescRspFormat_t *msg) {
	if (msg->Status == MT_RPC_SUCCESS) {
		log_print("\tEndpoint: 0x%02X\n", msg->Endpoint);
		log_print("\tProfileID: 0x%04X\n", msg->ProfileID);
		log_print("\tDeviceID: 0x%04X\n", msg->DeviceID);
		log_print("\tDeviceVersion: 0x%02X\n", msg->DeviceVersion);
		log_print("\tNumInClusters: %d\n", msg->NumInClusters);
		uint32_t i;
		for (i = 0; i < msg->NumInClusters; i++) {
			log_print("\t\tInClusterList[%d]: 0x%04X\n", i, msg->InClusterList[i]);
		}
		log_print("\tNumOutClusters: %d\n", msg->NumOutClusters);
		for (i = 0; i < msg->NumOutClusters; i++) {
			log_print("\t\tOutClusterList[%d]: 0x%04X\n", i, msg->OutClusterList[i]);
		}
		log_print("\n");
	}
	else {
		log_print("SimpleDescRsp Status: FAIL 0x%02X\n", msg->Status);
	}

	return msg->Status;
}

static uint8_t mtZdoMgmtLqiRspCb(MgmtLqiRspFormat_t *msg) {
	/*uint8_t devType = 0;
	 uint8_t devRelation = 0;
	 MgmtLqiReqFormat_t req;
	 if (msg->Status == MT_RPC_SUCCESS) {
	 nodeList[nodeCount].NodeAddr = msg->SrcAddr;
	 nodeList[nodeCount].Type = (msg->SrcAddr == 0 ?
	 DEVICETYPE_COORDINATOR :
	 DEVICETYPE_ROUTER);
	 nodeList[nodeCount].ChildCount = 0;
	 uint32_t i;
	 for (i = 0; i < msg->NeighborLqiListCount; i++) {
	 devType = msg->NeighborLqiList[i].DevTyp_RxOnWhenIdle_Relat & 3;
	 devRelation = ((msg->NeighborLqiList[i].DevTyp_RxOnWhenIdle_Relat >> 4) & 7);
	 if (devRelation == 1 || devRelation == 3) {
	 uint8_t cCount = nodeList[nodeCount].ChildCount;
	 nodeList[nodeCount].childs[cCount].ChildAddr = msg->NeighborLqiList[i].NetworkAddress;
	 nodeList[nodeCount].childs[cCount].Type = devType;
	 nodeList[nodeCount].ChildCount++;
	 if (devType == DEVICETYPE_ROUTER) {
	 req.DstAddr = msg->NeighborLqiList[i].NetworkAddress;
	 req.StartIndex = 0;
	 zdoMgmtLqiReq(&req);
	 }
	 }
	 }
	 nodeCount++;

	 }
	 else {
	 log_print("MgmtLqiRsp Status: FAIL 0x%02X\n", msg->Status);
	 }

	 return msg->Status;*/
	return 0;
}

static uint8_t mtZdoActiveEpRspCb(ActiveEpRspFormat_t *msg) {
	//SimpleDescReqFormat_t simReq;
	log_print("NwkAddr: 0x%04X\n", msg->NwkAddr);
	if (msg->Status == MT_RPC_SUCCESS) {
		log_print("Number of Endpoints: %d\nActive Endpoints: ", msg->ActiveEPCount);
		uint32_t i;
		for (i = 0; i < msg->ActiveEPCount; i++) {
			log_print("0x%02X\t", msg->ActiveEPList[i]);

		}
		log_print("\n");
	}
	else {
		log_print("ActiveEpRsp Status: FAIL 0x%02X\n", msg->Status);
	}

	return msg->Status;
}

static uint8_t mtZdoEndDeviceAnnceIndCb(EndDeviceAnnceIndFormat_t *msg) {
	// new device
	log_print("New device joined network:\n");
	log_print("SrcAddr: %04x\n", msg->SrcAddr);
	log_print("NwkAddr: %04x\n", msg->NwkAddr);
	uint32_t top = msg->IEEEAddr >> 32;
	uint32_t bot = msg->IEEEAddr & 0xffffffff;
	log_print("IEEEAddr: %08x%08x\r\n", top, bot);
	log_print("Capabilities: %02x\n", msg->Capabilities);

	// check if the endpoint is active
	ActiveEpReqFormat_t actReq;
	actReq.DstAddr = msg->NwkAddr;
	actReq.NwkAddrOfInterest = msg->NwkAddr;
	zdoActiveEpReq(&actReq);

	//
	return 0;
}

static mtZdoCb_t mtZdoCb = { //
		NULL,       						// MT_ZDO_NWK_ADDR_RSP
				NULL,      					// MT_ZDO_IEEE_ADDR_RSP
				NULL,      					// MT_ZDO_NODE_DESC_RSP
				NULL,     					// MT_ZDO_POWER_DESC_RSP
				mtZdoSimpleDescRspCb,   	// MT_ZDO_SIMPLE_DESC_RSP
				mtZdoActiveEpRspCb,     	// MT_ZDO_ACTIVE_EP_RSP
				NULL,     					// MT_ZDO_MATCH_DESC_RSP
				NULL,   					// MT_ZDO_COMPLEX_DESC_RSP
				NULL,      					// MT_ZDO_USER_DESC_RSP
				NULL,     					// MT_ZDO_USER_DESC_CONF
				NULL,    					// MT_ZDO_SERVER_DISC_RSP
				NULL, 						// MT_ZDO_END_DEVICE_BIND_RSP
				NULL,          				// MT_ZDO_BIND_RSP
				NULL,       				// MT_ZDO_UNBIND_RSP
				NULL,   					// MT_ZDO_MGMT_NWK_DISC_RSP
				mtZdoMgmtLqiRspCb,      	// MT_ZDO_MGMT_LQI_RSP
				NULL,       				// MT_ZDO_MGMT_RTG_RSP
				NULL,      					// MT_ZDO_MGMT_BIND_RSP
				NULL,     					// MT_ZDO_MGMT_LEAVE_RSP
				NULL,     					// MT_ZDO_MGMT_DIRECT_JOIN_RSP
				NULL,     					// MT_ZDO_MGMT_PERMIT_JOIN_RSP
				mtZdoStateChangeIndCb,   	// MT_ZDO_STATE_CHANGE_IND
				mtZdoEndDeviceAnnceIndCb,	// MT_ZDO_END_DEVICE_ANNCE_IND
				NULL,        				// MT_ZDO_SRC_RTG_IND
				NULL,						// MT_ZDO_BEACON_NOTIFY_IND
				NULL,						// MT_ZDO_JOIN_CNF
				NULL,	 					// MT_ZDO_NWK_DISCOVERY_CNF
				NULL,                   	// MT_ZDO_CONCENTRATOR_IND_CB
				NULL,       				// MT_ZDO_LEAVE_IND
				NULL,   					// MT_ZDO_STATUS_ERROR_RSP
				NULL, 						// MT_ZDO_MATCH_DESC_RSP_SENT
				NULL,						//
				NULL 						//
		};
/********************************************************************
 * AF CALL BACK FUNCTIONS
 */

static uint8_t mtAfDataConfirmCb(DataConfirmFormat_t *msg) {
	if (msg->Status == MT_RPC_SUCCESS) {
		log_print("Message transmited Succesfully!\n");
	}
	else {
		log_print("Message failed to transmit\n");
	}
	return msg->Status;
}

static uint8_t mtAfIncomingMsgCb(IncomingMsgFormat_t *msg) {
	log_print("\nIncoming Message from Endpoint 0x%02X and Address 0x%04X:\n", msg->SrcEndpoint, msg->SrcAddr);
	for (uint8_t i = 0; i < msg->Len; i++)
		log_print("%02x ", msg->Data[i]);
	;

	return 0;
}

static mtAfCb_t mtAfCb = { //
		mtAfDataConfirmCb,			// MT_AF_DATA_CONFIRM
				mtAfIncomingMsgCb,	// MT_AF_INCOMING_MSG
				NULL,				// MT_AF_INCOMING_MSG_EXT
				NULL,				// MT_AF_DATA_RETRIEVE
				NULL,			    // MT_AF_REFLECT_ERROR
		};

////////////////////////////////////////////////////

uint8_t mtAppCfgCommissioningNotifyCb(appCfgCommissioningNotifyFormat_t *msg) {
	log_print("Commissioning notify\r\n");
	log_print("Status: %02x\r\n", msg->status);
	log_print("Mode: %02x\r\n", msg->commissioningMode1);
	log_print("Mode: %02x\r\n", msg->commissioningMode2);
	return 0;
}

uint8_t mtAppCfgSetChannelCb(appCfgSetChannelFormat_t *msg) {
	log_print("Set channel response: %02x (%s)\r\n", msg->success, (msg->success) ? "ERROR" : "SUCCESS");
	return 0;
}

uint8_t mtAppCfgCommissioningStartCb(appCfgStartCommissioningStart_t *msg) {
	log_print("Commissioning start response: %02x (%s)\r\n", msg->success, (msg->success) ? "ERROR" : "SUCCESS");
	return 0;
}

static mtAppCfgCb_t mtAppCfgCb = { //
		mtAppCfgCommissioningNotifyCb, 			//
				mtAppCfgSetChannelCb, 			//
				mtAppCfgCommissioningStartCb 	//
		};

/********************************************************************
 * START OF UTIL CALL BACK FUNCTIONS
 */
static uint8_t mtUtilGetDeviceInfoCb(utilGetDeviceInfoFormat_t *msg) {
	log_print("Get Info Response\r\n");
	log_print("Success: %02x\r\n", msg->success);
	uint32_t top = msg->ieee_addr >> 32;
	uint32_t bot = msg->ieee_addr & 0xffffffff;
	log_print("IEEE Addr: %08x%08x\r\n", top, bot);
	log_print("Short Addr: %04x\r\n", msg->short_addr);
	log_print("Device Type: %02x\r\n", msg->device_type);
	log_print("Device state: %02x\r\n", msg->device_state);
	log_print("Ass Dev Cnt: %02x\r\n", msg->ass_device_cnt);
	for (uint8_t i = 0; i < msg->ass_device_cnt; i++)
		log_print("Ass Dev %d: %04x\r\n", i, msg->ass_device_list[i]);
	return 0;
}

static mtUtilCb_t mtUtilCb = { //
		mtUtilGetDeviceInfoCb //
		};

/////////////////////////////////////////////////

void rcpWaitPeriod(uint32_t period) {
	uint32_t waittime = period;
	uint32_t start = xTaskGetTickCount();
	while (rpcWaitMqClientMsg(waittime) == 0) {
		uint32_t passed_time = start - xTaskGetTickCount();
		waittime -= passed_time;
		start = xTaskGetTickCount();
	}
}

// init coordinator
// taken from https://sunmaysky.blogspot.com/2017/02/use-ztool-z-stack-30-znp-to-set-up.html
int znp_init_coordinator(uint8_t enable_commissioning) {
	OsalNvWriteFormat_t req;
	setChannelFormat_t chn;
	startCommissioningFormat_t strt;
	ResetReqFormat_t rst;

	/*rcpWaitPeriod(1000);
	 log_print("----------------------\r\n");

	 // Clear complete memory of CC2530
	 req.Id = 0x0003;
	 req.Offset = 0x00;
	 req.Len = 0x01;
	 req.Value[0] = 0x03;
	 sysOsalNvWrite(&req);*/

	rcpWaitPeriod(1000);
	log_print("----------------------\r\n");

	// hard reset
	rst.Type = 0x00;
	sysResetReq(&rst);

	rcpWaitPeriod(4000);
	log_print("----------------------\r\n");

	// Write ZCD_NV_LOGICAL_TYPE to 0 which means coordinator
	req.Id = 0x0087;
	req.Offset = 0x00;
	req.Len = 0x01;
	req.Value[0] = 0x00;
	sysOsalNvWrite(&req);

	rcpWaitPeriod(1000);
	log_print("----------------------\r\n");

	// set primary channel to 13
	chn.primaryChannel = 1;
	chn.channel = CFG_CHANNEL_0x00002000;
	appCfgSetChannel(&chn);

	rcpWaitPeriod(1000);
	log_print("----------------------\r\n");

	// disable secondary channel
	chn.primaryChannel = 0;
	chn.channel = CFG_CHANNEL_NONE;
	appCfgSetChannel(&chn);

	rcpWaitPeriod(1000);
	log_print("----------------------\r\n");

	if (enable_commissioning) {
		// start commissioning using network formation
		strt.commissioningMode = CFG_COMM_MODE_NWK_FORMATION;
		appCfgStartCommissioning(&strt);

		rcpWaitPeriod(10000);

		log_print("----------------------\r\n");

		// get device info
		utilGetDeviceInfo();

		rcpWaitPeriod(1000);
		log_print("----------------------\r\n");

		// Write ZCD_NV_LOGICAL_TYPE to 0 which means coordinator
		req.Id = 0x008F;
		req.Offset = 0x00;
		req.Len = 0x01;
		req.Value[0] = 0x01;
		sysOsalNvWrite(&req);

		rcpWaitPeriod(1000);
		log_print("----------------------\r\n");
	}
	else {
		// get device info
		utilGetDeviceInfo();

		rcpWaitPeriod(1000);
		log_print("----------------------\r\n");
	}

	// start commissioning using network steering
	strt.commissioningMode = CFG_COMM_MODE_NWK_STEERING;
	appCfgStartCommissioning(&strt);

	//
	return 0;
}

/*
 read from:
 https://github.com/zigpy/zigpy-cc/blob/e75d1fccfcd225abf6ac93a1e215ea16b2018897/zigpy_cc/zigbee/start_znp.py#L150

 Endpoints = [
 Endpoint(endpoint=1, appprofid=0x0104),
 Endpoint(endpoint=2, appprofid=0x0101),
 Endpoint(endpoint=3, appprofid=0x0106),
 Endpoint(endpoint=4, appprofid=0x0107),
 Endpoint(endpoint=5, appprofid=0x0108),
 Endpoint(endpoint=6, appprofid=0x0109),
 Endpoint(endpoint=8, appprofid=0x0104),
 Endpoint(
 endpoint=11,
 appprofid=0x0104,
 appdeviceid=0x0400,
 appnumoutclusters=2,
 appoutclusterlist=[IasZone.cluster_id, IasWd.cluster_id],
 ),
 # TERNCY: https://github.com/Koenkk/zigbee-herdsman/issues/82
 Endpoint(endpoint=0x6E, appprofid=0x0104),
 Endpoint(endpoint=12, appprofid=0xC05E),
 Endpoint(
 endpoint=13,
 appprofid=0x0104,
 appnuminclusters=1,
 appinclusterlist=[Ota.cluster_id],
 ),
 # Insta/Jung/Gira: OTA fallback EP (since it's buggy in firmware 10023202
 # when it tries to find a matching EP for OTA - it queries for ZLL profile,
 # but then contacts with HA profile)
 Endpoint(endpoint=47, appprofid=0x0104),
 Endpoint(endpoint=242, appprofid=0xA1E0),
 ]
 */

int znp_register_coordinator(void) {
	RegisterFormat_t reg_req = { 0 };

	// register endpoint 1
	reg_req.EndPoint = 0x01;
	reg_req.AppProfId = 0x104;
	reg_req.AppDeviceId = 5;
	reg_req.AppDevVer = 0x00;
	reg_req.LatencyReq = 0;
	reg_req.AppNumInClusters = 0;
	reg_req.AppNumOutClusters = 0;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// register endpoint 2
	reg_req.EndPoint = 0x02;
	reg_req.AppProfId = 0x101;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// register endpoint 3
	reg_req.EndPoint = 0x03;
	reg_req.AppProfId = 0x106;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// register endpoint 4
	reg_req.EndPoint = 0x04;
	reg_req.AppProfId = 0x107;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// register endpoint 5
	reg_req.EndPoint = 0x05;
	reg_req.AppProfId = 0x108;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// register endpoint 6
	reg_req.EndPoint = 0x06;
	reg_req.AppProfId = 0x109;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// register endpoint 8
	reg_req.EndPoint = 0x08;
	reg_req.AppProfId = 0x104;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// register endpoint 12
	reg_req.EndPoint = 0x0C;
	reg_req.AppProfId = 0xC05E;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// register endpoint 47
	reg_req.EndPoint = 0x2F;
	reg_req.AppProfId = 0x104;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// register endpoint 110
	reg_req.EndPoint = 0x6e;
	reg_req.AppProfId = 0x104;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// register endpoint 242
	reg_req.EndPoint = 0xf2;
	reg_req.AppProfId = 0xA1E0;
	afRegister(&reg_req);
	rcpWaitPeriod(1000);

	// all good
	return 0;
}

#define ZCL_READ_ATTR			0x00
#define ZCL_READ_ATTR_RSP		0x01
#define ZCL_WRITE_ATTR			0x02
#define ZCL_WRITE_ATTR_RSP		0x04

// Zigbee cluster library specification
// https://zigbeealliance.org/wp-content/uploads/2019/12/07-5123-06-zigbee-cluster-library-specification.pdf

void do_transaction(uint16_t addr) {
	IeeeAddrReqFormat_t req;
	ActiveEpReqFormat_t act_req;
	DataRequestFormat_t data_req;

	rcpWaitPeriod(10000);
	log_print("1 ----------------------\r\n");

	// check if the endpoint is active
	act_req.DstAddr = addr;
	act_req.NwkAddrOfInterest = addr;
	zdoActiveEpReq(&act_req);

	rcpWaitPeriod(10000);
	log_print("2 ----------------------\r\n");

	// request ieee address
	req.ShortAddr = addr;
	req.ReqType = 0x00;
	req.StartIndex = 0x00;
	zdoIeeeAddrReq(&req);

	rcpWaitPeriod(10000);
	log_print("3 ----------------------\r\n");

	// read a cluster (0x0000, 0x0004, aka manufacturer name)
	data_req.DstAddr = addr;
	data_req.DstEndpoint = 0x01;
	data_req.SrcEndpoint = 0x01;
	data_req.ClusterID = 0x0000;
	data_req.TransID = 0x05;
	data_req.Options = 0x00;
	data_req.Radius = 0x07;
	data_req.Len = 5;
	data_req.Data[0] = 0x00; // frame control
	data_req.Data[1] = 0x02; // transaction sequence num
	data_req.Data[2] = ZCL_READ_ATTR; // Command ID
	data_req.Data[3] = 0x00; // Cluster 16bit low
	data_req.Data[4] = 0x00; // Cluster 16bit high
	afDataRequest(&data_req);

	rcpWaitPeriod(10000);
	log_print("4 ----------------------\r\n");

	// move to hue command (ZCL specification page 149)
	data_req.DstAddr = addr;
	data_req.DstEndpoint = 0x01;
	data_req.SrcEndpoint = 0x01;
	data_req.ClusterID = 0x0006;
	data_req.TransID = 0x05;
	data_req.Options = 0x00;
	data_req.Radius = 0x07;
	data_req.Len = 3;
	data_req.Data[0] = 0x01; // frame control
	data_req.Data[1] = 0x03; // transaction sequence num
	data_req.Data[2] = 0x01; // on command

	afDataRequest(&data_req);

	rcpWaitPeriod(10000);
	log_print("5 ----------------------\r\n");

	// move to hue command (ZCL specification page 334)
	data_req.DstAddr = addr;
	data_req.DstEndpoint = 0x01;
	data_req.SrcEndpoint = 0x01;
	data_req.ClusterID = 0x0300;
	data_req.TransID = 0x05;
	data_req.Options = 0x00;
	data_req.Radius = 0x07;
	data_req.Len = 7;
	data_req.Data[0] = 0x01; // frame control
	data_req.Data[1] = 0x03; // transaction sequence num
	data_req.Data[2] = 0x00; // move to hue command
	data_req.Data[3] = 0x50; // HUE
	data_req.Data[4] = 0x00; // direction, shortest distance
	data_req.Data[5] = 0x00; // transition time low
	data_req.Data[6] = 0x00; // transition time high

	afDataRequest(&data_req);

	rcpWaitPeriod(10000);
	log_print("6 ----------------------\r\n");
}

/////////////////////////////////////////////////
void vAppTask(void *pvParameters) {
	log_print("System started\r\n");

	//Register callbacks
	sysRegisterCallbacks(mtSysCb);
	zdoRegisterCallbacks(mtZdoCb);
	afRegisterCallbacks(mtAfCb);
	appCfgRegisterCallbacks(mtAppCfgCb);
	utilRegisterCallbacks(mtUtilCb);

	// startup delay
	vTaskDelay(1000);

	// ping ok?
	if (sysVersion() == 0) {
		// initialize coordinator
		znp_init_coordinator(1);

		// give some time to breathe
		rcpWaitPeriod(10000);

		// register clusters in coordinator
		znp_register_coordinator();

		// give some time to breathe
		rcpWaitPeriod(10000);

		// register cluster
		do_transaction(0x09d1);
		do_transaction(0x8203);
	}

	// endless loop, handle CC2530 packets
	while (1) {
		rpcWaitMqClientMsg(portMAX_DELAY);
	}
}

void vComTask(void *pvParameters) {
	// init queues
	rpcInitMq();

	// initialize serial port
	rpcOpen();

	// loop
	while (1) {
		// keep procesing packets
		rpcProcess();

		// always a little loop delay
		vTaskDelay(1);
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_LPUART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	dbg_sem = xSemaphoreCreateBinary();
	xSemaphoreGive(dbg_sem);

	xTaskCreate(vAppTask, "APP", 512, NULL, 6, NULL);
	xTaskCreate(vComTask, "COM", 256, NULL, 5, NULL);

	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 20;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_LPUART1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
