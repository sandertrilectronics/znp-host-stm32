#include "znp_cb.h"
#include "dbgPrint.h"
#include "mtAf.h"
#include "mtAppCfg.h"
#include "mtParser.h"
#include "mtSys.h"
#include "mtUtil.h"
#include "mtZdo.h"
#include "rpc.h"
#include "znp_if.h"
#include <stddef.h>

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

static mtSysCb_t mtSysCb = {NULL, NULL, NULL, mtSysResetIndCb, mtVersionIndCb, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
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

    // devState = (devStates_t) newDevState;

    return SUCCESS;
}

static uint8_t mtZdoSimpleDescRspCb(SimpleDescRspFormat_t *msg) {
    // send event
    event_result_t res;
    res.type = EVT_RSP_SIMPLE_DESC;
    res.result = msg->Status;
    znp_if_evt_send(&res);

    // print info
    if (msg->Status == MT_RPC_SUCCESS) {
        log_print("SrcAddr: %04x\n", msg->SrcAddr);
        log_print("NwkAddr: %04x\n", msg->NwkAddr);
        log_print("\tEndpoint: 0x%02X\n", msg->Endpoint);
        log_print("\tProfileID: 0x%04X\n", msg->ProfileID);
        log_print("\tDeviceID: 0x%04X\n", msg->DeviceID);
        log_print("\tDeviceVersion: 0x%02X\n", msg->DeviceVersion);
        log_print("\tNumInClusters: %d\n", msg->NumInClusters);
        for (uint32_t i = 0; i < msg->NumInClusters; i++) {
            log_print("\t\tInClusterList[%d]: 0x%04X\n", i, msg->InClusterList[i]);
        }
        log_print("\tNumOutClusters: %d\n", msg->NumOutClusters);
        for (uint32_t i = 0; i < msg->NumOutClusters; i++) {
            log_print("\t\tOutClusterList[%d]: 0x%04X\n", i, msg->OutClusterList[i]);
        }

        // copy all data to device to register it
        znp_device_t *dev = znp_if_dev_get(msg->SrcAddr);
        dev->profile_id = msg->ProfileID;
        dev->device_id = msg->DeviceID;
        dev->clstr_in_cnt = MIN(msg->NumInClusters, CLSTR_LIST_MAX);
        for (uint32_t i = 0; i < msg->NumInClusters; i++) {
            dev->clstr_in_list[i] = msg->InClusterList[i];
        }
        dev->clstr_out_cnt = MIN(msg->NumOutClusters, CLSTR_LIST_MAX);
        for (uint32_t i = 0; i < msg->NumOutClusters; i++) {
            dev->clstr_out_list[i] = msg->OutClusterList[i];
        }
    } else {
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
    // send event
    event_result_t res;
    res.type = EVT_RSP_IS_ACTIVE;
    res.result = msg->Status;
    znp_if_evt_send(&res);

    // SimpleDescReqFormat_t simReq;
    log_print("NwkAddr: 0x%04X\n", msg->NwkAddr);
    if (msg->Status == MT_RPC_SUCCESS) {
        log_print("Number of Endpoints: %d\nActive Endpoints: ", msg->ActiveEPCount);
        for (uint32_t i = 0; i < msg->ActiveEPCount; i++) {
            log_print("0x%02X\t", msg->ActiveEPList[i]);
        }
        log_print("\n");
    } else {
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

    // register device
    znp_if_dev_add(msg->SrcAddr);
    znp_if_dev_set_ieee(msg->SrcAddr, msg->IEEEAddr);

    // check if the endpoint is active
    ActiveEpReqFormat_t actReq;
    actReq.DstAddr = msg->NwkAddr;
    actReq.NwkAddrOfInterest = msg->NwkAddr;
    zdoActiveEpReq(&actReq);

    //
    return 0;
}

static mtZdoCb_t mtZdoCb = {
    //
    NULL,                      // MT_ZDO_NWK_ADDR_RSP
    NULL,                      // MT_ZDO_IEEE_ADDR_RSP
    NULL,                      // MT_ZDO_NODE_DESC_RSP
    NULL,                      // MT_ZDO_POWER_DESC_RSP
    mtZdoSimpleDescRspCb,      // MT_ZDO_SIMPLE_DESC_RSP
    mtZdoActiveEpRspCb,        // MT_ZDO_ACTIVE_EP_RSP
    NULL,                      // MT_ZDO_MATCH_DESC_RSP
    NULL,                      // MT_ZDO_COMPLEX_DESC_RSP
    NULL,                      // MT_ZDO_USER_DESC_RSP
    NULL,                      // MT_ZDO_USER_DESC_CONF
    NULL,                      // MT_ZDO_SERVER_DISC_RSP
    NULL,                      // MT_ZDO_END_DEVICE_BIND_RSP
    NULL,                      // MT_ZDO_BIND_RSP
    NULL,                      // MT_ZDO_UNBIND_RSP
    NULL,                      // MT_ZDO_MGMT_NWK_DISC_RSP
    mtZdoMgmtLqiRspCb,         // MT_ZDO_MGMT_LQI_RSP
    NULL,                      // MT_ZDO_MGMT_RTG_RSP
    NULL,                      // MT_ZDO_MGMT_BIND_RSP
    NULL,                      // MT_ZDO_MGMT_LEAVE_RSP
    NULL,                      // MT_ZDO_MGMT_DIRECT_JOIN_RSP
    NULL,                      // MT_ZDO_MGMT_PERMIT_JOIN_RSP
    mtZdoStateChangeIndCb,     // MT_ZDO_STATE_CHANGE_IND
    mtZdoEndDeviceAnnceIndCb,  // MT_ZDO_END_DEVICE_ANNCE_IND
    NULL,                      // MT_ZDO_SRC_RTG_IND
    NULL,                      // MT_ZDO_BEACON_NOTIFY_IND
    NULL,                      // MT_ZDO_JOIN_CNF
    NULL,                      // MT_ZDO_NWK_DISCOVERY_CNF
    NULL,                      // MT_ZDO_CONCENTRATOR_IND_CB
    NULL,                      // MT_ZDO_LEAVE_IND
    NULL,                      // MT_ZDO_STATUS_ERROR_RSP
    NULL,                      // MT_ZDO_MATCH_DESC_RSP_SENT
    NULL,                      //
    NULL                       //
};
/********************************************************************
 * AF CALL BACK FUNCTIONS
 */

static uint8_t mtAfDataConfirmCb(DataConfirmFormat_t *msg) {
    if (msg->Status == MT_RPC_SUCCESS) {
        log_print("Message transmited Succesfully!\n");
    } else {
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

static uint8_t mtAfRegisterCb(RegisterSrspFormat_t *msg) {
    // send event
    event_result_t res;
    res.type = EVT_RSP_REGISTER;
    res.result = msg->success;
    znp_if_evt_send(&res);

    if (msg->success == 0) {
        log_print("Register OK\n");
    } else {
        log_print("Register Error\n");
    }
}

static uint8_t mtAfDataRequestCb(DataRequestSrspFormat_t *msg) {
    // send event
    event_result_t res;
    res.type = EVT_RSP_DATA_REQUEST;
    res.result = msg->success;
    znp_if_evt_send(&res);

    if (msg->success == 0) {
        log_print("Data request OK\n");
    } else {
        log_print("Data request Error\n");
    }
}

static mtAfCb_t mtAfCb = {
    //
    mtAfDataConfirmCb,  // MT_AF_DATA_CONFIRM
    mtAfIncomingMsgCb,  // MT_AF_INCOMING_MSG
    NULL,               // MT_AF_INCOMING_MSG_EXT
    mtAfRegisterCb,     // MT_AF_REGISTER
    mtAfDataRequestCb,
    NULL,  // MT_AF_DATA_RETRIEVE
    NULL,  // MT_AF_REFLECT_ERROR
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

static mtAppCfgCb_t mtAppCfgCb = {
    //
    mtAppCfgCommissioningNotifyCb,  //
    mtAppCfgSetChannelCb,           //
    mtAppCfgCommissioningStartCb    //
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
    for (uint8_t i = 0; i < msg->ass_device_cnt; i++) {
        // print info
        log_print("Ass Dev %d: %04x\r\n", i, msg->ass_device_list[i]);

        // register device
        znp_if_dev_add(msg->ass_device_list[i]);
    }
    return 0;
}

static mtUtilCb_t mtUtilCb = {
    //
    mtUtilGetDeviceInfoCb  //
};

void znp_cb_register(void) {
    // Register callbacks
    sysRegisterCallbacks(mtSysCb);
    zdoRegisterCallbacks(mtZdoCb);
    afRegisterCallbacks(mtAfCb);
    appCfgRegisterCallbacks(mtAppCfgCb);
    utilRegisterCallbacks(mtUtilCb);
}