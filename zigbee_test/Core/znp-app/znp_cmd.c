#include "znp_cmd.h"
#include "znp_if.h"
#include "znp_cb.h"
#include "dbgPrint.h"
#include "mtAf.h"
#include "mtAppCfg.h"
#include "mtParser.h"
#include "mtSys.h"
#include "mtUtil.h"
#include "mtZdo.h"
#include  <stddef.h>

int znp_cmd_dev_is_active(uint16_t address) {
    // get device handle
    znp_device_t* dev = znp_if_dev_get(address);

    // invalid handle?
    if (dev == NULL)
        return -1;

    // check if the endpoint is active
    ActiveEpReqFormat_t act_req;
    act_req.DstAddr = dev->adr_short;
    act_req.NwkAddrOfInterest = dev->adr_short;
    if (zdoActiveEpReq(&act_req) != 0)
        return -1;

    // wait for response
    if (znp_if_wait_for_event(EVT_RSP_IS_ACTIVE, 10000))
        return 0;
    else
        return -1;
}

int znp_cmd_dev_refresh_info(uint16_t address) {
    // get device handle
    znp_device_t* dev = znp_if_dev_get(address);

    // invalid handle?
    if (dev == NULL)
        return -1;

    // request descriptors
    SimpleDescReqFormat_t desc_req;
    desc_req.DstAddr = dev->adr_short;
    desc_req.NwkAddrOfInterest = dev->adr_short;
    desc_req.Endpoint = 1;
    if (zdoSimpleDescReq(&desc_req) != 0)
        return -1;

    // wait for response
    if (znp_if_wait_for_event(EVT_RSP_SIMPLE_DESC, 10000))
        return 0;
    else
        return -1;
}

int znp_cmd_dev_register(uint16_t address) {
    // get device handle
    znp_device_t* dev = znp_if_dev_get(address);

    // invalid handle?
    if (dev == NULL)
        return -1;

    // sanity check if data is ok
    if (dev->clstr_in_cnt == 0 || dev->clstr_out_cnt == 0 || dev->device_id == 0 || dev->profile_id == 0)
        return -1;

    // register device
    RegisterFormat_t reg_req;
    reg_req.EndPoint = 0x01;
    reg_req.AppProfId = dev->profile_id;
    reg_req.AppDeviceId = dev->device_id;
    reg_req.AppDevVer = 0x01;
    reg_req.LatencyReq = 0;
    reg_req.AppNumInClusters = dev->clstr_in_cnt;
    for (uint8_t i = 0; i < dev->clstr_in_cnt; i++)
        reg_req.AppInClusterList[i] = dev->clstr_in_list[i];
    for (uint8_t i = 0; i < dev->clstr_out_cnt; i++)
        reg_req.AppOutClusterList[i] = dev->clstr_out_list[i];
    if (afRegister(&reg_req) != 0)
        return -1;

    // wait for response
    if (znp_if_wait_for_event(EVT_RSP_REGISTER, 10000))
        return 0;
    else
        return -1;
}