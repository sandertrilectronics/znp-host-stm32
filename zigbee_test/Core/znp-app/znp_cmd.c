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
#include <stddef.h>

static uint8_t znp_dev_has_in_cluster(znp_device_t* dev, uint16_t cluster) {
    for (uint8_t i = 0; i < dev->clstr_in_cnt; i++) {
        if (dev->clstr_in_list[i] == cluster)
            return 1;
    }
    return 0;
}

static uint8_t znp_dev_has_out_cluster(znp_device_t* dev, uint16_t cluster) {
    for (uint8_t i = 0; i < dev->clstr_out_cnt; i++) {
        if (dev->clstr_out_list[i] == cluster)
            return 1;
    }
    return 0;
}

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
    zdoActiveEpReq(&act_req);

    // wait for response
    if (znp_if_wait_for_event(EVT_RSP_IS_ACTIVE, dev->adr_short, 10000) != NULL)
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
    zdoSimpleDescReq(&desc_req);

    // wait for response
    if (znp_if_wait_for_event(EVT_RSP_SIMPLE_DESC, dev->adr_short, 10000) != NULL)
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
    reg_req.AppNumOutClusters = dev->clstr_out_cnt;
    for (uint8_t i = 0; i < dev->clstr_out_cnt; i++)
        reg_req.AppOutClusterList[i] = dev->clstr_out_list[i];
    afRegister(&reg_req);

    // wait for response
    if (znp_if_wait_for_event(EVT_RSP_REGISTER, dev->adr_short, 10000) != NULL)
        return 0;
    else
        return -1;
}

znp_cluster_read_rsp_t* znp_cmd_cluster_in_read(uint16_t address, uint16_t cluster, uint16_t attribute) {
    static znp_cluster_read_rsp_t ret;
    static uint8_t sequence_num = 0;

    // get device handle
    znp_device_t* dev = znp_if_dev_get(address);

    // invalid handle?
    if (dev == NULL)
        return NULL;

    // sanity check if data is ok
    if (!znp_dev_has_in_cluster(dev, cluster))
        return NULL;

    // increase number
    sequence_num++;

    // read a cluster
    DataRequestFormat_t data_req;
    data_req.DstAddr = dev->adr_short;
    data_req.DstEndpoint = 0x01;
    data_req.SrcEndpoint = 0x01;
    data_req.ClusterID = cluster;
    data_req.TransID = 0x05;
    data_req.Options = 0x00;
    data_req.Radius = 0x07;
    data_req.Len = 5;
    data_req.Data[0] = 0x00;               // ZCL Header: frame control
    data_req.Data[1] = sequence_num;       // ZCL Header: transaction sequence num
    data_req.Data[2] = ZCL_CMD_READ_ATTR;  // ZCL Header: Command ID
    data_req.Data[3] = attribute & 0xFF;   // Cluster 16bit low
    data_req.Data[4] = attribute >> 8;     // Cluster 16bit high
    afDataRequest(&data_req);

    // wait for response
    event_result_t* event = znp_if_wait_for_event(EVT_RSP_DATA_REQUEST, dev->adr_short, 10000);
    if (event == NULL)
        return NULL;

    // ZCL Header: check transaction sequence number
    if (event->data[1] != sequence_num)
        return NULL;

    // ZCL Header: check command ID
    if (event->data[2] != ZCL_CMD_READ_ATTR_RSP)
        return NULL;

    // Data: which attribute is in this response?
    uint16_t read_attribute = (event->data[3] | (event->data[4] << 8));
    if (read_attribute != attribute)
        return NULL;

    // Data: status not ok?
    if (event->data[5] != 0)
        return NULL;

    // save type
    ret.type = event->data[6];

    uint8_t parse_valid = 0;

    // Data: parse type and data
    switch (ret.type) {
        case ZCL_NO_DATA_TYPE:
        case ZCL_DATA_8BITS:
        case ZCL_BITMAP_8BITS:
        case ZCL_UNSIGNED_8BITS:
        case ZCL_ENUMERATION_8BITS:
        case ZCL_BOOLEAN_8BITS:
        case ZCL_SIGNED_8BITS:
            ret.data_u8 = event->data[7];
            parse_valid = 1;
            break;

        case ZCL_DATA_16BITS:
        case ZCL_BITMAP_16BITS:
        case ZCL_UNSIGNED_16BITS:
        case ZCL_ENUMERATION_16BITS:
        case ZCL_SIGNED_16BITS:
            ret.data_u16 = (event->data[7] | (event->data[8] << 8));
            parse_valid = 1;
            break;

        case ZCL_DATA_24BITS:
        case ZCL_BITMAP_24BITS:
        case ZCL_UNSIGNED_24BITS:
            ret.data_u32 = (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16));
            parse_valid = 1;
            break;

        case ZCL_SIGNED_24BITS:
            break;

        case ZCL_DATA_32BITS:
        case ZCL_BITMAP_32BITS:
        case ZCL_UNSIGNED_32BITS:
        case ZCL_SIGNED_32BITS:
            ret.data_u32 = (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16) | (event->data[10] << 24));
            parse_valid = 1;
            break;

        case ZCL_DATA_40BITS:
        case ZCL_BITMAP_40BITS:
        case ZCL_UNSIGNED_40BITS:
            ret.data_u64 = (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16) | (event->data[10] << 24) |
                            (event->data[11] << 32));
            parse_valid = 1;
            break;

        case ZCL_SIGNED_40BITS:
            break;

        case ZCL_DATA_48BITS:
        case ZCL_BITMAP_48BITS:
        case ZCL_UNSIGNED_48BITS:
            ret.data_u64 = (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16) | (event->data[10] << 24) |
                            (event->data[11] << 32) | (event->data[12] << 40));
            parse_valid = 1;
            break;

        case ZCL_SIGNED_48BITS:
            break;

        case ZCL_DATA_56BITS:
        case ZCL_BITMAP_56BITS:
        case ZCL_UNSIGNED_56BITS:

            ret.data_u64 = (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16) | (event->data[10] << 24) |
                            (event->data[11] << 32) | (event->data[12] << 40) | (event->data[13] << 48));
            parse_valid = 1;
            break;

        case ZCL_SIGNED_56BITS:
            break;

        case ZCL_DATA_64BITS:
        case ZCL_BITMAP_64BITS:
        case ZCL_UNSIGNED_64BITS:
        case ZCL_SIGNED_64BITS:
            ret.data_u64 =
                (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16) | (event->data[10] << 24) |
                 (event->data[11] << 32) | (event->data[12] << 40) | (event->data[13] << 48) | (event->data[14] << 56));
            parse_valid = 1;
            break;

        case ZCL_SEMI_PRECISION:
        case ZCL_SINGLE_PRECISION:
        case ZCL_DOUBLE_PRECISION:

            break;

        case ZCL_OCTET_STRING:
        case ZCL_CHARACTER_STRING:
            memcpy(ret.data_arr, &event->data[8], MIN(ZNP_DATA_LEN_MAX, event->data[7]));
            parse_valid = 1;
            break;

        case ZCL_LONG_OCTET_STRING:
        case ZCL_LONG_CHAR_STRING:
            break;

        case ZCL_ARRAY_ORDERED:
        case ZCL_STRUCTURE_ORDERED:
        case ZCL_SET_COLLECTION:
        case ZCL_BAG_COLLECTION:
        case ZCL_TIME_OF_DAY:
        case ZCL_DATE_OF_DAY:
        case ZCL_UTC_TIME:
        case ZCL_CLUSTER_ID:
        case ZCL_ATTRIBUTE_ID:
        case ZCL_BACNET_OID:
        case ZCL_IEEE_ADDRESS:
        case ZCL_SECURITY_KEY_128BITS:
        case ZCL_UNKNOWN:
            break;
    }

    // check
    if (parse_valid)
        return &ret;
    else
        return NULL;
}