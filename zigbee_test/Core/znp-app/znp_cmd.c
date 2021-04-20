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

static uint8_t _znp_cmd_sequence_num = 0;

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

int znp_cmd_cluster_in_read(uint16_t address, uint16_t cluster, uint16_t attribute, zcl_cluster_record_t* record) {
    // get device handle
    znp_device_t* dev = znp_if_dev_get(address);

    // invalid handle?
    if (dev == NULL)
        return -1;

    // sanity check if data is ok
    if (!znp_dev_has_in_cluster(dev, cluster))
        return -1;

    // increase number
    _znp_cmd_sequence_num++;

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
    data_req.Data[0] = 0x00;                   // ZCL Header: frame control
    data_req.Data[1] = _znp_cmd_sequence_num;  // ZCL Header: transaction sequence num
    data_req.Data[2] = ZCL_CMD_READ_ATTR;      // ZCL Header: Command ID
    data_req.Data[3] = attribute & 0xFF;       // Cluster 16bit low
    data_req.Data[4] = attribute >> 8;         // Cluster 16bit high
    afDataRequest(&data_req);

    // wait for response
    event_result_t* event = znp_if_wait_for_event(EVT_RSP_DATA_REQUEST, dev->adr_short, 10000);
    if (event == NULL)
        return -1;

    // check length
    if (event->data_len < 6)
        return -1;

    // ZCL Header: check transaction sequence number
    if (event->data[1] != _znp_cmd_sequence_num)
        return -1;

    // ZCL Header: check command ID
    if (event->data[2] != ZCL_CMD_READ_ATTR_RSP)
        return -1;

    // Data: which attribute is in this response?
    uint16_t read_attribute = (event->data[3] | (event->data[4] << 8));
    if (read_attribute != attribute)
        return -1;

    // Data: status not ok?
    if (event->data[5] != 0)
        return -1;

    // save type
    record->type = event->data[6];

    uint8_t parse_valid = 0;

    // Data: parse type and data
    switch (record->type) {
        case ZCL_NO_DATA_TYPE:
        case ZCL_DATA_8BITS:
        case ZCL_BITMAP_8BITS:
        case ZCL_UNSIGNED_8BITS:
        case ZCL_ENUMERATION_8BITS:
        case ZCL_BOOLEAN_8BITS:
        case ZCL_SIGNED_8BITS:
            record->data_u8 = event->data[7];
            parse_valid = 1;
            break;

        case ZCL_DATA_16BITS:
        case ZCL_BITMAP_16BITS:
        case ZCL_UNSIGNED_16BITS:
        case ZCL_ENUMERATION_16BITS:
        case ZCL_SIGNED_16BITS:
            record->data_u16 = (event->data[7] | (event->data[8] << 8));
            parse_valid = 1;
            break;

        case ZCL_DATA_24BITS:
        case ZCL_BITMAP_24BITS:
        case ZCL_UNSIGNED_24BITS:
            record->data_u32 = (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16));
            parse_valid = 1;
            break;

        case ZCL_SIGNED_24BITS:
            break;

        case ZCL_DATA_32BITS:
        case ZCL_BITMAP_32BITS:
        case ZCL_UNSIGNED_32BITS:
        case ZCL_SIGNED_32BITS:
            record->data_u32 = (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16) | (event->data[10] << 24));
            parse_valid = 1;
            break;

        case ZCL_DATA_40BITS:
        case ZCL_BITMAP_40BITS:
        case ZCL_UNSIGNED_40BITS:
            record->data_u64 = (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16) | (event->data[10] << 24) |
                            (event->data[11] << 32));
            parse_valid = 1;
            break;

        case ZCL_SIGNED_40BITS:
            break;

        case ZCL_DATA_48BITS:
        case ZCL_BITMAP_48BITS:
        case ZCL_UNSIGNED_48BITS:
            record->data_u64 = (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16) | (event->data[10] << 24) |
                            (event->data[11] << 32) | (event->data[12] << 40));
            parse_valid = 1;
            break;

        case ZCL_SIGNED_48BITS:
            break;

        case ZCL_DATA_56BITS:
        case ZCL_BITMAP_56BITS:
        case ZCL_UNSIGNED_56BITS:
            record->data_u64 = (event->data[7] | (event->data[8] << 8) | (event->data[9] << 16) | (event->data[10] << 24) |
                            (event->data[11] << 32) | (event->data[12] << 40) | (event->data[13] << 48));
            parse_valid = 1;
            break;

        case ZCL_SIGNED_56BITS:
            break;

        case ZCL_DATA_64BITS:
        case ZCL_BITMAP_64BITS:
        case ZCL_UNSIGNED_64BITS:
        case ZCL_SIGNED_64BITS:
            record->data_u64 =
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
            record->data_arr_len = MIN(ZNP_DATA_LEN_MAX, event->data[7]);
            memcpy(record->data_arr, &event->data[8], record->data_arr_len);
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
        return 0;
    else
        return -1;
}

int znp_cmd_cluster_in_write(uint16_t address, uint16_t cluster, uint16_t attribute, zcl_cluster_record_t* record) {
    // get device handle
    znp_device_t* dev = znp_if_dev_get(address);

    // invalid handle?
    if (dev == NULL)
        return -1;

    // sanity check if data is ok
    if (!znp_dev_has_in_cluster(dev, cluster))
        return -1;

    // increase number
    _znp_cmd_sequence_num++;

    // read a cluster
    DataRequestFormat_t data_req;
    data_req.DstAddr = dev->adr_short;
    data_req.DstEndpoint = 0x01;
    data_req.SrcEndpoint = 0x01;
    data_req.ClusterID = cluster;
    data_req.TransID = 0x05;
    data_req.Options = 0x00;
    data_req.Radius = 0x07;
    data_req.Len = 0;
    data_req.Data[data_req.Len++] = 0x00;                   // ZCL Header: frame control
    data_req.Data[data_req.Len++] = _znp_cmd_sequence_num;  // ZCL Header: transaction sequence num
    data_req.Data[data_req.Len++] = ZCL_CMD_WRITE_ATTR;     // ZCL Header: Command ID
    data_req.Data[data_req.Len++] = attribute & 0xFF;       // Data: Cluster 16bit low
    data_req.Data[data_req.Len++] = attribute >> 8;         // Data: Cluster 16bit high

    uint8_t parse_valid = 0;

    // Data: parse type and data
    switch (record->type) {
        case ZCL_NO_DATA_TYPE:
        case ZCL_DATA_8BITS:
        case ZCL_BITMAP_8BITS:
        case ZCL_UNSIGNED_8BITS:
        case ZCL_ENUMERATION_8BITS:
        case ZCL_BOOLEAN_8BITS:
        case ZCL_SIGNED_8BITS:
            data_req.Data[data_req.Len++] = record->type;
            data_req.Data[data_req.Len++] = record->data_u8;
            parse_valid = 1;
            break;

        case ZCL_DATA_16BITS:
        case ZCL_BITMAP_16BITS:
        case ZCL_UNSIGNED_16BITS:
        case ZCL_ENUMERATION_16BITS:
        case ZCL_SIGNED_16BITS:
            data_req.Data[data_req.Len++] = record->type;
            data_req.Data[data_req.Len++] = record->data_u16 & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u16 >> 8) & 0xFF;
            parse_valid = 1;
            break;

        case ZCL_DATA_24BITS:
        case ZCL_BITMAP_24BITS:
        case ZCL_UNSIGNED_24BITS:
            data_req.Data[data_req.Len++] = record->type;
            data_req.Data[data_req.Len++] = record->data_u32 & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u32 >> 8) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u32 >> 16) & 0xFF;
            parse_valid = 1;
            break;

        case ZCL_SIGNED_24BITS:
            break;

        case ZCL_DATA_32BITS:
        case ZCL_BITMAP_32BITS:
        case ZCL_UNSIGNED_32BITS:
        case ZCL_SIGNED_32BITS:
            data_req.Data[data_req.Len++] = record->type;
            data_req.Data[data_req.Len++] = record->data_u32 & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u32 >> 8) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u32 >> 16) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u32 >> 24) & 0xFF;
            parse_valid = 1;
            break;

        case ZCL_DATA_40BITS:
        case ZCL_BITMAP_40BITS:
        case ZCL_UNSIGNED_40BITS:
            data_req.Data[data_req.Len++] = record->type;
            data_req.Data[data_req.Len++] = record->data_u64 & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 8) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 16) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 24) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 32) & 0xFF;
            parse_valid = 1;
            break;

        case ZCL_SIGNED_40BITS:
            break;

        case ZCL_DATA_48BITS:
        case ZCL_BITMAP_48BITS:
        case ZCL_UNSIGNED_48BITS:
            data_req.Data[data_req.Len++] = record->type;
            data_req.Data[data_req.Len++] = record->data_u64 & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 8) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 16) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 24) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 32) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 40) & 0xFF;
            parse_valid = 1;
            break;

        case ZCL_SIGNED_48BITS:
            break;

        case ZCL_DATA_56BITS:
        case ZCL_BITMAP_56BITS:
        case ZCL_UNSIGNED_56BITS:
            data_req.Data[data_req.Len++] = record->type;
            data_req.Data[data_req.Len++] = record->data_u64 & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 8) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 16) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 24) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 32) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 40) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 48) & 0xFF;
            parse_valid = 1;
            break;

        case ZCL_SIGNED_56BITS:
            break;

        case ZCL_DATA_64BITS:
        case ZCL_BITMAP_64BITS:
        case ZCL_UNSIGNED_64BITS:
        case ZCL_SIGNED_64BITS:
            data_req.Data[data_req.Len++] = record->type;
            data_req.Data[data_req.Len++] = record->data_u64 & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 8) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 16) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 24) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 32) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 40) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 48) & 0xFF;
            data_req.Data[data_req.Len++] = (record->data_u64 >> 56) & 0xFF;
            parse_valid = 1;
            break;

        case ZCL_SEMI_PRECISION:
        case ZCL_SINGLE_PRECISION:
        case ZCL_DOUBLE_PRECISION:
            break;

        case ZCL_OCTET_STRING:
        case ZCL_CHARACTER_STRING:
            data_req.Data[data_req.Len++] = record->type;
            uint8_t len = MIN(record->data_arr_len, sizeof(data_req.Data) - 6);
            memcpy(&data_req.Data[data_req.Len], record->data_arr, len);
            data_req.Len += len;
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

    // unsupported data given?
    if (!parse_valid)
        return -1;

    // do the request
    afDataRequest(&data_req);

    // wait for response
    event_result_t* event = znp_if_wait_for_event(EVT_RSP_DATA_REQUEST, dev->adr_short, 10000);
    if (event == NULL)
        return -1;

    // check response length
    if (event->data_len < 4)
        return -1;

    // ZCL Header: parse
    if (event->data[1] != _znp_cmd_sequence_num)
        return -1;

    // ZCL Header: parse
    if (event->data[2] != ZCL_CMD_WRITE_ATTR_RSP)
        return -1;

    // result was ok?
    if (event->data[3] == 0x00)
        return 0;
    else
        return -1;
}