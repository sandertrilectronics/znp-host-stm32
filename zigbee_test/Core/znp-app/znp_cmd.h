#ifndef __ZNP_CMD_H__
#define __ZNP_CMD_H__

#include <stdint.h>

#define ZNP_DATA_LEN_MAX            32

typedef enum {
  ZCL_CMD_READ_ATTR = 0x00,
  ZCL_CMD_READ_ATTR_RSP = 0x01,
  ZCL_CMD_WRITE_ATTR = 0x02,
  ZCL_CMD_WRITE_ATTR_RSP = 0x04
} zcl_cmd_type_t;

typedef enum {
  ZCL_NO_DATA_TYPE = 0x00, 
  ZCL_DATA_8BITS = 0x08, 
  ZCL_DATA_16BITS = 0x09, 
  ZCL_DATA_24BITS = 0x0a,
  ZCL_DATA_32BITS = 0x0b, 
  ZCL_DATA_40BITS = 0x0c, 
  ZCL_DATA_48BITS = 0x0d, 
  ZCL_DATA_56BITS = 0x0e,
  ZCL_DATA_64BITS = 0x0f, 
  ZCL_BOOLEAN_8BITS = 0x10, 
  ZCL_BITMAP_8BITS = 0x18, 
  ZCL_BITMAP_16BITS = 0x19,
  ZCL_BITMAP_24BITS = 0x1a, 
  ZCL_BITMAP_32BITS = 0x1b, 
  ZCL_BITMAP_40BITS = 0x1c, 
  ZCL_BITMAP_48BITS = 0x1d,
  ZCL_BITMAP_56BITS = 0x1e, 
  ZCL_BITMAP_64BITS = 0x1f, 
  ZCL_UNSIGNED_8BITS = 0x20, 
  ZCL_UNSIGNED_16BITS = 0x21,
  ZCL_UNSIGNED_24BITS = 0x22, 
  ZCL_UNSIGNED_32BITS = 0x23, 
  ZCL_UNSIGNED_40BITS = 0x24, 
  ZCL_UNSIGNED_48BITS = 0x25,
  ZCL_UNSIGNED_56BITS = 0x26, 
  ZCL_UNSIGNED_64BITS = 0x27, 
  ZCL_SIGNED_8BITS = 0x28, 
  ZCL_SIGNED_16BITS = 0x29,
  ZCL_SIGNED_24BITS = 0x2a, 
  ZCL_SIGNED_32BITS = 0x2b, 
  ZCL_SIGNED_40BITS = 0x2c, 
  ZCL_SIGNED_48BITS = 0x2d,
  ZCL_SIGNED_56BITS = 0x2e, 
  ZCL_SIGNED_64BITS = 0x2f, 
  ZCL_ENUMERATION_8BITS = 0x30, 
  ZCL_ENUMERATION_16BITS = 0x31,
  ZCL_SEMI_PRECISION = 0x38, 
  ZCL_SINGLE_PRECISION = 0x39, 
  ZCL_DOUBLE_PRECISION = 0x3a, 
  ZCL_OCTET_STRING = 0x41,
  ZCL_CHARACTER_STRING = 0x42, 
  ZCL_LONG_OCTET_STRING = 0x43, 
  ZCL_LONG_CHAR_STRING = 0x44, 
  ZCL_ARRAY_ORDERED = 0x48,
  ZCL_STRUCTURE_ORDERED = 0x4c, 
  ZCL_SET_COLLECTION = 0x50, 
  ZCL_BAG_COLLECTION = 0x51, 
  ZCL_TIME_OF_DAY = 0xe0,
  ZCL_DATE_OF_DAY = 0xe1, 
  ZCL_UTC_TIME = 0xe2, 
  ZCL_CLUSTER_ID = 0xe8, 
  ZCL_ATTRIBUTE_ID = 0xe9,
  ZCL_BACNET_OID = 0xea, 
  ZCL_IEEE_ADDRESS = 0xf0, 
  ZCL_SECURITY_KEY_128BITS = 0xf1, 
  ZCL_UNKNOWN = 0xff
} zcl_data_type_t;

typedef struct {
    zcl_data_type_t type;
    union {
        int8_t data_i8;
        uint8_t data_u8;
        int16_t data_i16;
        uint16_t data_u16;
        int32_t data_i32;
        uint32_t data_u32;
        int64_t data_i64;
        uint64_t data_u64;
        uint8_t data_bool;
        float data_float;
        uint8_t data_arr[ZNP_DATA_LEN_MAX];
        uint8_t data_arr_len;
    };
} zcl_cluster_record_t;

extern void znp_cmd_init(void);

extern int znp_cmd_dev_is_active(uint16_t address);

extern int znp_cmd_dev_refresh_info(uint16_t address);

extern int znp_cmd_dev_get_ieee(uint16_t address, uint64_t *adr_ieee);

extern int znp_cmd_dev_register(uint16_t address);

extern int znp_cmd_cluster_in_read(uint16_t address, uint16_t cluster, uint16_t attribute, zcl_cluster_record_t* record);

extern int znp_cmd_cluster_in_write(uint16_t address, uint16_t cluster, uint16_t attribute, zcl_cluster_record_t* record);

#endif
