/*
 * mtUtil.h
 *
 *  Created on: Apr 12, 2021
 *      Author: Sande
 */

#ifndef _MT_UTIL_MTUTIL_H_
#define _MT_UTIL_MTUTIL_H_

#include <stdint.h>

#define MT_UTIL_SRSP_GET_DEVICE_INFO		0x00

#define ASS_DEVICE_LIST_MAX					32

typedef struct {
	uint8_t success;
	uint64_t ieee_addr;
	uint16_t short_addr;
	uint8_t device_type;
	uint8_t device_state;
	uint8_t ass_device_cnt;
	uint16_t ass_device_list[ASS_DEVICE_LIST_MAX];
} utilGetDeviceInfoFormat_t;

typedef uint8_t (*mtUtilGetDeviceInfoCb_t)(utilGetDeviceInfoFormat_t *msg);

typedef struct {
	mtUtilGetDeviceInfoCb_t pfnUtilGetDeviceInfoCb_t;
} mtUtilCb_t;

void utilRegisterCallbacks(mtUtilCb_t cbs);
void utilProcess(uint8_t *rpcBuff, uint8_t rpcLen);
uint8_t utilGetDeviceInfo(void);

#endif /* ZNP_HOST_FRAMEWORK_FRAMEWORK_MT_UTIL_MTUTIL_H_ */
