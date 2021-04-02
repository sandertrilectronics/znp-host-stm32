/*
 * mtAppCfg.c
 *
 *  Created on: Apr 2, 2021
 *      Author: Sande
 */

#include "mtAppCfg.h"

uint8_t appCfgSetChannel() {
	uint8_t status;

	uint8_t payload[5] = { 0 };
	status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_APP_CFG), 0x08, payload, 5);

	if (status == MT_RPC_SUCCESS) {
		rpcWaitMqClientMsg(50);
	}

	return status;
}
