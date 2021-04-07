/*
 * rpc.h
 *
 * This module contains the RPC (Remote Procedure Call) API for the
 * ZigBee Network Processor (ZNP) Host Interface.
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef RPC_H
#define RPC_H

#ifdef __cplusplus
extern "C"
{
#endif

/***********************************************************************************
 * INCLUDES
 */
#include <stdint.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

//
// RPC (Remote Procedure Call) definitions
//
// SOF (Start of Frame) indicator byte byte
#define MT_RPC_SOF                 (0xFE)

// The 3 MSB's of the 1st command field byte (Cmd0) are for command type
#define MT_RPC_CMD_TYPE_MASK       (0xE0)

// The 5 LSB's of the 1st command field byte (Cmd0) are for the subsystem
#define MT_RPC_SUBSYSTEM_MASK      (0x1F)

// maximum length of RPC frame
// (1 byte length + 2 bytes command + 0-250 bytes data)
#define RPC_MAX_LEN                (256)

// RPC Frame field lengths
#define RPC_UART_SOF_LEN           (1)
#define RPC_UART_FCS_LEN           (1)

#define RPC_UART_FRAME_START_IDX   (1)

#define RPC_LEN_FIELD_LEN          (1)
#define RPC_CMD0_FIELD_LEN         (1)
#define RPC_CMD1_FIELD_LEN         (1)

#define RPC_HDR_LEN                (RPC_LEN_FIELD_LEN + RPC_CMD0_FIELD_LEN + \
		                            RPC_CMD1_FIELD_LEN)

#define RPC_UART_HDR_LEN           (RPC_UART_SOF_LEN + RPC_HDR_LEN)

/***********************************************************************************
 * TYPEDEFS
 */

// Cmd0 Command Type
typedef enum
{
	MT_RPC_CMD_POLL = 0x00,  // POLL command
	MT_RPC_CMD_SREQ = 0x20,  // SREQ (Synchronous Request) command
	MT_RPC_CMD_AREQ = 0x40,  // AREQ (Acynchronous Request) command
	MT_RPC_CMD_SRSP = 0x60,  // SRSP (Synchronous Response)
	MT_RPC_CMD_RES4 = 0x80,  // Reserved
	MT_RPC_CMD_RES5 = 0xA0,  // Reserved
	MT_RPC_CMD_RES6 = 0xC0,  // Reserved
	MT_RPC_CMD_RES7 = 0xE0   // Reserved
} mtRpcCmdType_t;

// Cmd0 Command Subsystem
typedef enum
{
	MT_RPC_SYS_RES0,   // Reserved.
	MT_RPC_SYS_SYS,    // SYS interface
	MT_RPC_SYS_MAC,
	MT_RPC_SYS_NWK,
	MT_RPC_SYS_AF,     // AF interface
	MT_RPC_SYS_ZDO,    // ZDO interface
	MT_RPC_SYS_SAPI,   // Simple API interface
	MT_RPC_SYS_UTIL,   // UTIL interface
	MT_RPC_SYS_DBG,
	MT_RPC_SYS_APP,
	MT_RPC_SYS_OTA,
	MT_RPC_SYS_ZNP,
	MT_RPC_SYS_SPARE_12,
	MT_RPC_SYS_SBL = 13, // 13 to be compatible with existing RemoTI - AKA MT_RPC_SYS_UBL
	MT_RPC_SYS_MAX, // Maximum value, must be last (so 14-32 available, not yet assigned).
	MT_RPC_SYS_APP_CFG = 15
} mtRpcSysType_t;

// Error codes in Attribute byte of SRSP packet
typedef enum
{
	MT_RPC_SUCCESS = 0,         // success
	MT_RPC_ERR_SUBSYSTEM = 1,   // invalid subsystem
	MT_RPC_ERR_COMMAND_ID = 2,  // invalid command ID
	MT_RPC_ERR_PARAMETER = 3,   // invalid parameter
	MT_RPC_ERR_LENGTH = 4       // invalid length
} mtRpcErrorCode_t;

/***********************************************************************************
 * GLOBAL VARIABLES
 */

/***********************************************************************************
 * GLOBAL FUNCTIONS
 */

int32_t rpcOpen(void);
void rpcClose(void);
int32_t rpcProcess(void);
uint8_t rpcSendFrame(uint8_t cmd0, uint8_t cmd1, uint8_t * payload,
        uint8_t payload_len);
void rpcForceRun(void);
int32_t rpcInitMq(void);
int32_t rpcGetMqClientMsg(void);
int32_t rpcWaitMqClientMsg(uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* RPC_H */
