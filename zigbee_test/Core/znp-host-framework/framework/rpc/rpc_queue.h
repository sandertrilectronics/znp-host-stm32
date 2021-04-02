/*
 * mqueue.h
 *
 * This module contains the POSIX wrapper for Semaphore.
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

#ifndef RPC_QUEUE_H
#define RPC_QUEUE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "queue.h"

typedef struct
{
	QueueHandle_t queue;
} llq_t;

/*********************************************************************
 * @fn      llq_open
 *
 * @brief   Create a queue handle
 *
 * @param    llq_t *hndl - handle to queue to be created
 *
 * @return   none
 */
extern void llq_open(llq_t *hndl);

/*********************************************************************
 * @fn      llq_timedreceive
 *
 * @brief   Block until a message is recieved or timeout
 *
 * @param   llq_t *hndl - handle to queue to read the message from
 * @Param	char *buffer - Pointer to buffer to read the message in to
 * @Param	int maxLength - Max length of message to read
 * @Param	struct timespec * timeout - Timeout value
 *
 * @return   length of message read from queue
 */
extern void llq_close(llq_t *hndl);

/*********************************************************************
 * @fn      llq_add
 *
 * @brief   write message to queue
 *
 * @param   llq_t *hndl - handle to queue to read the message from
 * @Param	char *buffer - Pointer to buffer containing the message
 * @Param	int len - Length of message
 * @Param	int prio - 1 message has priority and should be added to
 * 			head of queue, 0 message assed to tail of queue
 *
 * @return   length of message read from queue
 */
extern int llq_add(llq_t *hndl, char *buffer, int len, int prio);

/*********************************************************************
 * @fn      llq_timedreceive
 *
 * @brief   Block until a message is recieved
 *
 * @param   llq_t *hndl - handle to queue to read the message from
 * @Param	char *buffer - Pointer to buffer to read the message in to
 * @Param	int maxLength - Max length of message to read
 *
 * @return   length of message read from queue
 */
extern int llq_receive(llq_t *hndl, char *buffer, int maxLength);

/*********************************************************************
 * @fn      llq_timedreceive
 *
 * @brief   Block until a message is recieved or timeout
 *
 * @param   llq_t *hndl - handle to queue to read the message from
 * @Param	char *buffer - Pointer to buffer to read the message in to
 * @Param	int maxLength - Max length of message to read
 * @Param	struct timespec * timeout - Timeout value
 *
 * @return   length of message read from queue
 */
extern int llq_timedreceive(llq_t *hndl, char *buffer, int maxLength, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* SEMAPHORE_H */
