/*
 * rpcTransportUart.c
 *
 * This module contains the API for the zll SoC Host Interface.
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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "usart.h"

#include "rpcTransport.h"
#include "dbgPrint.h"

#include "FreeRTOS.h"
#include "queue.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
/*******************************/
/*** Bootloader Commands ***/
/*******************************/
#define SB_FORCE_BOOT               0xF8
#define SB_FORCE_RUN               (SB_FORCE_BOOT ^ 0xFF)

/************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8_t uartDebugPrintsEnabled = 0;

/*********************************************************************
 * LOCAL VARIABLES
 */
static QueueHandle_t rpc_q_uart_tx;
static QueueHandle_t rpc_q_uart_rx;
static uint32_t isr_stat;
static uint8_t isr_data;

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      rpcTransportOpen
 *
 * @brief   opens the serial port to the CC253x.
 *
 * @param   devicePath - path to the UART device
 *
 * @return  status
 */
int32_t rpcTransportOpen(char *_devicePath, uint32_t port) {
	rpc_q_uart_tx = xQueueCreate(256, sizeof(uint8_t));
	rpc_q_uart_rx = xQueueCreate(256, sizeof(uint8_t));
	return 0;
}

/*********************************************************************
 * @fn      rpcTransportClose
 *
 * @brief   closes the serial port to the CC253x.
 *
 * @param   fd - file descriptor of the UART device
 *
 * @return  status
 */
void rpcTransportClose(void) {
	vQueueDelete(rpc_q_uart_tx);
	vQueueDelete(rpc_q_uart_rx);

	// enable receive interrupt
	SET_BIT(hlpuart1.Instance->CR1, USART_CR1_RXNEIE);
}

/*********************************************************************
 * @fn      rpcTransportISR
 *
 * @brief   Interrupt Service Routine for RPC uart
 */
void rpcTransportISR(void) {
	// read status register
	isr_stat = hlpuart1.Instance->ISR;

	// overrun error?
	if (isr_stat & USART_ISR_ORE)
		hlpuart1.Instance->ICR |= USART_ICR_ORECF;

	// check for rx interrupt
	if (isr_stat & USART_ISR_RXNE) {
		// read the data
		isr_data = hlpuart1.Instance->RDR & 0xFF;
		// check for errors
		if ((isr_stat & (USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) == 0) {
			// Transmit data to queue
			xQueueSendFromISR(rpc_q_uart_rx, (void*) &isr_data, NULL);
		}
	}

	// check for data to send
	if (isr_stat & USART_ISR_TC) {
		// grab data from fifo
		if (xQueueReceiveFromISR(rpc_q_uart_tx, (void*) &isr_data, NULL) == pdFALSE) {
			// end of transmission
			CLEAR_BIT(hlpuart1.Instance->CR1, USART_CR1_TCIE);
		}
		else {
			// send another byte
			hlpuart1.Instance->TDR = isr_data;
		}
	}
}

/*********************************************************************
 * @fn      rpcTransportWrite
 *
 * @brief   Write to the the serial port to the CC253x.
 *
 * @param   fd - file descriptor of the UART device
 *
 * @return  status
 */
void rpcTransportWrite(uint8_t *buf, uint8_t len) {
	for (uint16_t i = 0; i < len; i++) {
		// add data to tx queue
		xQueueSend(rpc_q_uart_tx, (void*) &buf[i], 1);

		// enable receive and transmit interrupt
		SET_BIT(hlpuart1.Instance->CR1, USART_CR1_RXNEIE);
		SET_BIT(hlpuart1.Instance->CR1, USART_CR1_TCIE);
	}
}

/*********************************************************************
 * @fn      rpcTransportRead
 *
 * @brief   Reads from the the serial port to the CC253x.
 *
 * @param   fd - file descriptor of the UART device
 *
 * @return  status
 */
uint8_t rpcTransportRead(uint8_t *buf, uint8_t len) {
	int index = 0;
	while (xQueueReceive(rpc_q_uart_rx, &buf[index], 0) == pdTRUE) {
		index++;
		if (index == len)
			break;
	}
	return index;
}
