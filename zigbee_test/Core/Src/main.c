/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "dbgPrint.h"
#include "mtAf.h"
#include "mtAppCfg.h"
#include "mtParser.h"
#include "mtSys.h"
#include "mtUtil.h"
#include "mtZdo.h"
#include "rpc.h"
#include "rpcTransport.h"
#include "znp_cmd.h"
#include "znp_if.h"
#include "log.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/////////////////////////////////////////////////

typedef enum {
    ZB_DANFOSS_ALLY,
} airmon_zigbee_device_t;

typedef struct {
    uint8_t is_initialized;
    uint32_t last_poll_time;
} zigbee_device_info_t;

typedef struct {
    char manufact_name[32];
    char device_name[32];
    airmon_zigbee_device_t type;
    uint16_t adr_short;
    uint64_t adr_ieee;

    zigbee_device_info_t info; // volatile, should not be saved
} zigbee_device_t;

zigbee_device_t *zigbee_device_get_free_handle(void) {
    static zigbee_device_t device = {0};
    return &device;
}

void zigbee_device_save(zigbee_device_t *dev) {

}

void task_maintain_devices(void *param) {

    while (1) {

    }
}

void task_register_device(void *param) {
    // parse parameter
    uint16_t *adr_p = (uint16_t *)param;
    uint16_t adr = *adr_p;

    // feedback
    log_print("[0x%04x] -> Device init started\r\n", adr);

    // get free device handle
    zigbee_device_t *dev = zigbee_device_get_free_handle();

    // sanity check
    if (dev == NULL) {
        // feedback
        log_print("[0x%04x] -> No more free handles\r\n", adr);

        // done, stop the task
        goto end_task;
    }

    // update device address
    dev->adr_short = adr;

    // check if the device already exists
    if (!znp_if_dev_exists(dev->adr_short)) {
        // feedback
        log_print("[0x%04x] -> Device not registered\r\n", dev->adr_short);

        // done, stop the task
        goto end_task;
    }

    // refresh device info
    if (znp_cmd_dev_refresh_info(dev->adr_short) != 0) {
        // feedback
        log_print("[0x%04x] -> Device couldn't be refreshed\r\n", dev->adr_short);

        // done, stop the task
        goto end_task;
    }

    // register device info
    if (znp_cmd_dev_register(dev->adr_short) != 0) {
        // feedback
        log_print("[0x%04x] -> Device couldn't be registered\r\n", dev->adr_short);

        // done, stop the task
        goto end_task;
    }

    // read ieee address
    if (znp_cmd_dev_get_ieee(dev->adr_short, &dev->adr_ieee) != 0) {
        // feedback
        log_print("[0x%04x] -> Device IEEE couldn't be read\r\n", dev->adr_short);

        // done, stop the task
        goto end_task;
    }

    // read manufacturer name cluster
    zcl_cluster_record_t wr;
    if (znp_cmd_cluster_in_read(dev->adr_short, 0x0000, 0x0004, &wr) != 0) {
        // feedback
        log_print("[0x%04x] -> Device couldn't be read\r\n", dev->adr_short);

        // done, stop the task
        goto end_task;
    }

    // check received type
    if (wr.type != ZCL_CHARACTER_STRING) {
        // feedback
        log_print("[0x%04x] -> Device response type not ok\r\n", dev->adr_short);

        // done, stop the task
        goto end_task;
    }

    // save the name
    memcpy(dev->manufact_name, wr.data_arr, MIN(32, wr.data_arr_len));

    // read device name cluster
    if (znp_cmd_cluster_in_read(dev->adr_short, 0x0000, 0x0005, &wr) != 0) {
        // feedback
        log_print("[0x%04x] -> Device couldn't be read\r\n", dev->adr_short);

        // done, stop the task
        goto end_task;
    }

    // check received type
    if (wr.type != ZCL_CHARACTER_STRING) {
        // feedback
        log_print("[0x%04x] -> Device response type not ok\r\n", dev->adr_short);

        // done, stop the task
        goto end_task;
    }

    // save the name
    memcpy(dev->device_name, wr.data_arr, MIN(32, wr.data_arr_len));

    // print device info
    uint32_t top = dev->adr_ieee >> 32;
    uint32_t bot = dev->adr_ieee & 0xffffffff;
    log_print("[0x%04x] -> IEEE Address: 0x%08x%08x\r\n", dev->adr_short, top, bot);
    log_print("[0x%04x] -> Manufacturer name: %s\r\n", dev->adr_short, dev->manufact_name);
    log_print("[0x%04x] -> Device name: %s\r\n", dev->adr_short, dev->device_name);

    // save the device
    zigbee_device_save(dev);

// stop the task
end_task:
    vTaskDelete(NULL);
}

void device_active_response(uint16_t adr) {
    xTaskCreate(task_register_device, "dev-reg", 1024, (void *)&adr, 6, NULL);
    vTaskDelay(10);
}

void rcpWaitPeriod(uint32_t period) {
	uint32_t waittime = period;
	uint32_t start = xTaskGetTickCount();
	while (rpcWaitMqClientMsg(waittime) == 0) {
		uint32_t passed_time = start - xTaskGetTickCount();
		waittime -= passed_time;
		start = xTaskGetTickCount();
	}
}

void znp_nvm_reset(void) {
	ResetReqFormat_t rst;
	OsalNvWriteFormat_t req;

	// Clear complete memory of CC2530
	req.Id = 0x0003;
	req.Offset = 0x00;
	req.Len = 0x01;
	req.Value[0] = 0x03;
	sysOsalNvWrite(&req);

	// hard reset
	rst.Type = 0x00;
	sysResetReq(&rst);
}

// init coordinator
// taken from https://sunmaysky.blogspot.com/2017/02/use-ztool-z-stack-30-znp-to-set-up.html
int znp_init_coordinator(uint8_t enable_commissioning) {
	OsalNvWriteFormat_t req;
	setChannelFormat_t chn;
	startCommissioningFormat_t strt;
	ResetReqFormat_t rst;

	// wait a second
	vTaskDelay(1000);
	log_print("1 ----------------------\r\n");

	// soft reset
	rst.Type = 0x01;
	sysResetReq(&rst);

	vTaskDelay(4000);
	log_print("2 ----------------------\r\n");

	// Write ZCD_NV_LOGICAL_TYPE to 0 which means coordinator
	req.Id = 0x0087;
	req.Offset = 0x00;
	req.Len = 0x01;
	req.Value[0] = 0x00;
	sysOsalNvWrite(&req);

	vTaskDelay(1000);
	log_print("3 ----------------------\r\n");

	// set primary channel to 13
	chn.primaryChannel = 1;
	chn.channel = CFG_CHANNEL_0x00002000;
	appCfgSetChannel(&chn);

	vTaskDelay(1000);
	log_print("4 ----------------------\r\n");

	// disable secondary channel
	chn.primaryChannel = 0;
	chn.channel = CFG_CHANNEL_NONE;
	appCfgSetChannel(&chn);

	vTaskDelay(1000);
	log_print("5 ----------------------\r\n");

	// start commissioning using network formation
	strt.commissioningMode = CFG_COMM_MODE_NWK_FORMATION;
	appCfgStartCommissioning(&strt);

	vTaskDelay(10000);
	log_print("6 ----------------------\r\n");

	if (enable_commissioning) {
		// get device info
		utilGetDeviceInfo();

		vTaskDelay(1000);
		log_print("7 ----------------------\r\n");

		// Write ZCD_NV_LOGICAL_TYPE to 0 which means coordinator
		req.Id = 0x008F;
		req.Offset = 0x00;
		req.Len = 0x01;
		req.Value[0] = 0x01;
		sysOsalNvWrite(&req);

		vTaskDelay(1000);
		log_print("8 ----------------------\r\n");

		// start commissioning using network steering
		strt.commissioningMode = CFG_COMM_MODE_NWK_STEERING;
		appCfgStartCommissioning(&strt);
	}
	else {
		// get device info
		utilGetDeviceInfo();
	}

	//
	return 0;
}

void register_clusters(uint16_t addr) {
    vTaskDelay(20000);

    // check registration
    if (!znp_if_dev_exists(addr)) {
        log_print("-> !!! Device not registered!\r\n");
        return;
    }

    log_print("9 ----------------------\r\n");

    // wait for device to be active
    while (1) {
        int ret = znp_cmd_dev_is_active(addr);
        log_print("znp_if_dev_is_active %d\r\n", ret);
        if (ret == 0)
            break;
        vTaskDelay(1000);
    }
    log_print("10 ----------------------\r\n");

    // refresh device info
    log_print("znp_if_dev_refresh_info %d\r\n", znp_cmd_dev_refresh_info(addr));
    vTaskDelay(1000);
    log_print("11 ----------------------\r\n");

    // register device
    log_print("znp_if_dev_register %d\r\n", znp_cmd_dev_register(addr));
    vTaskDelay(1000);
    log_print("12 ----------------------\r\n");

    // read device name cluster
    zcl_cluster_record_t wr;
    log_print("znp_cmd_cluster_in_read %d\r\n", znp_cmd_cluster_in_read(addr, 0, 4, &wr));
    log_print("Type: %d\r\n", wr.type);
    log_print("Str: %s\r\n", wr.data_arr);

    // write thermostat to 19 degree
    wr.type = ZCL_SIGNED_16BITS;
    wr.data_i16 = 1900;
    log_print("znp_cmd_cluster_in_write %d\r\n", znp_cmd_cluster_in_write(addr, 0x0201, 0x0012, &wr));

    // read thermostat value
    log_print("znp_cmd_cluster_in_read %d\r\n", znp_cmd_cluster_in_read(addr, 0x0201, 0x0012, &wr));
    log_print("Type: %d\r\n", wr.type);
    log_print("Data: %d\r\n", wr.data_i16);
}

/////////////////////////////////////////////////
void vAppTask(void *pvParameters) {
	log_print("System started\r\n");

	// initiailze application interface
	znp_if_init();

    // initiailze application interface
    znp_if_init();
    znp_cmd_init();

	// startup delay
	vTaskDelay(1000);

    // ping the CC2530 every second until response is ok
    uint8_t ret = 0;
    do {
        vTaskDelay(1000);
        ret = sysVersion();
    } while (ret != 0);

    // initialize coordinator
    znp_init_coordinator(0);

    // register cluster
    register_clusters(0x82bc);

	// endless loop, handle CC2530 packets
	while (1) {
		vTaskDelay(1000);
	}
}

void vPollTask(void *pvParameters) {
	// endless loop, handle CC2530 packets
	while (1) {
		rpcWaitMqClientMsg(portMAX_DELAY);
	}
}

void vComTask(void *pvParameters) {
	// init queues
	rpcInitMq();

	// initialize serial port
	rpcOpen();

    // start poll task
    xTaskCreate(vPollTask, "POLL", 1024, NULL, 5, NULL);

	// loop
	while (1) {
		// keep procesing packets
		rpcProcess();

        // give other tasks time to run
        vTaskDelay(1);
    }
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
	log_print("-> !!! Stack overflow in %s\r\n", pcTaskName);
	while (1)
		;
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void) {
	/* vApplicationMallocFailedHook() will only be called if
	 configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
	 function that will get called if a call to pvPortMalloc() fails.
	 pvPortMalloc() is called internally by the kernel whenever a task, queue,
	 timer or semaphore is created. It is also called by various parts of the
	 demo application. If heap_1.c or heap_2.c are used, then the size of the
	 heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	 FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	 to query the size of free heap space that remains (although it does not
	 provide information on how the remaining heap might be fragmented). */
	log_print("-> !!! Malloc failed\r\n");
	while (1)
		;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_LPUART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	log_init();

    xTaskCreate(vAppTask, "APP", 2048, NULL, 5, NULL);
    xTaskCreate(vComTask, "COM", 1024, NULL, 6, NULL);

	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 20;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_LPUART1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
