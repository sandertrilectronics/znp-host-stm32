#include "log.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h"
#include <stdarg.h>

// semaphore
static SemaphoreHandle_t dbg_sem;

// small local working buffer
static char working_buffer[256];

void log_init(void) {
    dbg_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(dbg_sem);
}

void log_print(const char *fmt, ...) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        // take semaphore
        if (xSemaphoreTake(dbg_sem, 1000) == pdFALSE)
            return;

        // append tick
        snprintf(working_buffer, 256, "%lu\t", xTaskGetTickCount());

        // append parameters
        va_list args;
        va_start(args, fmt);
        vsnprintf(&working_buffer[strlen(working_buffer)], 256 - strlen(working_buffer), fmt, args);
        va_end(args);

        // send data
        HAL_UART_Transmit(&huart2, (char *)working_buffer, strlen(working_buffer), 100);

        // Give semaphore back
        xSemaphoreGive(dbg_sem);
    } 
    else {
        // append parameters
        va_list args;
        va_start(args, fmt);
        vsnprintf(working_buffer, 256, fmt, args);
        va_end(args);

        // send data
        HAL_UART_Transmit(&huart2, (char *)working_buffer, strlen(working_buffer), 100);
    }
}