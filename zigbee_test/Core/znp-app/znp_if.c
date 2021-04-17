#include "znp_if.h"
#include <stddef.h>
#include "FreeRTOS.h"
#include "queue.h"

static znp_device_t _dev_mem[DEVICE_MEM_MAX];
static QueueHandle_t _znp_ev_queue;

void znp_if_init(void) {
    _znp_ev_queue = xQueueCreate(EV_QUEUE_SIZE, sizeof(event_result_t));
}

void znp_if_evt_send(event_result_t* res) {
    xQueueSendToFront(_znp_ev_queue, res, 100);
}

uint8_t znp_if_wait_for_event(event_type_t event_to_wait_for, uint32_t timeout) {
    uint32_t waittime = timeout;
    uint32_t start = xTaskGetTickCount();
    event_result_t res;

    // loop
    while (1) {
        // calculate new timeout
        uint32_t passed_time = start - xTaskGetTickCount();
        waittime -= passed_time;
        start = xTaskGetTickCount();

        // handle message
        if (rpcWaitMqClientMsg(waittime) != 0)
            break;

        // bit was set?
        if (xQueueReceive(_znp_ev_queue, &res, 0) == pdTRUE) {
            if (res.type == event_to_wait_for) {
                break;
            }
            else {
                xQueueSendToBack(_znp_ev_queue, &res, 0);
            }
        }
    }

    log_print("Done %d %d\r\n", res.type, res.result);

    // did we get the right event type?
    if (res.type == event_to_wait_for) {
        if (res.result == 0)
            return 1;
        else
            return 0;
    }
    // bit not set, timeout
    else {
        return 0;
    }
}

znp_device_t* znp_if_dev_get(uint16_t address) {
    // invalid address?
    if (address == 0x0000 || address == 0xFFFF)
        return NULL;

    // check if device already exists
    for (uint8_t i = 0; i < DEVICE_MEM_MAX; i++) {
        if (_dev_mem[i].adr_short == address) {
            return &_dev_mem[i];
        }
    }
    return NULL;
}

uint8_t znp_if_dev_exists(uint16_t address) {
    // check if device already exists
    if (znp_if_dev_get(address) == NULL)
        return 0;
    else
        return 1;
}

int znp_if_dev_add(uint16_t address) {
    // check if device already exists
    if (znp_if_dev_exists(address))
        return 0;

    // search for a free spot
    uint8_t index;
    for (index = 0; index < DEVICE_MEM_MAX; index++) {
        if (_dev_mem[index].adr_short == 0x0000) {
            break;
        }
    }

    // no free spot found?
    if (index >= DEVICE_MEM_MAX)
        return -1;

    // free spot, remember address
    _dev_mem[index].adr_short = address;

    // all good
    return 0;
}

int znp_if_dev_set_ieee(uint16_t address, uint64_t ieee_adr) {
    // get device handle
    znp_device_t* dev = znp_if_dev_get(address);

    // invalid handle?
    if (dev == NULL)
        return -1;

    // save ieee
    dev->adr_ieee = ieee_adr;

    // all good
    return 0;
}