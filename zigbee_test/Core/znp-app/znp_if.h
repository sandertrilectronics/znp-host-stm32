#ifndef __ZNP_IF_H__
#define __ZNP_IF_H__

#include <stdint.h>

#define DEVICE_MEM_MAX                  32
#define CLSTR_LIST_MAX                  16
#define EV_QUEUE_SIZE                   16

typedef struct {
    uint16_t adr_short;
    uint16_t adr_ieee;
    uint16_t profile_id;
    uint16_t device_id;
    uint8_t clstr_in_cnt;
    uint16_t clstr_in_list[CLSTR_LIST_MAX];
    uint8_t clstr_out_cnt;
    uint16_t clstr_out_list[CLSTR_LIST_MAX];
} znp_device_t;

typedef enum {
    EVT_NONE,
    EVT_RSP_IS_ACTIVE,
    EVT_RSP_SIMPLE_DESC,
    EVT_RSP_REGISTER,
    EVT_RSP_DATA_REQUEST,
} event_type_t;

typedef struct {
    event_type_t type;
    uint8_t result;
} event_result_t;

extern void znp_if_init(void);

extern void znp_if_evt_send(event_result_t *res);

extern uint8_t znp_if_wait_for_event(event_type_t event_to_wait_for, uint32_t timeout);

extern znp_device_t* znp_if_dev_get(uint16_t address);

extern uint8_t znp_if_dev_exists(uint16_t address);

extern int znp_if_dev_add(uint16_t address);

extern int znp_if_dev_set_ieee(uint16_t address, uint64_t ieee_adr);

#endif