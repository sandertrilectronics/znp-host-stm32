#ifndef __ZNP_CMD_H__
#define __ZNP_CMD_H__

#include <stdint.h>

extern int znp_cmd_dev_is_active(uint16_t address);

extern int znp_cmd_dev_refresh_info(uint16_t address);

extern int znp_cmd_dev_register(uint16_t address);

#endif