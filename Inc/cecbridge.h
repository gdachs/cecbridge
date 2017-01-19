/*
 * cecbridge for SolidPC
 *
 * Copyright 2016 Gerald Dachs <gda@dachsweb.de>
 * based on work from kliment
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version of 2 of the License, or (at your
 * option) any later version. See the file COPYING in the main directory of
 * this archive for more details.
 */

#ifndef __CECBRIDGE_H
#define __CECBRIDGE_H

CEC_HandleTypeDef *getCecHandle(void);

enum ErrorCode {
    NO_ERROR,
    PARAMETER_ERROR,
    CEC_ERROR,
};

#define CEC_MAX_MSG_SIZE        16

typedef struct {
    uint8_t logical_address;
    uint8_t bit_field[2];
    uint8_t physical_address[2];
    uint8_t device_type;
    uint8_t retry_count;
    uint8_t configuration_bits[2];
    char osd_name[15];
} cecbridge_t;

void set_host_power_state(int8_t state);

void FLASH_PageErase(uint32_t PageAddress);

#endif /* __CECBRIDGE_H */
