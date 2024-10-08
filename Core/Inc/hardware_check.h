/*
 * hardware_check.h
 *
 */
#pragma once

#ifndef INC_HARDWARE_CHECK_H_
#define INC_HARDWARE_CHECK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "i2c.h"
#include "spi.h"

#include "hdc1080.h"
#include "lps22.h"
#include "lis331dlh.h"

typedef enum{
	SENSORS_BAD_ALL_SENSORS,
	SENSORS_BAD_HDC_AND_LPS,
	SENSORS_BAD_HDC_AND_LIS,
	SENSORS_BAD_HDC,
	SENSORS_BAD_LPS_AND_LIS,
	SENSORS_BAD_LPS,
	SENSORS_BAD_LIS,
	SENSORS_OK
} Sensors_contidition_t;


uint8_t check_hdc1080();
uint8_t check_lps22();
uint8_t check_lis331dlh();

void check_all_hardware();

void debug_all_hardware();

#ifdef __cplusplus
}
#endif

#endif /* INC_HARDWARE_CHECK_H_ */
