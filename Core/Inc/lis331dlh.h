#pragma once

#ifndef INC_LIS3DH_H_
#define INC_LIS3DH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "spi.h"

#define LIS331DLH_WHO_AM_I_VAL 0x32


typedef enum {
	FULL_SCALE_2G = 0b00,
	FULL_SCALE_4G = 0b01,
	FULL_SCALE_8G = 0b11,
} acceleration_range_t;


typedef enum {
	DATA_RATE_50 	= 0b00,
	DATA_RATE_100 	= 0b01,
	DATA_RATE_400 	= 0b10,
	DATA_RATE_1000 	= 0b11,
} data_rate_t;


typedef enum {
	AXIS_Z	= 0U,
	AXIS_X	= 1U,
} axis_select_t;


typedef struct {
	uint32_t timestamp;
	int32_t x;
	int32_t y;
	int32_t z;
} accelaration_t;


typedef struct {
	uint8_t config;
	uint8_t source;
	uint8_t threshold;
	uint8_t duration;
} interrupt_register_t;


typedef struct {
	SPI_HandleTypeDef * spi;
	I2C_HandleTypeDef * i2c;

	GPIO_TypeDef * chip_select_port;
	uint16_t chip_select_pin;

	data_rate_t data_rate;
	acceleration_range_t range;
	float range_factor;

	axis_select_t axis_select_x_z;

	uint8_t device_id;

	uint8_t control_registers[5];

	uint8_t reference;
	uint8_t status_register;

	uint8_t out_x_low;
	uint8_t out_x_high;

	uint8_t out_y_low;
	uint8_t out_y_high;

	uint8_t out_z_low;
	uint8_t out_z_high;

	interrupt_register_t interrupt_1;
	interrupt_register_t interrupt_2;

	accelaration_t accelarations;

	float zero_acceleration_for_z;
	float one_g_acceleration_for_z;
} lis331dlh_t;


float DATA_RATE_TO_TIME(data_rate_t data_rate);

int lis331dlh_init_spi(lis331dlh_t *, SPI_HandleTypeDef *, GPIO_TypeDef *, uint16_t);
int lis331dlh_init_i2c(lis331dlh_t *, I2C_HandleTypeDef *);

void lis331dlh_update_config(lis331dlh_t *);
int lis331dlh_update_accelaration(lis331dlh_t *);

uint8_t lis331dlh_read_status(lis331dlh_t *);
//int lis331dlh_is_ready(lis331dlh_t *);

//int lis331dlh_set_zero(lis331dlh_t *);

uint16_t lis331dlh_print_config(lis331dlh_t *);

#ifdef __cplusplus
}
#endif

#endif /* INC_LIS3DH_H_ */
