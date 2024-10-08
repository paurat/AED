// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
/*
 * lps22.cpp
 *
 */

#include "lps22.h"

I2C_HandleTypeDef * i2c;

void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value){

	uint8_t data[] = {Reg, Value};
	HAL_I2C_Master_Transmit(SENSORS_I2C, Addr, data, 2, 100);
}

uint8_t  SENSOR_IO_Read(uint8_t Addr, uint8_t Reg, uint8_t* state = NULL){

	uint8_t data = 0;
	uint8_t st = HAL_I2C_Mem_Read(SENSORS_I2C, Addr, Reg, 1, &data, 1, 100);

	if(state != NULL) *state = (st == HAL_OK);

	return data;
}

bool lps22hb_init(I2C_HandleTypeDef * i2c_handler){

	i2c = i2c_handler;

	if(HAL_I2C_IsDeviceReady(i2c, LPS_LOWER_ADDRESS, 3, 1000) != HAL_OK) return false;

	uint8_t tmp;

	/* Set Power mode */
	tmp = SENSOR_IO_Read(LPS_LOWER_ADDRESS, LPS22HB_RES_CONF_REG);

	tmp &= ~LPS22HB_LCEN_MASK;
	tmp |= (uint8_t)0x01; /* Set low current mode */

	/* Read CTRL_REG1 */
	tmp = SENSOR_IO_Read(LPS_LOWER_ADDRESS, LPS22HB_CTRL_REG1);

	/* Set default ODR */
	tmp &= ~LPS22HB_ODR_MASK;
	tmp |= (uint8_t)0x20; /* Set ODR to 10Hz */

	/* Enable BDU */
	tmp &= ~LPS22HB_BDU_MASK;
	tmp |= ((uint8_t)0x02);

	/* Apply settings to CTRL_REG1 */
	SENSOR_IO_Write(LPS_LOWER_ADDRESS, LPS22HB_CTRL_REG1, tmp);


	// включаем работу DATA_READY пина
	tmp = SENSOR_IO_Read(LPS_LOWER_ADDRESS, LPS22HB_CTRL_REG3);

	SENSOR_IO_Write(LPS_LOWER_ADDRESS, LPS22HB_CTRL_REG3, tmp | LPS22HB_DRDY_MASK);

	return true;
}


uint8_t lps22hb_get_status() {
	uint8_t status = SENSOR_IO_Read(LPS_LOWER_ADDRESS, LPS22HB_STATUS_REG);
	return status;
}

bool lps22hb_check_pressure_overrun(uint8_t status) {
	bool is_overrun = status & 0x10;
	if (is_overrun) {
		debug("Pressure data overrun! SR %02x\r\n", status);
	}

	return is_overrun;
}

bool lps22hb_check_temperature_overrun(uint8_t status) {
	bool is_overrun = status & 0x20;
	if (is_overrun) {
		debug("Temperature data overrun! SR %02x\r\n", status);
	}

	return is_overrun;
}

bool lps22hb_check_pressure_data_available(uint8_t status) {
	return status & 0x01;
}

bool lps22hb_check_temperature_data_available(uint8_t status) {
	return status & 0x02;
}

uint8_t lps22hb_read_id(){

	  uint8_t ctrl = 0x00;

	  /* Read value at Who am I register address */
	  ctrl = SENSOR_IO_Read(LPS_LOWER_ADDRESS, LPS22HB_WHO_AM_I_REG);

	  return ctrl;
}

int32_t raw_press = 0;

float lps22hb_read_pressure(){

	uint8_t buffer[3];
	for(uint8_t i = 0; i < sizeof(buffer); i++) {
		buffer[i] = SENSOR_IO_Read(LPS_LOWER_ADDRESS, (LPS22HB_PRESS_OUT_XL_REG + i));
	}

	/* Build the raw data */
	uint32_t tmp = 0;
	for(int i = 0; i < 3; i++) {
		tmp |= (((uint32_t)buffer[i]) << (8 * i));
	}

	/* convert the 2's complement 24 bit to 2's complement 32 bit */
	if(tmp & 0x00800000) {
		tmp |= 0xFF000000;
	}

	raw_press = ((int32_t)tmp);
	raw_press = (raw_press * 100) / 4096;

	return (float)((float)raw_press / 100.0f);
}

float lps22hb_read_temperature(){

	  int16_t raw_data;
	  uint8_t buffer[2];
	  uint16_t tmp;
	  uint8_t i;

	  for(i = 0; i < 2; i++)
	  {
	    buffer[i] = SENSOR_IO_Read(LPS_LOWER_ADDRESS, (LPS22HB_TEMP_OUT_L_REG + i));
	  }

	  /* Build the raw tmp */
	  tmp = (((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0];

	  raw_data = (tmp * 10) / 100;

	  return ((float)(raw_data / 10.0f));
}


