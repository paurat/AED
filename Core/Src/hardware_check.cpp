// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
/*
 * hardware_check.cpp
 *
 */

#include "hardware_check.h"
#include "debug.h"
#include <stdlib.h>
#include "sensors_state.h"

lis331dlh_t test_config = {
	.data_rate = DATA_RATE_1000,
	.range = FULL_SCALE_2G,
};


uint8_t check_hdc1080(){

	uint8_t state = hdc1080_init(SENSORS_I2C, Temperature_Resolution_14_bit, Humidity_Resolution_14_bit);

	if(state == true){
		debug("HDC1080 OK\r\n");
	} else {
		debug("HDC1080 CAN'T INIT\r\n");
	}

	send_debug_messages();

	return state;
}

uint8_t check_lps22(){

	uint8_t state = (LPS22HB_WHO_AM_I_VAL == lps22hb_read_id());

	if(state == true){
		debug("LPS22HB OK\r\n");
	} else {
		debug("LPS22HB CAN'T INIT\r\n");
	}

	send_debug_messages();

	return state;
}

uint8_t check_lis331dlh(){

	lis331dlh_init_spi(&test_config, ACCELEROMETER_SPI, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin);
	lis331dlh_update_config(&test_config);

	uint8_t state = (LIS331DLH_WHO_AM_I_VAL == test_config.device_id);

	if(state == true){
		debug("LIS331DLH OK\r\n");
	} else {
		debug("LIS331DLH CAN'T INIT\r\n");
	}

	send_debug_messages();

	return state;
}

void check_all_hardware(){

	check_hdc1080();
	check_lps22();
	check_lis331dlh();
}

void debug_all_hardware() {

	float temp;
	uint8_t hum;

	hdc1080_start_measurement(&temp, &hum);

	float pressure = lps22hb_read_pressure();
	float last_temperature = lps22hb_read_temperature();

	debug("\r\nHDC1080:\r\nTemperature: %f \r\nHumidity: %d", temp, hum);
	debug("\r\nLPS22HB:\r\nTemperature: %f \r\nPressure: %f", last_temperature, pressure);
}
