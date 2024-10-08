// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
#include "stm32g0xx.h"

#include "lis331dlh.h"
#include "spi.h"
#include "debug.h"


enum REGISTER_NAMES : uint8_t {
	WHO_AM_I = 0x0F,

	CTRL_REG1 = 0x20,
	CTRL_REG2,
	CTRL_REG3,
	CTRL_REG4,
	CTRL_REG5,

	HP_FILTER_RESET,
	REFERENCE,
	STATUS_REG,

	OUT_X_L,
	OUT_X_H,
	OUT_Y_L,
	OUT_Y_H,
	OUT_Z_L,
	OUT_Z_H,

	INT1_CFG = 0x30,
	INT1_SOURCE,
	INT1_THS,
	INT1_DURATION,

	INT2_CFG,
	INT2_SOURCE,
	INT2_THS,
	INT2_DURATION,
};

#define WHO_AM_I_VALUE 0x32


constexpr float ONE_G_VALUE(acceleration_range_t range) {
	float range_value = 0;

	switch(range) {
	case FULL_SCALE_8G:
		range_value = 4096;
		break;

	case FULL_SCALE_4G:
		range_value = 8192;
		break;

	case FULL_SCALE_2G:
	default:
		range_value = 16384;
	}

	return range_value;
}

float DATA_RATE_TO_TIME(data_rate_t data_rate) {
	float integral_factor = 0;

	switch(data_rate) {
	case DATA_RATE_1000:
		integral_factor = 1;
		break;

	case DATA_RATE_100:
		integral_factor = 10;
		break;

	case DATA_RATE_50:
		integral_factor = 20;
		break;

	case DATA_RATE_400:
		integral_factor = 2.5;
		break;
	}

	return integral_factor;
}

void chip_select(lis331dlh_t * config) {
	HAL_GPIO_WritePin(config->chip_select_port, config->chip_select_pin, GPIO_PIN_RESET);
}


void chip_deselect(lis331dlh_t * config) {
	HAL_GPIO_WritePin(config->chip_select_port, config->chip_select_pin, GPIO_PIN_SET);
}


void read_register(
	lis331dlh_t * config, uint8_t address,
	uint8_t * for_receive, uint8_t receive_length = 1
) {
	if (config->spi != NULL) {
		chip_select(config);

		address |= 0xC0;

		HAL_SPI_Transmit(config->spi, &address, 1, 100);
		HAL_SPI_Receive(config->spi, for_receive, receive_length, 100);

		chip_deselect(config);
	}
	else {
		for (uint8_t i = 0; i < receive_length; ++i) {
			uint8_t current = address++;
			HAL_I2C_Master_Transmit(config->i2c, (uint16_t)0x32, &current, 1, 50);
			HAL_I2C_Master_Receive(config->i2c, (uint16_t)0x32, &for_receive[i], 1, 50);
//			HAL_Delay(1);
		}
	}
}


void write_register(lis331dlh_t * config, uint8_t address, uint8_t value) {
	if (config->spi != NULL) {
		chip_select(config);

		HAL_SPI_Transmit(config->spi, &address, 1, 100);
		HAL_SPI_Transmit(config->spi, &value, 1, 10);

		chip_deselect(config);
	}
	else {
		uint8_t buffer[] = { address, value };
		HAL_I2C_Master_Transmit(config->i2c, (uint16_t)0x32, buffer, 2, 100);
	}
}

int lis331dlh_check_presence(lis331dlh_t * config) {

	uint8_t lis331_who_am_i_contents;

	read_register(config, WHO_AM_I, &lis331_who_am_i_contents);

	return(lis331_who_am_i_contents != WHO_AM_I_VALUE);
}

int lis331dlh_init(lis331dlh_t * config) {

	if(lis331dlh_check_presence(config)) {
		return 1;
	}

	config->range_factor = DATA_RATE_TO_TIME(config->data_rate);

	lis331dlh_update_config(config);

//	config->control_registers[0] &= !(0b11 << 3);
//	config->control_registers[0] |= 0x27 | (config->data_rate << 3);

	config->control_registers[0] = 0x27 | (config->data_rate << 3);

//	config->control_registers[1] &= !(0b10011);
//	config->control_registers[1] |= (0 << 4) | 0b00;

	config->control_registers[1] = 0;

//	config->control_registers[3] &= !(0b11 << 5);
//	config->control_registers[3] |= (config->range << 4) | (1 << 7);

	config->control_registers[3] = (config->range << 4) | (1 << 7);

	write_register(config, CTRL_REG1, config->control_registers[0]);
	write_register(config, CTRL_REG2, config->control_registers[1]);
	write_register(config, CTRL_REG3, config->control_registers[2]);
	write_register(config, CTRL_REG4, config->control_registers[3]);
	write_register(config, CTRL_REG5, config->control_registers[4]);

	HAL_Delay(10);

	return 0;
}

int lis331dlh_init_spi(
		lis331dlh_t * config,  SPI_HandleTypeDef * spi,
		GPIO_TypeDef * port, uint16_t pin
) {
	config->spi = spi;
	config->chip_select_port = port;
	config->chip_select_pin = pin;

	config->axis_select_x_z = (axis_select_t) HAL_GPIO_ReadPin(X_Z_SELECT_GPIO_Port, X_Z_SELECT_Pin);

	if(config->axis_select_x_z == AXIS_X) {
		debug("AXIS SELECTED: X\r\n");
	}

	if(config->axis_select_x_z == AXIS_Z) {
		debug("AXIS SELECTED: Z\r\n");
	}

	if(lis331dlh_init(config)) {
		return 1;
	}

	return 0;
}

int lis331dlh_init_i2c(
	lis331dlh_t * config, I2C_HandleTypeDef * i2c
) {
	config->i2c = i2c;

	return lis331dlh_init(config);
}

uint16_t lis331dlh_print_config(lis331dlh_t * config) {
	debug("SR: 0x%02x, Device ID: 0x%02x\r\n",
		config->status_register, config->device_id);

	debug("CTRL_REG_N[5]: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n",
		config->control_registers[0],
		config->control_registers[1],
		config->control_registers[2],
		config->control_registers[3],
		config->control_registers[4]);

	debug("INT1_CFG: 0x%02x, INT1_SRC: 0x%02x, INT1_THS: 0x%02x, INT1_DURACTION: 0x%02x\r\n",
		config->interrupt_1.config,
		config->interrupt_1.source,
		config->interrupt_1.threshold,
		config->interrupt_1.duration);

	debug("INT2_CFG: 0x%02x, INT2_SRC: 0x%02x, INT2_THS: 0x%02x, INT2_DURACTION: 0x%02x\r\n",
		config->interrupt_2.config,
		config->interrupt_2.source,
		config->interrupt_2.threshold,
		config->interrupt_2.duration);

	return 0;
}


inline int16_t get_scaled_accelaration(uint8_t low_byte, uint8_t high_byte, acceleration_range_t range) {
	return static_cast<int16_t>(high_byte << 8 | low_byte); 		// 12 бит число представлено в виде
																	// 16 бит и младшие 4 бита - нулевые
}

int lis331dlh_update_accelaration(lis331dlh_t * config) {
	read_register(config, OUT_X_L, &config->out_x_low, 6);

	config->accelarations = {
		HAL_GetTick(),
		get_scaled_accelaration(config->out_x_low, config->out_x_high, config->range) / 16,
		get_scaled_accelaration(config->out_y_low, config->out_y_high, config->range) / 16,
		get_scaled_accelaration(config->out_z_low, config->out_z_high, config->range) / 16,
	};

	return 0;
}


int lis331dlh_update_accelaration(lis331dlh_t * config, int16_t & x, int16_t & y, int16_t & z) {
	read_register(config, OUT_X_L, &config->out_x_low, 6);

	x =	get_scaled_accelaration(config->out_x_low, config->out_x_high, config->range) / 16;
	y = get_scaled_accelaration(config->out_y_low, config->out_y_high, config->range) / 16;
	z = get_scaled_accelaration(config->out_z_low, config->out_z_high, config->range) / 16;

	return 0;
}


void lis331dlh_update_config(lis331dlh_t * config) {
	read_register(config, WHO_AM_I, &config->device_id);
	read_register(config, STATUS_REG, &config->status_register);

	read_register(config, CTRL_REG1, (uint8_t *)&config->control_registers,
			sizeof(config->control_registers));

	read_register(config, INT1_CFG, (uint8_t *)&config->interrupt_1, sizeof(interrupt_register_t));
	read_register(config, INT2_CFG, (uint8_t *)&config->interrupt_2, sizeof(interrupt_register_t));
}


uint8_t lis331dlh_read_status(lis331dlh_t * config) {
	read_register(config, STATUS_REG, &config->status_register);
	return config->status_register;
}

