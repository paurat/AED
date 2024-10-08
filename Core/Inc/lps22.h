/*
 * lps22.h
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifndef INC_LPS22_H_
#define INC_LPS22_H_

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"
#include "debug.h"
#include "sensors_state.h"

#define LPS22HB_BIT(x) ((uint8_t)x)

#define LPS_LOWER_ADDRESS 0xB8
#define LPS_HIGHER_ADDRESS 0xBA


#define LPS22HB_WHO_AM_I_REG         (uint8_t)0x0F
#define LPS22HB_WHO_AM_I_VAL         (uint8_t)0xB1

#define LPS22HB_REF_P_XL_REG         (uint8_t)0x15

#define LPS22HB_REF_P_L_REG          (uint8_t)0x16

#define LPS22HB_REF_P_H_REG          (uint8_t)0x17

#define LPS22HB_RES_CONF_REG     (uint8_t)0x1A
#define LPS22HB_LCEN_MASK        (uint8_t)0x01

#define LPS22HB_CTRL_REG1      (uint8_t)0x10

#define LPS22HB_ODR_MASK                (uint8_t)0x70
#define LPS22HB_LPFP_MASK               (uint8_t)0x08
#define LPS22HB_LPFP_CUTOFF_MASK        (uint8_t)0x04
#define LPS22HB_BDU_MASK                (uint8_t)0x02
#define LPS22HB_SIM_MASK                (uint8_t)0x01

#define LPS22HB_LPFP_BIT    LPS22HB_BIT(3)

#define LPS22HB_CTRL_REG2      (uint8_t)0x11

#define LPS22HB_BOOT_BIT       LPS22HB_BIT(7)
#define LPS22HB_FIFO_EN_BIT    LPS22HB_BIT(6)
#define LPS22HB_WTM_EN_BIT     LPS22HB_BIT(5)
#define LPS22HB_ADD_INC_BIT    LPS22HB_BIT(4)
#define LPS22HB_I2C_BIT        LPS22HB_BIT(3)
#define LPS22HB_SW_RESET_BIT   LPS22HB_BIT(2)

#define LPS22HB_FIFO_EN_MASK   (uint8_t)0x40
#define LPS22HB_WTM_EN_MASK    (uint8_t)0x20
#define LPS22HB_ADD_INC_MASK   (uint8_t)0x10
#define LPS22HB_I2C_MASK       (uint8_t)0x08
#define LPS22HB_ONE_SHOT_MASK  (uint8_t)0x01

#define LPS22HB_CTRL_REG3      (uint8_t)0x12

#define LPS22HB_PP_OD_BIT       LPS22HB_BIT(6)
#define LPS22HB_FIFO_FULL_BIT   LPS22HB_BIT(5)
#define LPS22HB_FIFO_FTH_BIT    LPS22HB_BIT(4)
#define LPS22HB_FIFO_OVR_BIT    LPS22HB_BIT(3)
#define LPS22HB_DRDY_BIT        LPS22HB_BIT(2)


#define LPS22HB_INT_H_L_MASK            (uint8_t)0x80
#define LPS22HB_PP_OD_MASK              (uint8_t)0x40
#define LPS22HB_FIFO_FULL_MASK          (uint8_t)0x20
#define LPS22HB_FIFO_FTH_MASK           (uint8_t)0x10
#define LPS22HB_FIFO_OVR_MASK           (uint8_t)0x08
#define LPS22HB_DRDY_MASK               (uint8_t)0x04
#define LPS22HB_INT_S12_MASK            (uint8_t)0x03

#define LPS22HB_INTERRUPT_CFG_REG  (uint8_t)0x0B

#define LPS22HB_DIFF_EN_BIT       LPS22HB_BIT(3)
#define LPS22HB_LIR_BIT           LPS22HB_BIT(2)
#define LPS22HB_PLE_BIT           LPS22HB_BIT(1)
#define LPS22HB_PHE_BIT           LPS22HB_BIT(0)

#define LPS22HB_AUTORIFP_MASK     (uint8_t)0x80
#define LPS22HB_RESET_ARP_MASK    (uint8_t)0x40
#define LPS22HB_AUTOZERO_MASK     (uint8_t)0x20
#define LPS22HB_RESET_AZ_MASK     (uint8_t)0x10
#define LPS22HB_DIFF_EN_MASK      (uint8_t)0x08
#define LPS22HB_LIR_MASK          (uint8_t)0x04
#define LPS22HB_PLE_MASK          (uint8_t)0x02
#define LPS22HB_PHE_MASK          (uint8_t)0x01

#define LPS22HB_INTERRUPT_SOURCE_REG   (uint8_t)0x25

#define LPS22HB_BOOT_STATUS_BIT        LPS22HB_BIT(7)
#define LPS22HB_IA_BIT                 LPS22HB_BIT(2)
#define LPS22HB_PL_BIT                 LPS22HB_BIT(1)
#define LPS22HB_PH_BIT                 LPS22HB_BIT(0)

#define LPS22HB_BOOT_STATUS_MASK      (uint8_t)0x80
#define LPS22HB_IA_MASK               (uint8_t)0x04
#define LPS22HB_PL_MASK               (uint8_t)0x02
#define LPS22HB_PH_MASK               (uint8_t)0x01

#define LPS22HB_STATUS_REG         (uint8_t)0x27

#define LPS22HB_TOR_BIT            LPS22HB_BIT(5)
#define LPS22HB_POR_BIT            LPS22HB_BIT(4)
#define LPS22HB_TDA_BIT            LPS22HB_BIT(1)
#define LPS22HB_PDA_BIT            LPS22HB_BIT(0)

#define LPS22HB_TOR_MASK           (uint8_t)0x20
#define LPS22HB_POR_MASK           (uint8_t)0x10
#define LPS22HB_TDA_MASK           (uint8_t)0x02
#define LPS22HB_PDA_MASK           (uint8_t)0x01

#define LPS22HB_PRESS_OUT_XL_REG        (uint8_t)0x28
#define LPS22HB_PRESS_OUT_L_REG        (uint8_t)0x29
#define LPS22HB_PRESS_OUT_H_REG        (uint8_t)0x2A
#define LPS22HB_TEMP_OUT_L_REG         (uint8_t)0x2B
#define LPS22HBH_TEMP_OUT_H_REG         (uint8_t)0x2C
#define LPS22HB_THS_P_LOW_REG           (uint8_t)0x0C
#define LPS22HB_THS_P_HIGH_REG         (uint8_t)0x0D
#define LPS22HB_CTRL_FIFO_REG          (uint8_t)0x14

#define LPS22HB_FIFO_MODE_MASK        (uint8_t)0xE0
#define LPS22HB_WTM_POINT_MASK        (uint8_t)0x1F

#define LPS22HB_STATUS_FIFO_REG        (uint8_t)0x26
#define LPS22HB_FTH_FIFO_BIT          LPS22HB_BIT(7)
#define LPS22HB_OVR_FIFO_BIT          LPS22HB_BIT(6)

#define LPS22HB_FTH_FIFO_MASK         (uint8_t)0x80
#define LPS22HB_OVR_FIFO_MASK         (uint8_t)0x40
#define LPS22HB_LEVEL_FIFO_MASK       (uint8_t)0x3F
#define LPS22HB_FIFO_EMPTY            (uint8_t)0x00
#define LPS22HB_FIFO_FULL             (uint8_t)0x20

typedef struct{
	float last_temperature;
	float last_pressure;
} LPS22HB_sensor;


bool lps22hb_init(I2C_HandleTypeDef * i2c_handler);

uint8_t lps22hb_get_status();
bool lps22hb_check_pressure_overrun(uint8_t status);
bool lps22hb_check_temperature_overrun(uint8_t status);
bool lps22hb_check_pressure_data_available(uint8_t status);
bool lps22hb_check_temperature_data_available(uint8_t status);


uint8_t lps22hb_read_id();
float lps22hb_read_pressure();
float lps22hb_read_temperature();



#ifdef __cplusplus
}
#endif

#endif /* INC_LPS22_H_ */
