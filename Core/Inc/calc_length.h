#ifndef INC_CALC_LENGTH_H_
#define INC_CALC_LENGTH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "lis331dlh.h"


void check_overrun(lis331dlh_t * config);
void check_positive_speed();

uint8_t is_measurement_ready(lis331dlh_t * config);
uint8_t is_need_stop_measuring();
uint8_t is_measuring_meaningful(float real_length);

float get_real_length();

void find_degree(lis331dlh_t * config);
void save_in_window(lis331dlh_t * config);
void reset_variables();
void update_speed_and_length();
void print_result(float real_length);
void calc_length(lis331dlh_t * config);


void reset_acceleration_maximums();
void reset_move_maximums();

float get_max_positive_acceleration();
float get_max_negative_acceleration();
float get_max_positive_move();
float get_max_negative_move();

uint8_t round_and_limit(float value, float limit);
float round_and_limit_float(float value);


#ifdef __cplusplus
}
#endif

#endif /* INC_CALC_LENGTH_H_ */
