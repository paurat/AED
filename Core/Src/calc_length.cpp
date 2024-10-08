// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
#include "calc_length.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>


#include "debug.h"


#define WINDOW_SIZE 40
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


// максимумы перемещений в плюс и в минус
float max_positive_move = 0.0f;
float max_negative_move = 0.0f;

// максимумы ускорений в плюс и в минус
float max_positive_acceleration = 0.0f;
float max_negative_acceleration = 0.0f;

// минуты умножить на секунды и миллисекунды
uint32_t maxmin_timeout_ms = 2 * 60 * 1000;

// текущие ускорения, скорость и перемещение
float acceleration = 0;
float speed = 0;
float length = 0;

// флаги направления движения, начала измерения и готовности окна
uint8_t is_positive_speed = 0;
uint8_t measuring_started = 0;
uint8_t window_ready = 0;

// счетчик сброса измерений
uint16_t watchdog = 0;

// окно измерений (массив и текущий элемент)
int32_t window[WINDOW_SIZE] = { 0 };
int32_t * in_window = window;
uint16_t window_value_counts = 0;

// сумма всех измерений в окне для вычисления среднего
int32_t part_sum = 0;

// время последнего обновления перемещения и ускорения
uint32_t move_timestamp = 0;
uint32_t acceleration_timestamp = 0;

// поправочный коэффициент на наклон
float incline_factor = 0;


// функция получения ускорения по рабочей оси
int32_t get_acceleration_data(lis331dlh_t * config) {

	if(config->axis_select_x_z == AXIS_X) {
		return config->accelarations.x;
	}

	if(config->axis_select_x_z == AXIS_Z) {
		return config->accelarations.z;
	}

	debug("ERROR: wrong axis selected!");
	return 0;
}


// нахождение угла наклона платы относительно целевой оси
void find_degree(lis331dlh_t * config) {
	int32_t summ = 0;
	float vector = 0;
	int count = 50;

	uint32_t timer = HAL_GetTick();
	for (int i = 0; i < count;) {
		lis331dlh_read_status(config);
		if (is_measurement_ready(config)) {					// если данные готовы
			lis331dlh_update_accelaration(config);			// обновление данных
			summ += get_acceleration_data(config);			// вытаскиваем ускорение по конкретной оси
			vector += sqrtf(config->accelarations.x * config->accelarations.x
						+ config->accelarations.y * config->accelarations.y
						+ config->accelarations.z * config->accelarations.z);	// вычисление длины вектора
			++i;
		}

		uint32_t time_passed = HAL_GetTick() - timer;
		if (time_passed > 1000) {
			debug("LIS331 timeout error");
			return;
		}  // выход, если датчик не отвечает
	}

	float sinus = fabs(summ) / vector;

	if (sinus > 0.001) {
		float degree = asinf(sinus) * 180 / M_PI;	// нахождение арксинуса и преобразование из радиан в градусы
		float mean_axis = fabs(summ) / (float)count;
		float mean_vector = vector / (float)count;
		incline_factor = 1 + mean_axis / mean_vector;	// вычисление поправочного коэффициента на угол наклона

		debug("Mean = %f\r\n", mean_axis);
		debug("Full = %f\r\n", mean_vector);
		debug("degree = %f\r\n", degree);
		debug("incline factor = %f\r\n", incline_factor);
	}
	else {
		debug("Incline factor > 2. Error!");
		incline_factor = 2;
	}

}

// проверка, что данные ускорений не были перезаписаны
void check_overrun(lis331dlh_t * config) {
	if ((config->status_register & 0xF0) > 0) {
//		HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_SET);
		debug("Acceleration data overrun! SR %02x\r\n", config->status_register);
//		HAL_UART_Transmit_IT(debug_uart, (uint8_t*)"XYZ Overrun\r\n", 13);
//		HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_RESET);
	}
}

// проверка, что данные готовы для считывания
uint8_t is_measurement_ready(lis331dlh_t * config) {
	return config->status_register & 0x08;
}

// сохранение данных в "окне" за последние 400 мс.
// Важно учитывать, что запись данных круговая
void save_in_window(float value) {
	size_t window_position = (in_window - window);
	if (window_position == WINDOW_SIZE) {
		in_window = window;
		window_ready = 1;
	}

	if (!window_ready) {
		++window_value_counts;
	}

	part_sum -= *in_window;
	*in_window++ = value;
	part_sum += value;
}

float get_mean_value_from_window() {
	return part_sum / (float)window_value_counts;
}

// проверка, что пора остановить измерения
// есть два главных условия для этого:
// 		измерение происходит дольше 400 мс (это означает, что платформа уже остановилась)
//		изменение знака скорости (это означает, что платформа прошла точку экстремума)
uint8_t is_need_stop_measuring() {
	return measuring_started && (watchdog == WINDOW_SIZE || is_positive_speed ^ (speed > 0));
}

// преобразование длины из "попугаев" в мм
// 		9.8 - ускорение 1g
//		1000 - преобразование единиц измерений
//		1024 - соответствует 1g при использовании диапазона 2G
float get_real_length() {
	return length * 9.8 / (float)1000 / (float)1024;
}

// проверка данных на корректность. условия два:
// 		перемещения меньше 1 считаем погрешностью
// 		должно быть готово "окно" для вычисления сдивига
uint8_t is_measuring_meaningful(float real_length) {
	return window_ready && (fabs(real_length) > 1.001f);
}

// сброс переменных
void reset_variables() {
	if (watchdog == WINDOW_SIZE) {
		speed = 0;
	}

	measuring_started = 0;
	length = 0;
	watchdog = 0;
	is_positive_speed = fabs(speed) > 0;
}

// проверяем направление движения
void check_positive_speed() {
	if (fabs(speed) < 0.001) {
		is_positive_speed = acceleration > 0.0f;
		speed = 0.0f;
	}
	else {
		is_positive_speed = speed > 0.0f;
	}
}

// интегрирование ускорения и скорости
void update_speed_and_length(lis331dlh_t * config) {
	if (measuring_started) {
		speed += acceleration * config->range_factor;
		length += speed * config->range_factor;
		++watchdog;
	}
}

// дебажная печать результатов вычислений
void print_result(float real_length) {
	if (is_measuring_meaningful(real_length)) {
		debug("LENGTH = %f, speed = %f, watchdog = %lu, max = %.2f, min = %.2f\r\n",
			real_length, speed, watchdog, max_positive_move, max_negative_move);
	}
}

void find_maxmin_accelerations() {
	float real_acceleration = acceleration * 9.8 / (float)1024;
	if (max_positive_acceleration < real_acceleration) {
		max_positive_acceleration = real_acceleration;
		acceleration_timestamp = HAL_GetTick();
	}

	if (max_negative_acceleration > real_acceleration) {
		max_negative_acceleration = real_acceleration;
		acceleration_timestamp = HAL_GetTick();
	}

	max_positive_acceleration = fmax(max_positive_acceleration, real_acceleration);
	max_negative_acceleration = fmin(max_negative_acceleration, real_acceleration);

}

// основной алгоритм вычисления длины перемещений
void calc_length(lis331dlh_t * config) {
//	debug("Get data\r\n");
	// получение данных ускорений
	acceleration = get_acceleration_data(config);

	// сохранение значения в окне на 40мс для вычисления среднего для компенсации
	// репроекции осей при движении
	save_in_window(acceleration);

	// вычитание среднего значения за последние 400 мс
	// необходимо для исключения ускорение свободного падения
	// и компенсации ошибки репроекции осей
	acceleration -= get_mean_value_from_window();
	// компенсация угла наклона
	acceleration *= incline_factor;

	// нахождение максимумов и минимумов ускорений
	find_maxmin_accelerations();

	// старт измерений происходит при значении ускорения больше 5
	// число 5.0 подобрано эмпирическим путем и является компромиссом
	// между точностью и ложными срабатываниями
	if (!measuring_started && fabs(acceleration) > 5.0f) {
		// флаг начала измерений
		measuring_started = 1;
		// проверка направления (необходимо для остановки измерений)
		check_positive_speed();
	}


//	debug("Integrate\r\n");
	// интегрирование
	update_speed_and_length(config);

	// проверка условий остановки
	if (is_need_stop_measuring()) {
		// преобразование сырых данных в систему счисления
		float real_length = get_real_length();

		// нахождение максимумов и минимумов по перемещениям
		if (is_measuring_meaningful(real_length)) {
			if (max_positive_move < real_length) {
				max_positive_move = real_length;
				move_timestamp = HAL_GetTick();
			}

			if (max_negative_move > real_length) {
				max_negative_move = real_length;
				move_timestamp = HAL_GetTick();
			}
		}
		print_result(real_length);
		reset_variables();
	}
//	debug("Return\r\n");
}

// сброс максимумом ускорений
void reset_acceleration_maximums() {
	if ((HAL_GetTick() - acceleration_timestamp) > maxmin_timeout_ms) {
		acceleration_timestamp = HAL_GetTick();

		max_positive_acceleration = 0.0f;
		max_negative_acceleration = 0.0f;
	}
}

// сброс максимумом по перемещениям
void reset_move_maximums() {
	if ((HAL_GetTick() - move_timestamp) > maxmin_timeout_ms) {
		move_timestamp = HAL_GetTick();

		max_positive_move = 0.0f;
		max_negative_move = 0.0f;
	}
}

// получить максимум ускорения в +
float get_max_positive_acceleration() {
	return max_positive_acceleration;
}


// получить максимум ускорения в -
float get_max_negative_acceleration() {
	return max_negative_acceleration;
}

// получить максимум перемещения в +
float get_max_positive_move() {
	return max_positive_move;
}

// получить максимум перемещения в -
float get_max_negative_move() {
	return max_negative_move;
}

uint8_t round_and_limit(float value, float limit) {
	if (value < 0) {
		value *= -1;
	}

	return (uint8_t)fmin(roundf(value), limit);
}

float round_and_limit_float(float value) {
	if (value < 0) {
		value *= -1;
	}

	return value;
}

