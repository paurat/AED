// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
#include "debug.h"

#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"

#include <math.h>

using uart_t = UART_HandleTypeDef;
using spi_t = SPI_HandleTypeDef;

#ifdef DEBUG
constexpr bool DEBUG_ENABLED = true;//
#endif

#ifndef DEBUG
constexpr bool DEBUG_ENABLED = false;
#endif

#ifdef EXTENDED_DEBUG
constexpr bool EXTENDED_DEBUG = true;
#endif

#ifndef EXTENDED_DEBUG
constexpr bool EXTENDED_DEBUG = false;
#endif

constexpr size_t MAX_DEBUG_MESSAGE_SIZE = 256;

constexpr uint8_t COMMAND_SIZE = 4;
constexpr uint8_t RECEIVE_BUFFER_SIZE = COMMAND_SIZE + 1;
//constexpr uint8_t TRANSMIT_BUFFER_SIZE = COMMAND_SIZE + 1;
constexpr uint16_t TRANSMIT_BUFFER_SIZE = 256;
constexpr uint16_t DEBUG_BUFFER_SIZE = DEBUG_ENABLED ? 8192 : 1;


static uint8_t debug_buffer[DEBUG_BUFFER_SIZE] = { 0, };
uint8_t receive_buffer[RECEIVE_BUFFER_SIZE] = { 0, };
uint8_t transmit_buffer[TRANSMIT_BUFFER_SIZE] = { 0, };

uint8_t * current_debug_buffer_pointer = debug_buffer;
uint8_t * last_trasmited_debug_buffer = current_debug_buffer_pointer;


bool need_logging = true;


void reset_debug_variables() {
	if (current_debug_buffer_pointer == last_trasmited_debug_buffer) {
		need_logging = true;
		memset(debug_buffer, 0, DEBUG_BUFFER_SIZE);
		current_debug_buffer_pointer = debug_buffer;
	}
	else {
		transmit(
			debug_uart, last_trasmited_debug_buffer,
			get_debug_buffer_length_to_send()
		);
	}
	last_trasmited_debug_buffer = current_debug_buffer_pointer;
}


inline size_t get_free_debug_buffer_size() {
	int32_t size = (int32_t)debug_buffer + (int32_t)DEBUG_BUFFER_SIZE - (int32_t)current_debug_buffer_pointer;

	if (size < 0) {
		size = 0;
	}

	return static_cast<size_t>(size);
}

inline uint16_t get_debug_buffer_length_to_send() {
//	size_t size = current_debug_buffer_pointer - last_trasmited_debug_buffer;
//	size = std::min(size, MAX_DEBUG_MESSAGE_SIZE);
	return current_debug_buffer_pointer - last_trasmited_debug_buffer;
}

uint8_t debug_enabled() {
	return DEBUG_ENABLED;
}

void debug(const char * message, ...) {
	if constexpr (DEBUG_ENABLED) {
		size_t max_buffer_size = get_free_debug_buffer_size();

		if (max_buffer_size > 0) {
			int16_t recorded = snprintf((char *)current_debug_buffer_pointer, max_buffer_size, "%lu ms: ", HAL_GetTick());

			uint16_t length = recorded > 0 ? (uint16_t)recorded : 0;
//			uint16_t length = 0;

			if (max_buffer_size > length) {
				max_buffer_size -= length;

				va_list args;
				va_start (args, message);
				recorded = vsnprintf((char *)(current_debug_buffer_pointer + length), max_buffer_size, message, args);
				va_end (args);

				if (recorded > 0) {
					length += recorded < (int16_t)max_buffer_size ? recorded : max_buffer_size;
				}

				current_debug_buffer_pointer += length;

				if (current_debug_buffer_pointer > (debug_buffer + 8192)) {
					current_debug_buffer_pointer = debug_buffer + 8192;
				}

			}
		}
		else {
			reset_debug_variables();
		}
	}
}

void extended_debug(const char * message, ...) {
	if constexpr (EXTENDED_DEBUG) {
		va_list args;
		va_start (args, message);
		debug(message, args);
		va_end (args);
	}
}

//void debug_str_num(char* str, uint8_t len, uint8_t num) {
//	char res[10] = {0};
//	itoa(num, res, 10);
//
//	HAL_UART_Transmit(debug_uart, (const uint8_t*)str, len, 100);
//	HAL_UART_Transmit(debug_uart, (const uint8_t*)res, 10, 100);
//}

void receive(uart_t * uart, uint8_t * buffer, size_t size) {
	HAL_UART_AbortReceive(uart);

	HAL_UART_Receive_IT(uart, buffer, size);
}

void transmit(uart_t * uart, uint8_t * buffer, uint16_t size) {
	HAL_UART_Transmit_IT(uart, buffer, size);
}

void send_debug_messages() {
	uint16_t size = get_debug_buffer_length_to_send();

	if (need_logging && size > 0) {
		transmit(debug_uart, debug_buffer, size);
		need_logging = false;
		last_trasmited_debug_buffer += size;
	}
}

void send_all_debug_buffer_blocking() {
	size_t size = current_debug_buffer_pointer - last_trasmited_debug_buffer;
	HAL_UART_Transmit(debug_uart, debug_buffer, size, 100);
}
