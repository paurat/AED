#ifndef DEBUG_H_
#define DEBUG_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

void reset_debug_variables();

void receive(UART_HandleTypeDef * uart, uint8_t * buffer, size_t size);
void transmit(UART_HandleTypeDef * uart, uint8_t * buffer, uint16_t size);

size_t get_free_debug_buffer_size();
uint16_t get_debug_buffer_length_to_send();

uint8_t debug_enabled();

void debug(const char * message, ...);
void extended_debug(const char * message, ...);
void send_debug_messages();
//void debug_str_num(char* str, uint8_t len, uint8_t num);
void send_all_debug_buffer_blocking();

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_H_ */
