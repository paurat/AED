// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
#include "terminal_parser.h"


//constexpr uint8_t COMMAND_SIZE = 4;
//
//uint8_t received_command[COMMAND_SIZE];
//
//uint8_t * begin = received_command;
//uint8_t * end = received_command + COMMAND_SIZE;
//uint8_t * current = begin;
//uint8_t * for_parsing = begin;
//uint8_t empty = 1;
//
//uint8_t is_empty() {
//	return empty;
//}
//
//void write(uint8_t value) {
//	*current++ = value;
//	empty = 0;
//	if (current == end) current = begin;
//}
//
//uint8_t check(uint8_t expected) {
//	return is_empty() ? 0 : *for_parsing == expected;
//}
//
