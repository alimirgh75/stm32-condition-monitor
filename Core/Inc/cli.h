/*
 * cli.h
 *
 *  Created on: Oct 1, 2026
 *      Author: alimi
 */
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef INC_CLI_H_
#define INC_CLI_H_

#define UART_COMMAND_MAX_LENGTH    32U

typedef enum{
	COMMAND_NOT_COMPLETE =0,
	COMMAND_IS_AVAILABLE,
	COMMAND_IS_DISCARDED,
	COMMAND_INVALID_ARGUMENT,


	COMMAND_ASSEMBLER_STATUS_COUNT

} command_assembler_status_t;

typedef struct
{
	bool discard_mode;
	uint32_t command_length;
	char command[UART_COMMAND_MAX_LENGTH + 1U];

} command_assembler_state_t;

command_assembler_status_t command_byte_processing(command_assembler_state_t *assembler_state, uint8_t recieved_byte);

void command_assembler_init(command_assembler_state_t *assembler_state);


#endif /* INC_CLI_H_ */
