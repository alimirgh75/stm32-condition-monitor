/*
 * cli.c
 *
 *  Created on: Oct 1, 2026
 *      Author: alimi
 */
#include "cli.h"

void command_assembler_init(command_assembler_state_t *assembler_state)
{
	if(assembler_state == NULL)
	{return;}

	assembler_state->command_length = 0U;
	assembler_state->command[0] = '\0';
	assembler_state->discard_mode = false;
}


command_assembler_status_t command_byte_processing(command_assembler_state_t *assembler_state, uint8_t recieved_byte)
{
	if(assembler_state == NULL)
	{return COMMAND_INVALID_ARGUMENT;}
	if((recieved_byte == '\r') || (recieved_byte == '\n'))
	{
		if(assembler_state->discard_mode)
		{
			assembler_state->discard_mode = false;
			assembler_state->command_length = 0U;
			assembler_state->command[assembler_state->command_length] = '\0';

			return COMMAND_IS_DISCARDED;

		}


		if(assembler_state->command_length >0U)
		{
			assembler_state->command[assembler_state->command_length] = '\0';
			return COMMAND_IS_AVAILABLE;
		}
		if(assembler_state->command_length == 0U)
		{
			return COMMAND_NOT_COMPLETE;
		}


	}
	if (assembler_state->discard_mode)
	{
	    return COMMAND_NOT_COMPLETE;
	}

	if ((recieved_byte == 0x08U) || (recieved_byte == 0x7FU))
	{
	    if (assembler_state->command_length > 0U)
	    {
	        --assembler_state->command_length;
	        assembler_state->command[assembler_state->command_length] = '\0';
	    }

	    return COMMAND_NOT_COMPLETE;
	}

	if(assembler_state->command_length >= UART_COMMAND_MAX_LENGTH){
		assembler_state->discard_mode = true;
		assembler_state->command_length = 0U;
		assembler_state->command[assembler_state->command_length] = '\0';
		return COMMAND_NOT_COMPLETE;

	}
	assembler_state->command[assembler_state->command_length] = recieved_byte;

	(assembler_state->command_length)++;

	return COMMAND_NOT_COMPLETE;

}
