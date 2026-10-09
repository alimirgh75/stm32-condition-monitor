/*
 * control_command.h
 *
 *  Created on: Oct 1, 2026
 *      Author: alimi
 */

#ifndef INC_CONTROL_COMMAND_H_
#define INC_CONTROL_COMMAND_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef enum{
	CONTROL_COMMAND_HELP,
	CONTROL_COMMAND_GET_STATUS,
	CONTROL_COMMAND_GET_CONFIG,
	CONTROL_COMMAND_GET_RATE,

	CONTROL_COMMAND_SET_RATE,
	CONTROL_COMMAND_GET_IMPACT_REFERENCE,
	CONTROL_COMMAND_SET_IMPACT_REFERENCE,
	CONTROL_COMMAND_GET_ERRORS,

	CONTROL_COMMAND_GET_VERSION,
	CONTROL_COMMAND_START,
	CONTROL_COMMAND_STOP,


	CONTROL_COMMAND_UNKNOWN
}control_command_type_t;



typedef struct{
	control_command_type_t command_type;
	float rate_hz;
	float impact_reference_mps2;

}control_command_t;

typedef struct{
	control_command_t command;
	bool success;

}control_response_t;

bool command_parser(const char *command, control_command_t *control_command);


#endif /* INC_CONTROL_COMMAND_H_ */
