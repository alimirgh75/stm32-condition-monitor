/*
 * control_command.c
 *
 *  Created on: Oct 6, 2026
 *      Author: alimi
 */


#include "control_command.h"
#include "string.h"
#include <math.h>
#include <errno.h>



bool command_parser(const char *command, control_command_t *control_command)
{
	if((command == NULL) || (control_command == NULL)){return false;}

	control_command->command_type = CONTROL_COMMAND_UNKNOWN;
	control_command->impact_reference_mps2 = 0.0f;
	control_command->rate_hz = 0.0f;

	if(strcmp(command, "help") == 0)
	{
		control_command->command_type = CONTROL_COMMAND_HELP;
		return true;

	}
	if(strcmp(command, "start") == 0)
	{
		control_command->command_type = CONTROL_COMMAND_START;
		return true;

	}
	if(strcmp(command, "stop") == 0)
	{
		control_command->command_type = CONTROL_COMMAND_STOP;
		return true;

	}
	if(strcmp(command, "get status") == 0)
	{
		control_command->command_type = CONTROL_COMMAND_GET_STATUS;
		return true;

	}
	if(strcmp(command, "get config") == 0)
	{
		control_command->command_type = CONTROL_COMMAND_GET_CONFIG;
		return true;

	}
	if(strcmp(command, "get rate") == 0)
	{
		control_command->command_type = CONTROL_COMMAND_GET_RATE;
		return true;

	}

	if(strcmp(command, "get impact-reference") == 0)
	{
		control_command->command_type = CONTROL_COMMAND_GET_IMPACT_REFERENCE;
		return true;

	}

	if(strcmp(command, "get errors") == 0)
	{
		control_command->command_type = CONTROL_COMMAND_GET_ERRORS;
		return true;

	}
	if(strcmp(command, "get version") == 0)
	{
		control_command->command_type = CONTROL_COMMAND_GET_VERSION;
		return true;

	}


	if (strncmp(command, "set rate ", 9U) == 0)
	{
	    /* The argument text starts at command + 9U. */
		const char *argument = command + 9U;
		char *end;
		errno = 0;
		const float rate = strtof(argument, &end);
		if(end == argument)
		{return false;}
		if(*end != '\0'){return false;}
		if((rate <= 0.0f) || (!isfinite(rate)) || (errno == ERANGE))
		{return false;}


		control_command->command_type = CONTROL_COMMAND_SET_RATE;
		control_command->rate_hz = rate;
		return true;


	}

	if (strncmp(command, "set impact-reference ", 21U) == 0)
	{
	    /* The argument text starts at command + 21U. */
		const char *argument = command + 21U;
		char *end;
		errno = 0;
		const float impact_reference = strtof(argument, &end);
		if(end == argument)
		{return false;}
		if(*end != '\0'){return false;}
		if((impact_reference <= 0.0f) || (!isfinite(impact_reference)) || (errno == ERANGE))
		{return false;}


		control_command->command_type = CONTROL_COMMAND_SET_IMPACT_REFERENCE;
		control_command->impact_reference_mps2 = impact_reference;
		return true;


	}

	return false;


}
