#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "cli.h"
#include "control_command.h"

static command_assembler_status_t feed(
    command_assembler_state_t *assembler, const char *text)
{
    command_assembler_status_t result = COMMAND_NOT_COMPLETE;
    for (; *text != '\0'; ++text)
    {
        result = command_byte_processing(assembler, (uint8_t)*text);
    }
    return result;
}

static void test_interrupted_line(void)
{
    command_assembler_state_t assembler;
    command_assembler_init(&assembler);
    assert(feed(&assembler, "set impact-reference ") == COMMAND_NOT_COMPLETE);

    /* The interface resets the assembler and discards to CR/LF after RX loss. */
    command_assembler_init(&assembler);
    assembler.discard_mode = true;
    assert(feed(&assembler, "999\r") == COMMAND_IS_DISCARDED);
    assert(assembler.command_length == 0U);
    assert(!assembler.discard_mode);
    assert(feed(&assembler, "\n") == COMMAND_NOT_COMPLETE);

    assert(feed(&assembler, "help\r") == COMMAND_IS_AVAILABLE);
    control_command_t command;
    assert(command_parser(assembler.command, &command));
    assert(command.command_type == CONTROL_COMMAND_HELP);
    puts("PASS: interrupted command is discarded and the next line parses");
}

static void test_editing_and_limit(void)
{
    command_assembler_state_t assembler;
    command_assembler_init(&assembler);
    assert(feed(&assembler, "get statuss\b\r") == COMMAND_IS_AVAILABLE);
    assert(strcmp(assembler.command, "get status") == 0);

    command_assembler_init(&assembler);
    for (uint32_t i = 0U; i < UART_COMMAND_MAX_LENGTH; ++i)
    {
        assert(command_byte_processing(&assembler, 'a') == COMMAND_NOT_COMPLETE);
    }
    assert(command_byte_processing(&assembler, '\r') == COMMAND_IS_AVAILABLE);
    assert(strlen(assembler.command) == UART_COMMAND_MAX_LENGTH);

    command_assembler_init(&assembler);
    for (uint32_t i = 0U; i <= UART_COMMAND_MAX_LENGTH; ++i)
    {
        assert(command_byte_processing(&assembler, 'a') == COMMAND_NOT_COMPLETE);
    }
    assert(command_byte_processing(&assembler, '\r') == COMMAND_IS_DISCARDED);
    puts("PASS: backspace and the 32-character line boundary work");
}

int main(void)
{
    test_interrupted_line();
    test_editing_and_limit();
    return 0;
}
