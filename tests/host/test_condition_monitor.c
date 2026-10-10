#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "condition_monitor.h"

static condition_monitor_config_t make_valid_config(void);
static bool float_is_close(float actual, float expected);
static bool test_valid_initialization(void);
static bool test_warning_entry_and_recovery(void);

int main(void)
{
    unsigned int failed_tests = 0U;

    if (!test_valid_initialization())
    {
        printf("FAIL: valid initialization\n");
        ++failed_tests;
    }
    else
    {
        printf("PASS: valid initialization\n");
    }
    if (!test_warning_entry_and_recovery())
    {
        printf("FAIL: warning entry and recovery\n");
        ++failed_tests;
    }
    else
    {
        printf("PASS: warning entry and recovery\n");
    }

    printf("%u test(s) failed\n", failed_tests);

    return (failed_tests == 0U)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


static condition_monitor_config_t make_valid_config(void)
{
    const condition_monitor_config_t config =
    {
        .impact_reference_mps2 = 1.0f,
        .decay_factor = 0.8f,

        .warning_enter_score = 3.0f,
        .warning_exit_score = 1.0f,

        .alarm_enter_score = 10.0f,
        .alarm_exit_score = 6.0f,

        .maximum_score = 100.0f,
        .sensor_fault_consecutive_error_limit = 3U
    };

    return config;
}


static bool float_is_close(float actual, float expected)
{
    const float tolerance = 0.0001f;

    return fabsf(actual - expected) <= tolerance;
}

static bool test_valid_initialization(void)
{
    condition_monitor_t monitor;

    const condition_monitor_config_t config =
        make_valid_config();

    if (!condition_monitor_init(&monitor, &config))
    {
        return false;
    }

    if (condition_monitor_get_state(&monitor) !=
        CONDITION_STATE_NORMAL)
    {
        return false;
    }

    if (!float_is_close(
            condition_monitor_get_severity_score(&monitor),
            0.0f))
    {
        return false;
    }

    return true;
}


static bool apply_window(
    condition_monitor_t *monitor,
    float max_magnitude_mps2)
{
    const condition_monitor_input_t input =
    {
        .has_new_window = true,
        .max_magnitude_mps2 = max_magnitude_mps2,
        .consecutive_sensor_errors = 0U
    };

    return condition_monitor_update(monitor, &input);
}

static bool test_warning_entry_and_recovery(void)
{
    condition_monitor_t monitor;

    const condition_monitor_config_t config =
        make_valid_config();

    if (!condition_monitor_init(&monitor, &config))
    {
        return false;
    }

    /* 5.0 / 1.0 - 1.0 gives a contribution of 4.0. */
    if (!apply_window(&monitor, 5.0f))
    {
        return false;
    }

    if (!float_is_close(
            condition_monitor_get_severity_score(&monitor),
            4.0f))
    {
        return false;
    }

    if (condition_monitor_get_state(&monitor) !=
        CONDITION_STATE_WARNING)
    {
        return false;
    }


    for (uint32_t i = 0U; i < 6U; ++i)
    {
        if (!apply_window(&monitor, 0.0f))
        {
            return false;
        }
    }

    if (!float_is_close(
            condition_monitor_get_severity_score(&monitor),
            1.048576f))
    {
        return false;
    }

    if (condition_monitor_get_state(&monitor) !=
        CONDITION_STATE_WARNING)
    {
        return false;
    }

    /*

     * This crosses warning_exit_score, so it returns to NORMAL.
     */
    if (!apply_window(&monitor, 0.0f))
    {
        return false;
    }

    if (!float_is_close(
            condition_monitor_get_severity_score(&monitor),
            0.8388608f))
    {
        return false;
    }

    if (condition_monitor_get_state(&monitor) !=
        CONDITION_STATE_NORMAL)
    {
        return false;
    }

    return true;
}
