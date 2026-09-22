/*
 * condition_monitor.c
 *
 *  Created on: Sep 20, 2026
 *      Author: alimi
 */


#include "condition_monitor.h"

#include <stddef.h>

static bool condition_monitor_config_is_valid(
    const condition_monitor_config_t *config);
bool condition_monitor_init(
    condition_monitor_t *monitor,
    const condition_monitor_config_t *config)
{
	if((monitor == NULL) || (config == NULL))
	{
		return false;
	}
	if(!condition_monitor_config_is_valid(config)){return false;}

	monitor->current_state = CONDITION_STATE_NORMAL;
	monitor->severity_score = 0.0f;

	monitor->config = *config;

	return true;

}

bool condition_monitor_update(
    condition_monitor_t *monitor,
    const condition_monitor_input_t *input)
{
    if ((monitor == NULL) || (input == NULL))
    {
        return false;
    }

    /* Sensor faults have highest priority. */
    if (input->consecutive_sensor_errors >=
        monitor->config.sensor_fault_consecutive_error_limit)
    {
        monitor->current_state = CONDITION_STATE_SENSOR_FAULT;
        return true;
    }

    /* For now, SENSOR_FAULT is latched until reset. */
    if (monitor->current_state == CONDITION_STATE_SENSOR_FAULT)
    {
        return true;
    }

    /*
     * This function may be called when no window has completed.
     * In that case, check sensor health but do not decay or increase
     * the severity score.
     */
    if (!input->has_new_window)
    {
        return true;
    }

    if ((!isfinite(input->max_magnitude_mps2)) ||
        (input->max_magnitude_mps2 < 0.0f))
    {
        return false;
    }

    float magnitude_ratio =
        input->max_magnitude_mps2 /
        monitor->config.impact_reference_mps2;

    float contribution = magnitude_ratio - 1.0f;

    if (contribution < 0.0f)
    {
        contribution = 0.0f;
    }

    monitor->severity_score =
        (monitor->config.decay_factor * monitor->severity_score) +
        contribution;

    if (monitor->severity_score > monitor->config.maximum_score)
    {
        monitor->severity_score = monitor->config.maximum_score;
    }

    switch (monitor->current_state)
    {
        case CONDITION_STATE_NORMAL:
            if (monitor->severity_score >=
                monitor->config.alarm_enter_score)
            {
                monitor->current_state = CONDITION_STATE_ALARM;
            }
            else if (monitor->severity_score >=
                     monitor->config.warning_enter_score)
            {
                monitor->current_state = CONDITION_STATE_WARNING;
            }
            break;

        case CONDITION_STATE_WARNING:
            if (monitor->severity_score >=
                monitor->config.alarm_enter_score)
            {
                monitor->current_state = CONDITION_STATE_ALARM;
            }
            else if (monitor->severity_score <=
                     monitor->config.warning_exit_score)
            {
                monitor->current_state = CONDITION_STATE_NORMAL;
            }
            break;

        case CONDITION_STATE_ALARM:
            /* Alarm remains latched until condition_monitor_reset(). */
            if (monitor->severity_score <=
                monitor->config.alarm_exit_score)
            {
                monitor->current_state = CONDITION_STATE_WARNING;
            }
            break;

        case CONDITION_STATE_SENSOR_FAULT:
            /* Already handled above. */
            break;

        default:
            return false;
    }

    return true;
}

condition_state_t condition_monitor_get_state(
    const condition_monitor_t *monitor)
{
    if (monitor == NULL)
    {
        return CONDITION_STATE_COUNT;
    }

	return monitor->current_state;
}

float condition_monitor_get_severity_score(
    const condition_monitor_t *monitor)
{
    if (monitor == NULL)
    {
        return 0.0f;
    }

	return monitor->severity_score;
}

void condition_monitor_reset(
    condition_monitor_t *monitor)
{
    if (monitor == NULL)
    {
        return;
    }

	monitor->current_state = CONDITION_STATE_NORMAL;
	monitor->severity_score = 0.0f;
}

static bool condition_monitor_config_is_valid(
    const condition_monitor_config_t *config)
{
	if (config == NULL)
	{
	    return false;
	}

	if((config->impact_reference_mps2 <= 0.0f) ||
			(config->decay_factor < 0.0f) || (config->decay_factor >= 1.0f) ||
			(config->sensor_fault_consecutive_error_limit == 0U) ||
			(config->warning_exit_score < 0.0f) ||
			(config->warning_enter_score <= config->warning_exit_score) ||
			(config->warning_enter_score >= config->alarm_enter_score) ||
			(config->maximum_score < config->alarm_enter_score) ||
			(config->warning_enter_score >= config->alarm_exit_score) ||
			(config->alarm_enter_score <= config->alarm_exit_score)

	)
	{
		return false;
	}
	if ((!isfinite(config->impact_reference_mps2)) ||
	    (!isfinite(config->decay_factor)) ||
	    (!isfinite(config->warning_enter_score)) ||
	    (!isfinite(config->warning_exit_score)) ||
	    (!isfinite(config->alarm_enter_score)) ||
	    (!isfinite(config->maximum_score)) ||
		(!isfinite(config->alarm_exit_score)))
	{
	    return false;
	}

	return true;
}
