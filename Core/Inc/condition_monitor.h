/*
 * condition_monitor.h
 *
 *  Created on: Sep 20, 2026
 *      Author: alimi
 */

#ifndef CONDITION_MONITOR_H
#define CONDITION_MONITOR_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

typedef enum
{
    CONDITION_STATE_NORMAL = 0,
    CONDITION_STATE_WARNING,
    CONDITION_STATE_ALARM,
    CONDITION_STATE_SENSOR_FAULT,
    CONDITION_STATE_COUNT
} condition_state_t;


typedef struct
{
    float impact_reference_mps2;
    float decay_factor;

    float warning_enter_score;
    float warning_exit_score;
    float alarm_enter_score;
    float alarm_exit_score;
    float maximum_score;

    uint32_t sensor_fault_consecutive_error_limit;
} condition_monitor_config_t;

typedef struct
{
    float max_magnitude_mps2;
    uint32_t consecutive_sensor_errors;
    bool has_new_window;

} condition_monitor_input_t;

typedef struct
{
    condition_state_t current_state;
    float severity_score;

    condition_monitor_config_t config;
} condition_monitor_t;


bool condition_monitor_init(
    condition_monitor_t *monitor,
    const condition_monitor_config_t *config);

bool condition_monitor_update(
    condition_monitor_t *monitor,
    const condition_monitor_input_t *input);

condition_state_t condition_monitor_get_state(
    const condition_monitor_t *monitor);

float condition_monitor_get_severity_score(
    const condition_monitor_t *monitor);

void condition_monitor_reset(
    condition_monitor_t *monitor);




#endif /* INC_CONDITION_MONITOR_H_ */
