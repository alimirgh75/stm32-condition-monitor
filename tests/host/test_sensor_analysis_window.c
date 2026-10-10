#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sensor_analysis_window.h"

static bool close_to(float actual, float expected)
{
    return fabsf(actual - expected) <= 0.0001f;
}

static void fill_window(sensor_window_t *window, bool impulse)
{
    sensor_window_init(window);
    for (uint32_t i = 0U; i < SENSOR_ANALYSIS_WINDOW_SIZE; ++i)
    {
        const sensor_physical_sample_t sample =
        {
            .data.acceleration_mps2 =
            {
                .x = (impulse && (i == SENSOR_ANALYSIS_WINDOW_SIZE - 1U))
                    ? 16.0f : 0.0f,
                .y = 0.0f,
                .z = 8.0f
            },
            .timestamp_us = (i + 1U) * 1000U
        };
        assert(sensor_window_push(window, &sample));
    }
}

static void test_constant_window(void)
{
    sensor_window_t window;
    fill_window(&window, false);
    sensor_acceleration_time_features_t features;
    memset(&features, 0xA5, sizeof(features));
    features.max_magnitude_index = SENSOR_ANALYSIS_WINDOW_SIZE;
    ism330dhcx_axes_t centered[SENSOR_ANALYSIS_WINDOW_SIZE];

    assert(sensor_window_calculate_acceleration_features(
        &window, &features, centered));
    assert(features.max_magnitude_index == 0U);
    assert(features.max_magnitude_timestamp_us == 1000U);
    assert(close_to(features.max_magnitude_mps2, 0.0f));
    assert(close_to(features.rms_mps2.x, 0.0f));
    assert(close_to(features.rms_mps2.y, 0.0f));
    assert(close_to(features.rms_mps2.z, 0.0f));
    assert(close_to(features.peak_mps2.x, 0.0f));
    assert(close_to(features.peak_mps2.y, 0.0f));
    assert(close_to(features.peak_mps2.z, 0.0f));
    for (uint32_t i = 0U; i < SENSOR_ANALYSIS_WINDOW_SIZE; ++i)
    {
        assert(close_to(centered[i].x, 0.0f));
        assert(close_to(centered[i].y, 0.0f));
        assert(close_to(centered[i].z, 0.0f));
    }
    puts("PASS: constant window initializes all features and peak metadata");
}

static void test_known_impulse(void)
{
    sensor_window_t window;
    fill_window(&window, true);
    sensor_acceleration_time_features_t features;
    memset(&features, 0xA5, sizeof(features));
    ism330dhcx_axes_t centered[SENSOR_ANALYSIS_WINDOW_SIZE];
    assert(sensor_window_calculate_acceleration_features(
        &window, &features, centered));

    const float expected_rms =
        (16.0f / SENSOR_ANALYSIS_WINDOW_SIZE) *
        sqrtf(SENSOR_ANALYSIS_WINDOW_SIZE - 1U);
    assert(close_to(features.rms_mps2.x, expected_rms));
    assert(close_to(features.rms_mps2.y, 0.0f));
    assert(close_to(features.rms_mps2.z, 0.0f));
    assert(close_to(features.peak_mps2.x, 15.875f));
    assert(close_to(features.max_magnitude_mps2, 15.875f));
    assert(features.max_magnitude_index == SENSOR_ANALYSIS_WINDOW_SIZE - 1U);
    assert(features.max_magnitude_timestamp_us == SENSOR_ANALYSIS_WINDOW_SIZE * 1000U);
    puts("PASS: known impulse produces the expected RMS, peak and timestamp");
}

int main(void)
{
    test_constant_window();
    test_known_impulse();
    return 0;
}
