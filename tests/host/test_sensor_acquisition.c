#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sensor_acquisition.h"

static sensor_acquisition_t acquisition;
static ism330dhcx_t device;
static uint32_t interrupt_mask;
static uint32_t start_count;
static bool complete_during_start;
static bool drdy_on_unmask;
static ism330dhcx_status_t start_result;

uint32_t __get_PRIMASK(void)
{
    return interrupt_mask;
}

void __disable_irq(void)
{
    interrupt_mask = 1U;
}

void __set_PRIMASK(uint32_t mask)
{
    interrupt_mask = mask;
    if ((mask == 0U) && drdy_on_unmask)
    {
        drdy_on_unmask = false;
        sensor_acquisition_on_drdy(&acquisition, 200U);
    }
}

ism330dhcx_status_t ism330dhcx_start_sample_read_dma(
    const ism330dhcx_t *sensor, uint8_t *buffer, uint16_t length)
{
    assert(sensor == &device);
    assert(buffer == acquisition.dma_rx_buffer);
    assert(length == ISM330DHCX_SAMPLE_BYTE_COUNT);
    assert(acquisition.dma_busy);
    ++start_count;

    if ((start_result == ISM330DHCX_OK) && complete_during_start)
    {
        sensor_acquisition_on_dma_complete(&acquisition);
    }
    return start_result;
}

ism330dhcx_status_t ism330dhcx_decode_raw_sample(
    const uint8_t *buffer, uint16_t length, ism330dhcx_raw_sample_t *sample)
{
    assert(buffer == acquisition.dma_rx_buffer);
    assert(length == ISM330DHCX_SAMPLE_BYTE_COUNT);
    memset(sample, 0, sizeof(*sample));
    sample->accel.x = 123;
    return ISM330DHCX_OK;
}

static void reset_fixture(void)
{
    memset(&acquisition, 0, sizeof(acquisition));
    memset(&device, 0, sizeof(device));
    interrupt_mask = 0U;
    start_count = 0U;
    complete_during_start = false;
    drdy_on_unmask = false;
    start_result = ISM330DHCX_OK;
    sensor_acquisition_init(&acquisition, &device);
}

static void test_early_completion(void)
{
    reset_fixture();
    complete_during_start = true;
    sensor_acquisition_on_drdy(&acquisition, 100U);
    assert(sensor_acquisition_process(&acquisition) == ISM330DHCX_OK);
    assert(!acquisition.dma_busy);
    assert(acquisition.dma_complete_count == 1U);

    sensor_sample_t sample;
    assert(sensor_acquisition_get_sample(&acquisition, &sample));
    assert(sample.timestamp_us == 100U);
    assert(sample.data.accel.x == 123);
    assert(sensor_acquisition_is_quiescent(&acquisition));

    sensor_acquisition_on_drdy(&acquisition, 200U);
    assert(sensor_acquisition_process(&acquisition) == ISM330DHCX_OK);
    assert(start_count == 2U);
    puts("PASS: completion during DMA start permits another read");
}

static void test_failed_start_can_retry(void)
{
    reset_fixture();
    start_result = ISM330DHCX_ERROR;
    sensor_acquisition_on_drdy(&acquisition, 100U);
    assert(sensor_acquisition_process(&acquisition) == ISM330DHCX_ERROR);
    assert(sensor_acquisition_is_quiescent(&acquisition));
    assert(acquisition.processed_drdy_event_count == 0U);

    start_result = ISM330DHCX_OK;
    complete_during_start = true;
    assert(sensor_acquisition_process(&acquisition) == ISM330DHCX_OK);
    assert(start_count == 2U);
    assert(acquisition.raw_sample.timestamp_us == 100U);
    puts("PASS: failed DMA start retains the event for retry");
}

static void test_drdy_snapshot(void)
{
    reset_fixture();
    complete_during_start = true;
    sensor_acquisition_on_drdy(&acquisition, 100U);
    drdy_on_unmask = true;
    assert(sensor_acquisition_process(&acquisition) == ISM330DHCX_OK);
    assert(acquisition.processed_drdy_event_count == 1U);
    assert(acquisition.drdy_event_count == 2U);

    sensor_sample_t sample;
    assert(sensor_acquisition_get_sample(&acquisition, &sample));
    assert(sample.timestamp_us == 100U);
    assert(sensor_acquisition_process(&acquisition) == ISM330DHCX_OK);
    assert(sensor_acquisition_get_sample(&acquisition, &sample));
    assert(sample.timestamp_us == 200U);
    assert(acquisition.dropped_sample_count == 0U);
    puts("PASS: a later DRDY keeps its own timestamp");
}

static void test_pause_drains_in_flight_read(void)
{
    reset_fixture();
    sensor_acquisition_on_drdy(&acquisition, 100U);
    assert(sensor_acquisition_process(&acquisition) == ISM330DHCX_OK);
    sensor_acquisition_set_new_reads_enabled(&acquisition, false);
    assert(!sensor_acquisition_is_quiescent(&acquisition));

    sensor_acquisition_on_dma_complete(&acquisition);
    assert(sensor_acquisition_process(&acquisition) == ISM330DHCX_OK);
    sensor_sample_t sample;
    assert(sensor_acquisition_get_sample(&acquisition, &sample));
    assert(sensor_acquisition_is_quiescent(&acquisition));

    sensor_acquisition_on_drdy(&acquisition, 200U);
    assert(sensor_acquisition_process(&acquisition) == ISM330DHCX_OK);
    assert(start_count == 1U);
    puts("PASS: pause drains a read without starting another");
}

static void test_newest_pending_event(void)
{
    reset_fixture();
    complete_during_start = true;
    sensor_acquisition_on_drdy(&acquisition, 100U);
    sensor_acquisition_on_drdy(&acquisition, 200U);
    sensor_acquisition_on_drdy(&acquisition, 300U);
    assert(sensor_acquisition_process(&acquisition) == ISM330DHCX_OK);
    assert(acquisition.raw_sample.timestamp_us == 300U);
    assert(acquisition.dropped_sample_count == 2U);
    puts("PASS: newest pending event is consumed and older events are counted");
}

int main(void)
{
    test_early_completion();
    test_failed_start_can_retry();
    test_drdy_snapshot();
    test_pause_drains_in_flight_read();
    test_newest_pending_event();
    return 0;
}
