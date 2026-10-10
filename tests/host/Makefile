CC = gcc
CPPFLAGS += -Istubs -I../../Core/Inc
CFLAGS += -std=c11 -Wall -Wextra -Werror -g
LDLIBS += -lm
BUILD_DIR = ../../build/host
HEADERS = $(wildcard ../../Core/Inc/*.h) stubs/stm32l4xx_hal.h
TESTS = test_condition_monitor test_sensor_acquisition test_sensor_analysis_window test_cli
BINARIES = $(addprefix $(BUILD_DIR)/,$(TESTS))

.PHONY: all test
all: $(BINARIES)

test: $(BINARIES)
	@set -e; for binary in $(BINARIES); do "$$binary"; done

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/test_condition_monitor: test_condition_monitor.c ../../Core/Src/condition_monitor.c $(HEADERS) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(filter %.c,$^) $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD_DIR)/test_sensor_acquisition: test_sensor_acquisition.c ../../Core/Src/sensor_acquisition.c $(HEADERS) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(filter %.c,$^) $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD_DIR)/test_sensor_analysis_window: test_sensor_analysis_window.c ../../Core/Src/sensor_analysis_window.c $(HEADERS) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(filter %.c,$^) $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD_DIR)/test_cli: test_cli.c ../../Core/Src/cli.c ../../Core/Src/control_command.c $(HEADERS) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(filter %.c,$^) $(LDFLAGS) $(LDLIBS) -o $@
