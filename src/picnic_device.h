#ifndef _PICNIC_DEVICE_H_
#define _PICNIC_DEVICE_H_

#include <stdint.h>

#include "module_state.h"

typedef struct picnic_device_t {
    int fd;
} picnic_device_t;

picnic_device_t *picnic_device_init(const char *dev_file);
void picnic_device_free(picnic_device_t *dev);

int picnic_get_device_id(const picnic_device_t *dev);

uint8_t picnic_device_get_servo_channels(const picnic_device_t *dev, uint8_t *value);
uint8_t picnic_device_get_pwm_channels(const picnic_device_t *dev, uint8_t *value);
uint8_t picnic_device_get_encoder_channels(const picnic_device_t *dev, uint8_t *value);
uint8_t picnic_device_get_input_channels(const picnic_device_t *dev, uint8_t *value);
uint8_t picnic_device_get_output_channels(const picnic_device_t *dev, uint8_t *value);
uint8_t picnic_device_holds_positions(const picnic_device_t *dev);
float picnic_device_channel_dir_hold(const picnic_device_t *dev);
float picnic_device_channel_step_hold(const picnic_device_t *dev);
unsigned long picnic_device_frequency(const picnic_device_t *dev);

uint16_t picnic_device_usec_to_ticks(const picnic_device_t *dev, float usec);
float picnic_device_ticks_to_usec(const picnic_device_t *dev, uint16_t ticks);

int picnic_device_execute(const picnic_device_t *dev, state_t *state, int period_ns);

#endif