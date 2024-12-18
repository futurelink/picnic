#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "hal.h"

#include "../module/picnic_const.h"
#include "picnic_device.h"

#define PICNIC_MAX_SEND 64
#define PICNIC_MAX_RECV 32

picnic_device_t *picnic_device_init(const char *dev_file) {
    picnic_device_t *d = malloc(sizeof(picnic_device_t));

    d->fd = open(dev_file, O_RDWR | O_NONBLOCK);
    if (d->fd < 0) {
	printf("PiCNC device communication error: %d\n", d->fd);
	return 0;
    }

    printf("PiCNC device communication instantiated via %s\n", dev_file);

    return d;
}

void picnic_device_free(picnic_device_t *d) {
    if (d->fd != 0) close(d->fd);
    free(d);
}

// Read device ID
int picnic_get_device_id(const picnic_device_t *d) {
    int bytes = 0;
    char data[32];
    data[0] = 1;
    data[1] = PICNIC_PROTO_CMD_READ_DEVICE_ID;
    bytes = write(d->fd, data, 2);
    if (bytes < 0) {
	printf("PiCNC: error writing to device: %s\n", strerror(errno));
	return -1;
    }

    bytes = read(d->fd, data, 10);
    if (bytes < 0) {
	printf("PiCNC: error reading from device: %s\n", strerror(errno));
	return -1;
    } else if ((bytes != 4) || (data[1] != 0)) {
	printf("PiCNC device communication error, can't get device ID. Received %d bytes\n", bytes);
	return -1;
    }

    printf("PiCNC DEVICE_ID received (%d bytes): 0x%02X%02X\n", bytes, data[3], data[2]);

    return 0;
}

/**
 * Main procedure which sends data to PiCNC device.
 */
int picnic_device_execute(const picnic_device_t *dev, state_t *state, int period_ns) {
    int i = 0, bytes = 0;
    uint8_t b[PICNIC_MAX_SEND];
    int servo_channels = state->config.servo_channels;
    int output_banks = (int)ceil(state->config.output_channels / 16);
    int cmd_num = 0;
    if (dev->fd == 0) return -1;

    b[i++] = 0; // Number of commands in send initially 0

    /*
     * Update servos command
     */
    b[i++] = PICNIC_PROTO_CMD_WRITE_SERVOS;
    b[i++] = servo_channels; // Number of channels

    for (int n = 0; n < servo_channels; n++) {
	if ((state->servo[n].period == 0) || (state->servo[n].pulses == 0)) {
	    b[i++] = 0;
	    b[i++] = 0;
	    b[i++] = (state->servo[n].direction ? 0x80 : 0x00); // Keep direction bit
	    b[i++] = 0;
	} else {
	    uint16_t step_hold = picnic_device_usec_to_ticks(dev, picnic_device_channel_step_hold(dev)) + 1;
	    uint16_t period = picnic_device_usec_to_ticks(dev, state->servo[n].period);
	    uint16_t pulses = state->servo[n].pulses | (state->servo[n].direction ? 0x8000 : 0x0000);

	    // Save period error in nanoseconds to avoid syncronization bias
	    // when pulses to count is not zero
	    if (state->servo[n].pulses != 0) {
	        state->servo[n].period_error = (period_ns / 1000.0f) - picnic_device_ticks_to_usec(dev, period * state->servo[n].pulses);
	    } else {
	        state->servo[n].period_error = 0;
	    }

	    // Period equal to zero means there's no any single pulse to do
	    period -= step_hold;

	    b[i++] = (period >> 8) & 0xff; // (MSB)
	    b[i++] = period & 0xff;        // (LSB)
	    b[i++] = (pulses >> 8) & 0xff;
	    b[i++] = pulses & 0xff;
	}
    }
    cmd_num++; // Add command

    /*
     * Update outputs command
     */
    int bank = 0;
    b[i++] = PICNIC_PROTO_CMD_WRITE_OUTPUTS;
    b[i++] = bank; // Bank 0
    b[i++] = (state->outputs[bank] >> 8) & 0xff; // (MSB)
    b[i++] = state->outputs[bank] & 0xff; // (LSB)
    cmd_num++; // Add command

    /*
     * Read inputs command
     */
    b[i++] = PICNIC_PROTO_CMD_READ_INPUTS;
    b[i++] = 0; // Bank 0
    cmd_num++;

    /********************** Send data ************************/
    b[0] = cmd_num;
    bytes = write(dev->fd, b, i);
    if (bytes < 0) return -1;

    /*
     * Read response and parse it
     */
    bytes = read(dev->fd, b, PICNIC_MAX_RECV);
    if (bytes < 0) return -1;

    // Error when received non-zero second byte - it's error code
    unsigned short int error = 0;
    int byte = 0;
    for (int cmd = 0; cmd < cmd_num; cmd++) {
	int cmd_code = b[byte++];
	int status_code = b[byte++];
	if (status_code != 0) {
	    printf("Write command %02X received error (%d bytes): %02X\n", cmd_code, bytes, status_code);
	    error = 1;
	} else {
	    switch (cmd_code) {
		case PICNIC_PROTO_CMD_WRITE_OUTPUTS:
		case PICNIC_PROTO_CMD_WRITE_SERVOS:
		    break;
		case PICNIC_PROTO_CMD_READ_INPUTS:
		    byte += 3; // 3 bytes in response (bank number + 2 bytes of values)
		    break;
		default:
		    printf("Command %02X is unknown in response\n", cmd);
		    error = 1;
		    break;
	    }
	}

	// Abort parsing response - it can't be correct anyway.
	if (error) break;
    }

    // Clean data after send
    // (check if we need to keep that data when error appears)
    for (int n = 0; n < servo_channels; n++) {
	state->servo[n].period = 0;
	state->servo[n].pulses = 0;
    }

    if (error) return -1;

    return 0;
}

/*
 * Reads setting value from kernel space driver
 */
static uint8_t read_setting(const picnic_device_t *dev, uint8_t setting, uint8_t *value) {
    uint8_t cmd[] = { 1, PICNIC_PROTO_CMD_READ_SETTINGS, setting };
    int bytes = write(dev->fd, cmd, 3);
    if (bytes < 0) return -1;

    uint8_t resp[PICNIC_MAX_RECV];
    bytes = read(dev->fd, resp, PICNIC_MAX_RECV);
    if (bytes < 0) return -1;

    if ((resp[1] == 0) && (resp[2] == setting)) {
	*value = resp[3];
	return 0;
    }
    return resp[1];
}

/**
 * Get number of servo channels from driver.
 * Function interfaces module configuration - should not be called from realtime loop.
 */
uint8_t picnic_device_get_servo_channels(const picnic_device_t *dev, uint8_t *value) {
    return read_setting(dev, PICNIC_PROTO_CMD_READ_SETTING_SERVO_CHANNELS, value);
}

/**
 * Get number of PWM channels from driver.
 * Function interfaces module configuration - should not be called from realtime loop.
 */
uint8_t picnic_device_get_pwm_channels(const picnic_device_t *dev, uint8_t *value) {
    return read_setting(dev, PICNIC_PROTO_CMD_READ_SETTING_PWM_CHANNELS, value);
}

/**
 * Get number of encoder channels from driver.
 * Function interfaces module configuration - should not be called from realtime loop.
 */
uint8_t picnic_device_get_encoder_channels(const picnic_device_t *dev, uint8_t *value) {
    return read_setting(dev, PICNIC_PROTO_CMD_READ_SETTING_ENCODER_CHANNELS, value);
}

/**
 * Get number of input channels from driver.
 * Function interfaces module configuration - should not be called from realtime loop.
 */
uint8_t picnic_device_get_input_channels(const picnic_device_t *dev, uint8_t *value) {
    return read_setting(dev, PICNIC_PROTO_CMD_READ_SETTING_INPUTS, value);
}

/**
 * Get number of output channels from driver.
 * Function interfaces module configuration - should not be called from realtime loop.
 */
uint8_t picnic_device_get_output_channels(const picnic_device_t *dev, uint8_t *value) {
    return read_setting(dev, PICNIC_PROTO_CMD_READ_SETTING_OUTPUTS, value);
}

uint8_t picnic_device_holds_positions(const picnic_device_t *dev) {
    return 0; // Device is not capable of storing and calculating positions
}

float picnic_device_channel_dir_hold(const picnic_device_t *dev) {
    return 0; // Per-servo dir hold is not supported
}

float picnic_device_channel_step_hold(const picnic_device_t *dev) {
    return 2.5 + 0.125; // Per-servo step hold is not supported
}

unsigned long picnic_device_frequency(const picnic_device_t *dev) {
    return 5000000UL; // 5MHz
}

uint16_t picnic_device_usec_to_ticks(const picnic_device_t *dev, float usec) {
    return roundf(usec * ((float) picnic_device_frequency(dev) / 1000000.0f));
}

float picnic_device_ticks_to_usec(const picnic_device_t *dev, uint16_t ticks) {
    return ticks / ((float) picnic_device_frequency(dev) / 1000000.0f);
}
