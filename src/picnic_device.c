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

#define PICNIC_MAX_SEND 128
#define PICNIC_MAX_RECV 128

uint8_t crc_table[256] = {
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

uint8_t crc8(const uint8_t bytes[], size_t length) {
    uint8_t crc = 0;
    for (int i = 0; i < length; i++) {
        uint8_t data = (uint8_t)(bytes[i] ^ crc);  /* XOR-in next input byte */
        crc = (uint8_t)(crc_table[data]); /* get current CRC value = remainder */
    }

    return crc;
}

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
    data[2] = crc8(data+1, 1);
    bytes = write(d->fd, data, 3);
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
 * This communication is synchronous, once request is sent
 * a response is supposed to be received immediately.
 */
int picnic_device_execute(const picnic_device_t *dev, state_t *state, int period_ns) {
    int i = 0, i_prev = 0, bytes = 0;
    uint8_t sb[PICNIC_MAX_SEND], rb[PICNIC_MAX_RECV];
    uint8_t servo_channels = state->config.servo_channels;
    uint8_t output_banks = (int)ceil(state->config.output_channels / 16);

    int cmd_num = 0;
    if (dev->fd == 0) return -1;

    sb[i++] = 0; // Number of commands in send initially 0

    /*
     * Update servos command
     */
    i_prev = i;
    sb[i++] = PICNIC_PROTO_CMD_WRITE_SERVOS;
    for (int n = 0; n < servo_channels; n++) {
	// Period equal to zero means there's no any single pulse to do
	if ((state->servo[n].period == 0) || (state->servo[n].pulses == 0)) {
	    sb[i++] = 0;
	    sb[i++] = 0;
	    sb[i++] = (state->servo[n].direction ? 0x80 : 0x00); // Keep direction bit
	    sb[i++] = 0;
	} else {
	    uint8_t step_hold = picnic_device_usec_to_ticks(dev, state->servo[n].step_hold);
	    uint16_t period = picnic_device_usec_to_ticks(dev, state->servo[n].period - state->servo[n].step_hold);
	    uint16_t pulses = state->servo[n].pulses | (state->servo[n].direction ? 0x8000 : 0x0000);

	    // Save period error in nanoseconds to avoid syncronization bias
	    // when pulses to count is not zero
	    if (state->servo[n].pulses != 0) {
	        state->servo[n].period_error = (period_ns / 1000.0f) - picnic_device_ticks_to_usec(dev, (period + step_hold) * state->servo[n].pulses);
	    } else {
	        state->servo[n].period_error = 0;
	    }

	    period -= 1;
	    sb[i++] = (period >> 8) & 0xff; // (MSB)
	    sb[i++] = period & 0xff;        // (LSB)
	    sb[i++] = (pulses >> 8) & 0xff;
	    sb[i++] = pulses & 0xff;
	}
    }
    sb[i++] = crc8(sb + i_prev, i - i_prev);
    cmd_num++; // Add command

    /*
     * Update outputs command
     */
    for (int bank = 0; bank < output_banks; bank++) {
	i_prev = i;
	sb[i++] = PICNIC_PROTO_CMD_WRITE_OUTPUTS;
	sb[i++] = bank; // Bank 0
	sb[i++] = (state->outputs[bank] >> 8) & 0xff; // (MSB)
	sb[i++] = state->outputs[bank] & 0xff; // (LSB)
	sb[i++] = crc8(sb + i_prev, i - i_prev);
	cmd_num++; // Add command
    }

    /*
     * Read inputs command
     */
    i_prev = i;
    sb[i++] = PICNIC_PROTO_CMD_READ_INPUTS;
    sb[i++] = 0; // Bank 0
    sb[i++] = crc8(sb + i_prev, i - i_prev);
    cmd_num++;

    /* Read current positions */
    i_prev = i;
    sb[i++] = PICNIC_PROTO_CMD_READ_POSITIONS;
    sb[i++] = crc8(sb + i_prev, i - i_prev);
    cmd_num++;

    /********************** Send data ************************/
    sb[0] = cmd_num;
    bytes = write(dev->fd, sb, i);
    if (bytes < 0) return -1;

    /*
     * Read response and parse it
     */
    bytes = read(dev->fd, rb, PICNIC_MAX_RECV);
    if (bytes < 0) return -1;

    // Error when received non-zero second byte - it's error code
    unsigned short int error = 0;
    int byte = 0;
    for (int cmd = 0; cmd < cmd_num; cmd++) {
	int cmd_code = rb[byte++];
	int status_code = rb[byte++];
	if (status_code != 0) {
	    printf("Write command %02X received error (%d bytes): %02X\n", cmd_code, bytes, status_code);
	    error = 1;
	} else {
	    switch (cmd_code) {
		case PICNIC_PROTO_CMD_WRITE_OUTPUTS:
		case PICNIC_PROTO_CMD_WRITE_SERVOS:
		    break;
		case PICNIC_PROTO_CMD_READ_INPUTS:
		    state->inputs[rb[byte++]] = ((rb[byte++] << 8) & 0xFF00) | rb[byte++];
		    break;
		case PICNIC_PROTO_CMD_READ_POSITIONS:
		    int servos = rb[byte++];
		    for (int n = 0; n < servos; n++) {
			state->positions[n] = (rb[byte++] << 24 & 0xFF000000) | (rb[byte++] << 16 & 0xFF0000) | (rb[byte++] << 8 & 0xFF00) | (rb[byte++] & 0xFF);
		    }
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

    // ********************** Clean data after send *************************
    // (TODO: check if we need to keep that data when error appears)
    for (int n = 0; n < servo_channels; n++) {
	state->servo[n].period = 0;
	state->servo[n].pulses = 0;
    }

    if (error) return -1;

    return 0;
}

int picnic_device_read_position(const picnic_device_t *dev, state_t *state) {
    int i = 0, bytes = 0;
    uint8_t sb[PICNIC_MAX_SEND], rb[PICNIC_MAX_RECV];
    int cmd_num = 0;
    if (dev->fd == 0) return -1;

    sb[i++] = 0; // Number of commands in send initially 0

    /* Read current positions */
    sb[i++] = PICNIC_PROTO_CMD_READ_POSITIONS;
    sb[i++] = crc8(sb + 1, 1);
    cmd_num++;

    /********************** Send data ************************/
    sb[0] = cmd_num;
    bytes = write(dev->fd, sb, i);
    if (bytes < 0) return -1;

    /*
     * Read response and parse it
     */
    bytes = read(dev->fd, rb, PICNIC_MAX_RECV);
    if (bytes < 0) return -1;

    unsigned short int error = 0;
    int byte = 0;
    for (int cmd = 0; cmd < cmd_num; cmd++) {
	int cmd_code = rb[byte++];
	int status_code = rb[byte++];
	if (status_code != 0) {
	    printf("Write command %02X received error (%d bytes): %02X\n", cmd_code, bytes, status_code);
	    error = 1;
	} else {
	    switch (cmd_code) {
		case PICNIC_PROTO_CMD_READ_POSITIONS:
		    int servos = rb[byte++];
		    for (int n = 0; n < servos; n++) {
			state->positions[n] = (rb[byte++] << 24 & 0xFF000000) | (rb[byte++] << 16 & 0xFF0000) | (rb[byte++] << 8 & 0xFF00) | (rb[byte++] & 0xFF);
		    }
		    break;
		default: break;
	    }
	}

	// Abort parsing response - it can't be correct anyway.
	if (error) break;
    }

    if (error) return -1;

    return 0;
}

/*
 * Reads setting value from kernel space driver
 */
static uint8_t read_setting(const picnic_device_t *dev, uint8_t setting, uint8_t *value) {
    uint8_t cmd[] = { 1, PICNIC_PROTO_CMD_READ_SETTINGS, setting, 0x00 };
    cmd[3] = crc8(cmd+1, 2);
    int bytes = write(dev->fd, cmd, 4);
    if (bytes < 0) return -1;

    uint8_t resp[PICNIC_MAX_RECV];
    bytes = read(dev->fd, resp, PICNIC_MAX_RECV);
    if (bytes < 0) return -1;

    if ((resp[0] == PICNIC_PROTO_CMD_READ_SETTINGS) && (resp[1] == 0)) {
	*value = resp[2];
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

uint8_t picnic_device_per_channel_dir_hold(const picnic_device_t *dev) {
    return 0; // Per-servo dir hold is not supported
}

uint8_t picnic_device_per_channel_step_hold(const picnic_device_t *dev) {
    return 0; // Per-servo step hold is not supported
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
