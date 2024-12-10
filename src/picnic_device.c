#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "hal.h"

#include "../driver/picnic_const.h"
#include "picnic_device.h"

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
void picnic_get_device_id(const picnic_device_t *d) {
    int bytes = 0;
    char data[32];
    data[0] = PICNIC_PROTO_CMD_READ_DEVICE_ID;
    bytes = write(d->fd, data, 1);
    if (bytes < 0) {
	printf("PiCNC: error writing to device: %s\n", strerror(errno));
	return;
    }

    bytes = read(d->fd, data, 32);
    if (bytes < 0) {
	printf("PiCNC: error reading from device: %s\n", strerror(errno));
	return;
    } else if (bytes != 2) {
	printf("PiCNC device communication error, can't get device ID\n");
	return;
    }

    printf("PiCNC DEVICE_ID received (%d bytes): 0x%02X%02X\n", bytes, data[1], data[0]);
}

int picnic_device_execute(const picnic_device_t *dev, state_t *state) {
    int period_ns = 2000000;
    int i = 0, bytes = 0;
    int channels = 4;
    uint8_t b[19];

    if (dev->fd != 0) {
	b[i++] = PICNIC_PROTO_CMD_WRITE_SERVOS;
	b[i++] = channels; // Number of channels

	for (int n = 0; n < channels; n++) {
	    uint16_t period = picnic_device_usec_to_ticks(dev, state->servo[n].period);
	    uint16_t pulses = state->servo[n].pulses | (state->servo[n].direction ? 0x8000 : 0x0000);

	    // Save period error in nanoseconds to avoid syncronization bias
	    // when pulses to count is not zero
	    if (state->servo[n].pulses != 0) {
		state->servo[n].period_error = period_ns / 1000.0f - picnic_device_ticks_to_usec(dev, period) * state->servo[n].pulses;
	    } else {
		state->servo[n].period_error = 0;
	    }

	    period = period / 2;

	    b[i++] = (period >> 8) & 0xff; // (MSB)
	    b[i++] = period & 0xff;      // (LSB)
	    b[i++] = (pulses >> 8) & 0xff;
	    b[i++] = pulses & 0xff;
	}

	bytes = write(dev->fd, b, i);
	if (bytes < 0) return -1;

        bytes = read(dev->fd, b, 10);
	if (bytes < 0) return -1;

        if ((bytes != 1) || (b[0] != 0)) {
	    printf("WRITE SERVOS received error (%d bytes): %02X%02X\n", bytes, b[0], b[1]);
	    return -1;
        }

	// Clean data after send
	for (int n = 0; n < channels; n++) {
	    state->servo[n].period = 0;
	    state->servo[n].pulses = 0;
	}
    }

    return 0;
}

uint8_t picnic_device_get_servo_channels(const picnic_device_t *dev) {
    return 4;
}

uint8_t picnic_device_get_pwm_channels(const picnic_device_t *dev) {
    return 0;
}

uint8_t picnic_device_get_encoder_channels(const picnic_device_t *dev) {
    return 0;
}

uint8_t picnic_device_get_input_channels(const picnic_device_t *dev) {
    return 16;
}

uint8_t picnic_device_get_output_channels(const picnic_device_t *dev) {
    return 16;
}

uint8_t picnic_device_holds_positions(const picnic_device_t *dev) {
    return 0; // Device is not capable of storing and calculating positions
}

uint8_t picnic_device_channel_dir_hold(const picnic_device_t *dev) {
    return 0; // Per-servo dir hold is not supported
}

uint8_t picnic_device_channel_step_hold(const picnic_device_t *dev) {
    return 0; // Per-servo step hold is not supported
}

unsigned long picnic_device_frequency(const picnic_device_t *dev) {
    return 5000000UL; // 5MHz
}

uint16_t picnic_device_usec_to_ticks(const picnic_device_t *dev, float usec) {
    return floorf(usec * ((float) picnic_device_frequency(dev) / 1000000.0f));
}

float picnic_device_ticks_to_usec(const picnic_device_t *dev, uint16_t ticks) {
    return ticks / ((float) picnic_device_frequency(dev) / 1000000.0f);
}
