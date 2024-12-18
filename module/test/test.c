#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#include "../picnic_const.h"

char *bytes_to_str(const uint8_t *bytes, int length) {
    char *str = malloc(length * 3);
    for (int i = 0; i < length; i++) {
	sprintf(str + (i * 3), "%02X ", bytes[i]);
    }
    return str;
}

int main(void) {
    int fd;

    fd = open("/dev/picnc", O_RDWR);
    if (fd < 0) {
	fprintf(stderr, "Can't open /dev/picnc device");
	return fd;
    }

    char *str;
    char b[1024];
    int bytes = 0;

    // Read device ID
    b[0] = 1;
    b[1] = PICNIC_PROTO_CMD_READ_DEVICE_ID;
    write(fd, b, 2);
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("DEVICE_ID received (%d bytes): %s\n", bytes, str);
    free(str);

    // Read inputs
    b[0] = 1;
    b[1] = PICNIC_PROTO_CMD_READ_INPUTS;
    write(fd, b, 2);
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("INPUTS received (%d bytes): %s\n", bytes, str);
    free(str);

    // Step hold write
    b[0] = 1;
    b[1] = PICNIC_PROTO_CMD_WRITE_STEP_HOLD;
    b[2] = 0x00;
    b[3] = 13;
    write(fd, b, 4);
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("WRITE STEP_HOLD received (%d bytes): %s\n", bytes, str);
    free(str);

    b[0] = 1;
    b[1] = PICNIC_PROTO_CMD_WRITE_DIR_HOLD;
    b[2] = 0x00;
    b[3] = 13;
    write(fd, b, 4);
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("WRITE DIR_HOLD received (%d bytes): %s\n", bytes, str);
    free(str);

    uint8_t request[] = { 1, PICNIC_PROTO_CMD_READ_SETTINGS, PICNIC_PROTO_CMD_READ_SETTING_SERVO_CHANNELS };
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ number of servo channels received (%d bytes): %s\n", bytes, str);
    free(str);

    request[2] = PICNIC_PROTO_CMD_READ_SETTING_PWM_CHANNELS;
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ number of PWM channels received (%d bytes): %s\n", bytes, str);
    free(str);

    request[2] = PICNIC_PROTO_CMD_READ_SETTING_OUTPUTS;
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ number of outputs received (%d bytes): %s\n", bytes, str);
    free(str);

    request[2] = PICNIC_PROTO_CMD_READ_SETTING_INPUTS;
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ number of inputs received (%d bytes): %s\n", bytes, str);
    free(str);

    request[2] = PICNIC_PROTO_CMD_READ_SETTING_DIR_HOLD;
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ DIR HOLD received (%d bytes): %s\n", bytes, str);
    free(str);

    request[2] = PICNIC_PROTO_CMD_READ_SETTING_STEP_HOLD;
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ STEP HOLD received (%d bytes): %s\n", bytes, str);
    free(str);

    return 0;

    int n = 0;
    for (n = 0; n < 200; n++) {

	for (int i = 0; i < 10; i++) {
	    // Write outputs
	    b[0] = 1; // Number of commands in batch
	    b[1] = PICNIC_PROTO_CMD_WRITE_OUTPUTS;
	    b[2] = 0; // Bank number
	    if (i % 2) {
		b[3] = 0xAA;
		b[4] = 0xAA;
	    } else {
		b[3] = 0x55;
		b[4] = 0x55;
	    }
	    write(fd, b, 5);
	    bytes = read(fd, b, 10);
	    printf("WRITE OUTPUTS received (%d bytes): %02X%02X\n", bytes, b[0], b[1]);
	    usleep(2000);
	}

        // Write servos
	b[0] = 1;
	b[1] = PICNIC_PROTO_CMD_WRITE_SERVOS;
	b[2] = 0x04; // Number of channels

	b[3] = 0x01; // Servo 0 (MSB)
	b[4] = 0x00; // (LSB)
	b[5] = 0x80;
	b[6] = 0xF0;

	b[7] = 0x01; // Servo 1
	b[8] = 0x00;
	b[9] = 0x00;
	b[10] = 0xF0;

	b[11] = 0x00; // Servo 2
	b[12] = 0x00;
	b[13] = 0x00;
	b[14] = 0x00;

	b[15] = 0x00; // Servo 3
	b[16] = 0x00;
	b[17] = 0x00;
	b[18] = 0x00;

	write(fd, b, 19);
	bytes = read(fd, b, 10);
	printf("WRITE SERVOS received (%d bytes): %02X%02X\n", bytes, b[0], b[1]);

	b[0] = 2;
	b[1] = PICNIC_PROTO_CMD_WRITE_SERVOS;
	b[2] = 0x04; // Number of channels

	b[3] = 0x01; // Servo 0 (MSB)
	b[4] = 0x00; // (LSB)
	b[5] = 0x00;
	b[6] = 0xF0;

	b[7] = 0x00; // Servo 1
	b[8] = 0x00;
	b[9] = 0x00;
	b[10] = 0x00;

	b[11] = 0x01; // Servo 2
	b[12] = 0x00;
	b[13] = 0x00;
	b[14] = 0xF0;

	b[15] = 0x10; // Servo 3
	b[16] = 0x00;
	b[17] = 0xF0;
	b[18] = 0x00;

	b[19] = PICNIC_PROTO_CMD_WRITE_OUTPUTS;
        b[20] = 0; // Bank number 0
	b[21] = 0x11;
        b[22] = 0x11;

	write(fd, b, 23);
	bytes = read(fd, b, 10);
	printf("WRITE SERVOS received (%d bytes): %02X%02X\n", bytes, b[0], b[1]);

	usleep(20000);
    }

    b[0] = 1; // Number of commands in batch
    b[1] = PICNIC_PROTO_CMD_WRITE_OUTPUTS;
    b[2] = 0; // Bank number 0
    b[3] = 0x00;
    b[4] = 0x00;
    write(fd, b, 3);
    bytes = read(fd, b, 10);
    printf("WRITE OUTPUTS received (%d bytes): %02X%02X\n", bytes, b[0], b[1]);

    close(fd);

    return 0;
}