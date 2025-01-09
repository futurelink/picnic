#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#include "../picnic_const.h"

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
    b[2] = crc8(b+1, 1);
    write(fd, b, 3);
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("DEVICE_ID received (%d bytes): %s\n", bytes, str);
    free(str);

    // Read inputs
    b[0] = 1;
    b[1] = PICNIC_PROTO_CMD_READ_INPUTS;
    b[2] = 0; // Bank number
    b[3] = crc8(b+1, 2);
    write(fd, b, 4);
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("INPUTS received (%d bytes): %s\n", bytes, str);
    free(str);

    // Step hold write
    b[0] = 1;
    b[1] = PICNIC_PROTO_CMD_WRITE_STEP_HOLD;
    b[2] = 0x00;
    b[3] = 13;
    b[4] = crc8(b+1, 3);
    write(fd, b, 5);
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("WRITE STEP_HOLD received (%d bytes): %s\n", bytes, str);
    free(str);

    b[0] = 1;
    b[1] = PICNIC_PROTO_CMD_WRITE_DIR_HOLD;
    b[2] = 0x00;
    b[3] = 13;
    b[4] = crc8(b+1, 3);
    write(fd, b, 5);
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("WRITE DIR_HOLD received (%d bytes): %s\n", bytes, str);
    free(str);

    uint8_t request[] = { 1, PICNIC_PROTO_CMD_READ_SETTINGS, PICNIC_PROTO_CMD_READ_SETTING_SERVO_CHANNELS, 0x00 };
    request[3] = crc8(request + 1, 2);
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ number of servo channels received (%d bytes): %s\n", bytes, str);
    free(str);

    request[2] = PICNIC_PROTO_CMD_READ_SETTING_PWM_CHANNELS;
    request[3] = crc8(&request[1], 2);
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ number of PWM channels received (%d bytes): %s\n", bytes, str);
    free(str);

    request[2] = PICNIC_PROTO_CMD_READ_SETTING_OUTPUTS;
    request[3] = crc8(&request[1], 2);
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ number of outputs received (%d bytes): %s\n", bytes, str);
    free(str);

    request[2] = PICNIC_PROTO_CMD_READ_SETTING_INPUTS;
    request[3] = crc8(&request[1], 2);
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ number of inputs received (%d bytes): %s\n", bytes, str);
    free(str);

    request[2] = PICNIC_PROTO_CMD_READ_SETTING_DIR_HOLD;
    request[3] = crc8(&request[1], 2);
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ DIR HOLD received (%d bytes): %s\n", bytes, str);
    free(str);

    request[2] = PICNIC_PROTO_CMD_READ_SETTING_STEP_HOLD;
    request[3] = crc8(&request[1], 2);
    write(fd, request, sizeof(request));
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("READ STEP HOLD received (%d bytes): %s\n", bytes, str);
    free(str);

    b[0] = 1;
    b[1] = PICNIC_PROTO_CMD_READ_POSITIONS;
    b[2] = crc8(&b[1], 1);
    write(fd, b, 3);
    bytes = read(fd, b, 32);
    str = bytes_to_str(b, bytes);
    printf("READ POSITIONS received (%d bytes): %s\n", bytes, str);
    free(str);

//    return 0;

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
	    b[5] = crc8(&b[1], 4);
	    write(fd, b, 6);
	    bytes = read(fd, b, 10);
	    str = bytes_to_str(b, bytes);
	    printf("WRITE OUTPUTS received (%d bytes): %s\n", bytes, str);
	    free(str);
	    usleep(2000);
	}

        // Write servos
	b[0] = 1;
	b[1] = PICNIC_PROTO_CMD_WRITE_SERVOS;

	b[2] = 0x01; // Servo 0 (MSB)
	b[3] = 0x00; // (LSB)
	b[4] = 0x80;
	b[5] = 0xF0;

	b[6] = 0x01; // Servo 1
	b[7] = 0x00;
	b[8] = 0x00;
	b[9] = 0xF0;

	b[10] = 0x00; // Servo 2
	b[11] = 0x00;
	b[12] = 0x00;
	b[13] = 0x00;

	b[14] = 0x00; // Servo 3
	b[15] = 0x00;
	b[16] = 0x00;
	b[17] = 0x00;
	b[18] = crc8(&b[1], 17);

	write(fd, b, 19);
	bytes = read(fd, b, 10);
	str = bytes_to_str(b, bytes);
	printf("WRITE SERVOS received (%d bytes): %s\n", bytes, str);
	free(str);

	b[0] = 2;
	b[1] = PICNIC_PROTO_CMD_WRITE_SERVOS;

	b[2] = 0x01; // Servo 0 (MSB)
	b[3] = 0x00; // (LSB)
	b[4] = 0x00;
	b[5] = 0xF0;

	b[6] = 0x00; // Servo 1
	b[7] = 0x00;
	b[8] = 0x00;
	b[9] = 0x00;

	b[10] = 0x01; // Servo 2
	b[11] = 0x00;
	b[12] = 0x00;
	b[13] = 0xF0;

	b[14] = 0x01; // Servo 3
	b[15] = 0x00;
	b[16] = 0x80;
	b[17] = 0xF0;
	b[18] = crc8(&b[1], 17);

	b[19] = PICNIC_PROTO_CMD_WRITE_OUTPUTS;
        b[20] = 0; // Bank number 0
	b[21] = 0x11;
        b[22] = 0x11;
	b[23] = crc8(&b[19], 4);

	write(fd, b, 24);
	bytes = read(fd, b, 10);
	str = bytes_to_str(b, bytes);
	printf("WRITE SERVOS received (%d bytes): %s\n", bytes, str);
	free(str);

	usleep(20000);
    }

    b[0] = 1; // Number of commands in batch
    b[1] = PICNIC_PROTO_CMD_WRITE_OUTPUTS;
    b[2] = 0; // Bank number 0
    b[3] = 0x00;
    b[4] = 0x00;
    b[5] = crc8(&b[1], 4);
    write(fd, b, 6);
    bytes = read(fd, b, 10);
    str = bytes_to_str(b, bytes);
    printf("WRITE OUTPUTS received (%d bytes): %s\n", bytes, str);
    free(str);

    close(fd);

    return 0;
}
