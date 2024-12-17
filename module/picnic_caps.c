#include "picnic_caps.h"

#define PICNIC_DEVICES_N	1

static const picnic_caps_t device_caps[PICNIC_DEVICES_N] = {
    { // EPM570 LinuxCNC v1.0
	.id = 0x0001,
	.servo_channels = 4,
	.servo_channel_addrs = { 0x0008, 0x0010, 0x0018, 0x0020 },
	.pwm_channels = 0,
	.encoder_channels = 0,
	.input_banks = 1,
	.input_addrs = { 0x0024 },
	.output_banks = 1,
	.output_addrs = { 0x0025 },
	.dir_hold_addr = 0x0002,
	.step_hold_addr = 0x0001
    }
};

static int picnic_get_device_index(__u16 device_id) {
    for (int i = 0; i < PICNIC_DEVICES_N; i++) {
	if (device_caps[i].id == device_id) return i;
    }
    return -1;
}

int picnic_caps_init(__u16 device_id, picnic_caps_t *caps) {
    int index = picnic_get_device_index(device_id);
    if(index < 0) return index; // Error, device ID is unknown

    *caps = device_caps[index];

    return 0;
}
