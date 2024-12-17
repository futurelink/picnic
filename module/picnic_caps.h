#ifndef _PICNIC_CAPS_H_
#define _PICNIC_CAPS_H_

#include <linux/types.h>

#define PICNIC_DEVICE_ID_REGISTER	0x00

#define PICNIC_MAX_SERVO_CHANNELS	6
#define PICNIC_MAX_PWM_CHANNELS		6
#define PICNIC_MAX_ENCODER_CHANNELS	6
#define PICNIC_MAX_OUTPUT_BANKS		4
#define PICNIC_MAX_INPUT_BANKS		4

/* Device capabilities structure */
typedef struct picnic_dev_caps_t {
    __u16 id; /* Device ID */
    __u8 servo_channels; /* Number of servo channels */
    __u8 pwm_channels; /* Number of PWM generation channels */
    __u8 encoder_channels; /* Number of encoder channels */
    __u8 input_banks; /* Number of input groups (16 pins in group) */
    __u8 output_banks; /* Numbber of output groups (16 pins in group) */

    __u16 servo_channel_addrs[PICNIC_MAX_SERVO_CHANNELS];
    __u16 pwm_channel_addrs[PICNIC_MAX_PWM_CHANNELS];
    __u16 encoder_channel_addrs[PICNIC_MAX_ENCODER_CHANNELS];
    __u16 output_addrs[PICNIC_MAX_OUTPUT_BANKS];
    __u16 input_addrs[PICNIC_MAX_INPUT_BANKS];

    __u16 dir_hold_addr;
    __u16 step_hold_addr;
} picnic_caps_t;

int picnic_caps_init(__u16 device_id, picnic_caps_t *caps);

#endif
