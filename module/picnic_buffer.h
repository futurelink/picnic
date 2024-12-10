#ifndef _PICNIC_BUFFER_H_
#define _PICNIC_BUFFER_H_

#include "picnic_const.h"

#include <linux/types.h>

typedef struct picnic_state_t {
    __u16 pulses[PICNIC_MAX_SERVO_CHANNELS];
    __u16 period[PICNIC_MAX_SERVO_CHANNELS];
    __u16 pwm[PICNIC_MAX_PWM_CHANNELS];
    __u16 outputs;
} picnic_state_t;

typedef struct picnic_buffer_t {
    picnic_state_t data[PICNIC_BUFFER_LEN];
    volatile unsigned short int head;
    volatile unsigned short int head_next;
    volatile unsigned short int tail;
} picnic_buffer_t;

void picnic_buffer_init(picnic_buffer_t *buffer);
int picnic_buffer_is_full(picnic_buffer_t *buffer);
int picnic_buffer_is_empty(picnic_buffer_t *buffer);
int picnic_buffer_push(picnic_buffer_t *buffer, picnic_state_t st);
picnic_state_t picnic_buffer_pull(picnic_buffer_t *buffer);

#endif