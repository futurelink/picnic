#ifndef _PICNIC_BUFFER_H_
#define _PICNIC_BUFFER_H_

#include "picnic_caps.h"
#include "picnic_const.h"

#include <linux/types.h>

#define PICNIC_BUFFER_UPDATED_SERVOS   0x01
#define PICNIC_BUFFER_UPDATED_PWMS     0x02
#define PICNIC_BUFFER_UPDATED_OUTPUTS  0x04

typedef struct picnic_state_t {
    __u8 update_flags;
    __u16 pulses[PICNIC_MAX_SERVO_CHANNELS];
    __u16 period[PICNIC_MAX_SERVO_CHANNELS];
    __u16 pwm[PICNIC_MAX_PWM_CHANNELS];
    __u16 outputs[PICNIC_MAX_OUTPUT_BANKS];
} picnic_state_t;

typedef struct picnic_buffer_t {
    picnic_state_t data[PICNIC_BUFFER_LEN];
    volatile __u16 head;
    volatile __u16 head_next;
    volatile __u16 tail;
} picnic_buffer_t;

void picnic_buffer_init(picnic_buffer_t *buffer);
int picnic_buffer_is_full(picnic_buffer_t *buffer);
int picnic_buffer_is_empty(picnic_buffer_t *buffer);
int picnic_buffer_push(picnic_buffer_t *buffer, picnic_state_t st);
picnic_state_t picnic_buffer_pull(picnic_buffer_t *buffer);
picnic_state_t picnic_buffer_get(picnic_buffer_t *buffer);
void picnic_buffer_update(picnic_buffer_t *buffer, picnic_state_t st);
void picnic_buffer_next(picnic_buffer_t *buffer);

#endif