#ifndef _MODULE_STATE_H_
#define _MODULE_STATE_H_

#include "hal.h"

/* ***********************
 * State structures
 *************************/

typedef struct {
    volatile bool       direction;  // Sent direction
    volatile uint16_t   pulses;     // Sent pulses count
    volatile float      period;     // Sent period
    volatile float      period_error;
} servo_state_t;

typedef struct {
    uint16_t duty;
} pwm_state_t;

typedef struct {
    volatile bool   initialized;    // Initialized flag

#if defined(CONNECTION_NETWORK) || defined(CONNECTION_USART)
    volatile uint8_t fifo_full;     // Controller FIFO status

    uint8_t         *send_buffer;   // Data send buffer
    uint16_t        send_len;       // Data buffer length

    uint8_t         *recv_buffer;
    uint8_t         recv_len;
#endif

    servo_state_t   *servo;         // Servo state
    pwm_state_t     *pwm;           // PWM state
} state_t;

#endif //_STEPGEN_STATE_H
