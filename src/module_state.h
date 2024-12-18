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

typedef struct config_t {
    uint16_t device_id;
    uint8_t has_feedback;
    uint8_t servo_channels;
    uint8_t encoder_channels;
    uint8_t pwm_channels;
    uint8_t output_channels;
    uint8_t input_channels;
} config_t;

typedef struct {
    volatile bool   initialized;    // Initialized flag
    config_t        config;         // Device configuration

#if defined(CONNECTION_NETWORK) || defined(CONNECTION_USART)
    volatile uint8_t fifo_full;     // Controller FIFO status

    uint8_t         *send_buffer;   // Data send buffer
    uint16_t        send_len;       // Data buffer length

    uint8_t         *recv_buffer;
    uint8_t         recv_len;
#endif


    uint16_t        *inputs;        // Digital inputs state
    uint16_t        *outputs;       // Digital outputs state
    servo_state_t   *servo;         // Servos state
    pwm_state_t     *pwm;           // PWMs state
} state_t;

#endif //_STEPGEN_STATE_H