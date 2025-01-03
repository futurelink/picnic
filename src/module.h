#ifndef _MODULE_H_
#define _MODULE_H_

#include "picnic_device.h"
#include "hal.h"

#include <float.h>
#include <stdlib.h>

#include "rtapi_math.h"
#include "module_state.h"

#define MODULE_NAME		"picnic"
#define FUNC_UPDATE_STATE	"picnic.update-state"
#define FUNC_UPDATE_FEEDBACK    "picnic.update-feedback"

/** This structure contains the runtime data for a single generator. */

/* structure members are ordered to optimize caching for makepulses,
   which runs in the fastest thread */

typedef struct {
    hal_float_t		pos_scale;          /* param: steps per position unit */
    hal_float_t		dir_hold;           /* param: DIR hold time before STEP pulse starts */
    hal_bit_t		dir_active_low;	    /* param: DIR signal is inverted */
    hal_float_t		step_hold;          /* param: STEP hold time */
    hal_bit_t		step_active_low;    /* param: STEP signal is inverted */

    hal_bit_t		*enable;            /* pin for enable stepgen */
    hal_float_t		*pos_cmd;           /* pin: position command (position units) */
    hal_float_t		*pos_fb;            /* pin: position feedback (position units) */
    hal_s32_t		*pos_fb_steps;      /* pin: position feedback (steps) */

    bool	        direction;          /* direction of movement */

    /* Old values */
    hal_float_t         prev_pos_cmd;       /* previous commanded position */
    hal_float_t         pos_error;          /* current error */
} servo_t;

typedef struct {
    hal_bit_t       *enable;
    hal_float_t     *value;                 /* pin: PWM value in system units */
    hal_float_t     scale;                  /* param: PWM scale */
} pwm_t;

/************************
 * Module main structure
 ************************/

typedef struct {
#ifdef CONNECTION_DEVICE
    picnic_device_t *device;
#endif

#ifdef CONNECTION_NETWORK
    connection_t    *network;
#endif

#ifdef CONNECTION_USART
    usart_t         *usart;
#endif

    hal_u32_t       offline_interval;   /* param: interval in seconds that identifies device as offline */
    hal_float_t     dir_hold;           /* param: DIR hold time before STEP pulse starts */
    hal_float_t     step_hold;          /* param: STEP hold time */
    hal_bit_t       *enable;
    hal_bit_t       *online;

    servo_t         *servo;         // Servos
    pwm_t           *pwms;          // PWMs
    hal_bit_t       *outputs[16];   // Digital outputs
    hal_bit_t       *inputs[16];    // Digital inputs

    state_t         *state;
} module_t;

#endif
