#include <rtapi.h>      /* RTAPI realtime OS API */
#include <rtapi_app.h>  /* RTAPI realtime module decls */
#include <hal.h>

#include <stdio.h>
#include <stdlib.h>

#include "module.h"
#include "module_funcs.h"

#define BUFFER_SIZE 128

/* Module information */
MODULE_AUTHOR("Denis Pavlov");
MODULE_DESCRIPTION("PiCNC Interface board");
MODULE_LICENSE("GPL");

int             comp_id;        /* component ID */
module_t        *module = 0;

char            *device = 0;
char            *dev = 0;
char            *host = 0;
int             port = 0;

RTAPI_MP_STRING(device, "Device file to communicate");
RTAPI_MP_STRING(host, "Device IP address");
RTAPI_MP_INT(port, "Device port");
RTAPI_MP_STRING(dev, "USART device");

/* Local functions */
static int init_module_config();
static int init_state();

static int export();
static int export_module_pins();
static int export_outputs();
static int export_pwms();
static int export_servos();
static int export_servo(int num, servo_t *servo);

/**
 * RTAPI entry point
 */
int rtapi_app_main(void) {
    int ret = 0;

    /*
     * This function exports a lot of stuff, which results in a lot of
     * logging if msg_level is at INFO or ALL. So we save the current value
     * of msg_level and restore it later. If you actually need to log this
     * function's actions, change the second line below
     */
    int msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_INFO);

    /* Preliminary initialization */
    ret = init_module_config();
    if (ret != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: module config init failed: %d\n", MODULE_NAME, ret);
	hal_exit(comp_id);
	return ret;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Config initialized, proceeding..\n", MODULE_NAME);

    picnic_get_device_id(module->device);

    /* Initialize state */
    ret = init_state();
    if (ret != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: export state failed\n", MODULE_NAME);
	if (module->device != 0) picnic_device_free(module->device);
	hal_exit(comp_id);
	return ret;
    }

    /* Have good config info, connect to the HAL */
    comp_id = hal_init(MODULE_NAME);
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", MODULE_NAME);
	return comp_id;
    }

    /* Export all stuff we need in LinuxCNC */
    ret = export();
    if (ret != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: export() failed\n", MODULE_NAME);
	return ret;
    }

    ret = hal_ready(comp_id);
    if (ret != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_ready() failed\n", MODULE_NAME);
	return ret;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Ready (%d)\n", MODULE_NAME, ret);

    /* restore saved message level */
    rtapi_set_msg_level(msg);

    return 0;
}

void rtapi_app_exit(void) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Shutting down\n", MODULE_NAME);
    if (module->device != 0) picnic_device_free(module->device);
    hal_exit(comp_id);
}

/*
 * Parse and init module configuration string
 */
static int init_module_config() {

	/* Allocate module structure */
	module = hal_malloc(sizeof(module_t));
	if(module == 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: module allocation failed\n", MODULE_NAME);
	    return -1;
	}

	/* Initialize device */
	if (device != 0) {
	    module->device = picnic_device_init(device);
	    if (module->device == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: Can't establish communication with PiCNC device\n", MODULE_NAME);
		return -1;
	    }
	}

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: Device initialized\n", MODULE_NAME);

	/* Get configuration form module device ID */
	module->config.servo_channels = picnic_device_get_servo_channels(module->device);
	if (module->config.servo_channels == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: no servo channels configured\n", MODULE_NAME);
		return -1;
	}

	module->config.pwm_channels = picnic_device_get_pwm_channels(module->device);
	module->config.encoder_channels = picnic_device_get_encoder_channels(module->device);
	module->config.input_channels = picnic_device_get_input_channels(module->device);
	module->config.output_channels = picnic_device_get_output_channels(module->device);
	module->config.has_feedback = picnic_device_holds_positions(module->device);

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: Configuration initialized\n", MODULE_NAME);
	return 0;
}

/*
 * Initialize state variables
 */
static int init_state() {

    /* Allocate module state structure */
    module->state = hal_malloc(sizeof(state_t));
    if (module->state == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", MODULE_NAME);
	hal_exit(comp_id);
	return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: State allocated\n", MODULE_NAME);

    /* Allocate module servo state structure */
    if (module->config.servo_channels > 0) {
	module->state->servo = hal_malloc(module->config.servo_channels * sizeof(servo_state_t));
	if (module->state->servo == 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", MODULE_NAME);
	    hal_exit(comp_id);
	    return -1;
	}
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Servos allocated\n", MODULE_NAME);

    /* Allocate module PWM state structure */
    if (module->config.pwm_channels > 0) {
	module->state->pwm = hal_malloc(module->config.pwm_channels * sizeof(pwm_state_t));
	if (module->state->pwm == 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", MODULE_NAME);
	    hal_exit(comp_id);
	    return -1;
	}
	rtapi_print_msg(RTAPI_MSG_INFO, "%s: PWMs allocated\n", MODULE_NAME);
    }

    // Allocate buffers
#if defined (CONNECTION_NETWORK) || defined(CONNECTION_USART)
    module->state->send_len = BUFFER_SIZE;
    module->state->send_buffer = (uint8_t *)malloc(module->state->send_len);

    module->state->recv_len = BUFFER_SIZE;
    module->state->recv_buffer = (uint8_t *)malloc(module->state->recv_len);

    module->state->fifo_full = 0x00;
#endif

    module->state->initialized = false;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: State initialized\n", MODULE_NAME);

    return 0;
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static int export() {
    int ret = 0;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Exporting all\n", MODULE_NAME);

    /* export module pins */
    ret = export_module_pins();
    if (ret != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: export_module_pins() failed\n", MODULE_NAME);
	return ret;
    }

    /* export digital IO pins */
    ret = export_outputs();
    if (ret != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: export_outputs() failed\n", MODULE_NAME);
	return ret;
    }

    /* allocate shared memory for servo structures */
    if (module->config.servo_channels > 0) {
	module->servo = hal_malloc(module->config.servo_channels * sizeof(servo_t));
	if (module->servo == 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", MODULE_NAME);
	    hal_exit(comp_id);
	    return -1;
        }

	ret = export_servos();
	if (ret != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: export_servos() failed\n", MODULE_NAME);
	    return ret;
	}
    }

    /* allocate shared memory for PWMs */
    if (module->config.pwm_channels > 0) {
	module->pwms = hal_malloc(module->config.pwm_channels * sizeof(pwm_t));
	if (module->pwms == 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", MODULE_NAME);
	    hal_exit(comp_id);
	    return -1;
	}

	ret = export_pwms();
	if (ret != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: export_pwms() failed\n", MODULE_NAME);
	    return ret;
	}
    }


    /* export functions */
    if (hal_export_funct(FUNC_UPDATE_STATE, update_state, module, 1, 0, comp_id) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pos update function export failed\n", MODULE_NAME);
        hal_exit(comp_id);
        return -1;
    }

    if (hal_export_funct(FUNC_UPDATE_FEEDBACK, update_feedback, module, 0, 0, comp_id) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: state update function export failed\n", MODULE_NAME);
        hal_exit(comp_id);
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Module instantiated\n", MODULE_NAME);

    return ret;
}

static int export_module_pins() {
    int retval = 0;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Exporting pins\n", MODULE_NAME);

#if defined(CONNECTION_NETWORK) || defined(CONNECTION_USART)
    /* export module parameter: offline-interval */
    retval = hal_param_u32_newf(HAL_RW, &(module->offline_interval), comp_id, "%s.offline-interval", MODULE_NAME);
    if (retval != 0) return retval;

#endif

    /* export module parameter: dir-hold */
    retval = hal_param_float_newf(HAL_RW, &(module->dir_hold), comp_id, "%s.dir-hold", MODULE_NAME);
    if (retval != 0) return retval;

    /* export module parameter: step-hold */
    retval = hal_param_float_newf(HAL_RW, &(module->step_hold), comp_id, "%s.step-hold", MODULE_NAME);
    if (retval != 0) return retval;

    /* export module pin: online */
    retval = hal_pin_bit_newf(HAL_OUT, &(module->online), comp_id, "%s.online", MODULE_NAME);
    if (retval != 0) return retval;

    /* export module pin: enable */
    retval = hal_pin_bit_newf(HAL_IN, &(module->enable), comp_id, "%s.enable", MODULE_NAME);
    if (retval != 0) return retval;

    return retval;
}

static int export_outputs() {
    int retval = 0;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Exporting %d outputs and %d inputs\n", MODULE_NAME, module->config.output_channels, module->config.input_channels);

    /* export pins for digital outputs */
    for (int i = 0; i < module->config.output_channels; i++) {
        retval = hal_pin_bit_newf(HAL_IN, &(module->outputs[i]), comp_id, "%s.output.%d", MODULE_NAME, i);
        if (retval != 0) return retval; 
    }

    /* export pins for digital outputs */
    for (int i = 0; i < module->config.input_channels; i++) {
        retval = hal_pin_bit_newf(HAL_OUT, &(module->inputs[i]), comp_id, "%s.input.%d", MODULE_NAME, i);
        if (retval != 0) return retval; 
    }

    return retval;
}

static int export_servos() {
    int ret = 0;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Exporting %d servo channels\n", MODULE_NAME, module->config.servo_channels);

    for (int n = 0; n < module->config.servo_channels; n++) {
        /* export all servo vars */
        ret = export_servo(n, &(module->servo[n]));
        if (ret != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: servo %d export failed\n", MODULE_NAME, n);
            hal_exit(comp_id);
            return ret;
        }
    }

    return 0;
}

static int export_pwms() {
    int retval = 0;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Exporting %d PWM channels\n", MODULE_NAME, module->config.pwm_channels);

    for (int num = 0; num < module->config.pwm_channels; num++) {
	pwm_t *pwm = &(module->pwms[num]);

	/* export pin for enable command */
	retval = hal_pin_bit_newf(HAL_IN, &(pwm->enable), comp_id, "%s.pwm.%d.enable", MODULE_NAME, num);
	if (retval != 0) return retval;   

	/* export pin for PWM duty value */
	retval = hal_pin_float_newf(HAL_IN, &(pwm->value), comp_id, "%s.pwm.%d.value", MODULE_NAME, num);
	if (retval != 0) return retval;   

	/* export PWM scale param */
	retval = hal_param_float_newf(HAL_RW, &(pwm->scale), comp_id, "%s.pwm.%d.scale", MODULE_NAME, num);
	if (retval != 0) return retval;
    }

    return retval;
}

static int export_servo(int num, servo_t *servo) {

    int retval = 0;

     /* export pin for enable command */
    retval = hal_pin_bit_newf(HAL_IN, &(servo->enable), comp_id, "%s.servo.%d.enable", MODULE_NAME, num);
    if (retval != 0) return retval;

    /* export parameter for position scaling */
    retval = hal_param_float_newf(HAL_RW, &(servo->pos_scale), comp_id, "%s.servo.%d.position-scale", MODULE_NAME, num);
    if (retval != 0) return retval;

    /* export pin for command */
    retval =  hal_pin_float_newf(HAL_IN, &(servo->pos_cmd), comp_id, "%s.servo.%d.position-cmd", MODULE_NAME, num);
    if (retval != 0) return retval;

    /* export pin for scaled position */
    retval = hal_pin_float_newf(HAL_OUT, &(servo->pos_fb), comp_id, "%s.servo.%d.position-fb", MODULE_NAME, num);
    if (retval != 0) return retval;

    /* export pin for position in steps */
    retval = hal_pin_s32_newf(HAL_OUT, &(servo->pos_fb_steps), comp_id, "%s.servo.%d.position-fb-steps", MODULE_NAME, num);
    if (retval != 0) return retval;

    /* export param DIR hold interval in microseconds */
    if (picnic_device_channel_dir_hold(module->device)) {
	retval = hal_param_float_newf(HAL_RW, &(servo->dir_hold), comp_id, "%s.servo.%d.dir-hold", MODULE_NAME, num);
	if (retval != 0) return retval;
	servo->dir_hold = 2.5; // Default value
    } else {
	servo->dir_hold = 0;
    }

    /* export param DIR signal is inverted */
    retval = hal_param_bit_newf(HAL_RW, &(servo->dir_active_low), comp_id, "%s.servo.%d.dir-active-low", MODULE_NAME, num);
    if (retval != 0) return retval;

    /* export param STEP hold interval in microseconds */
    if (picnic_device_channel_step_hold(module->device)) {
	retval = hal_param_float_newf(HAL_RW, &(servo->step_hold), comp_id, "%s.servo.%d.step-hold", MODULE_NAME, num);
	if (retval != 0) return retval;
	servo->step_hold = 2.5; // Default value
    } else {
	servo->step_hold = 0;
    }

    /* export param STEP signal is inverted */
    retval = hal_param_bit_newf(HAL_RW, &(servo->step_active_low), comp_id, "%s.servo.%d.step-active-low", MODULE_NAME, num);
    if (retval != 0) return retval;

    /* Initialize values */
    servo->dir_active_low = 0;
    servo->step_active_low = 0;
    servo->pos_scale = 1.0;

    *(servo->enable) = false;
    *(servo->pos_fb) = 0.0;
    *(servo->pos_cmd) = 0.0;
    *(servo->pos_fb_steps) = 0;

    servo->prev_pos_cmd = 0.0;
    servo->pos_error = 0.0;

    return 0;
}
