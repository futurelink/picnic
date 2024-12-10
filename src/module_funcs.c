#include <stdio.h>

#include "picnic_device.h"
#include "module_state.h"
#include "module.h"

#define MAX_PWM_DUTY 0xFFFE

volatile uint8_t exec_update_state = false;

void write_output_buffer(module_t *module);
void read_input_buffer(module_t *module);

void reset_position(module_t *module);
void initialize_position(module_t *module);
void recalculate_feedback(module_t *module);

uint8_t update_servos(module_t *module, long period);
uint8_t update_pwms(module_t *module, long period);

/*
 * SEND DATA
 */
void update_state(void *arg, long period) {
    module_t *module = arg;
    state_t *state = module->state;

    if (exec_update_state) return;
    exec_update_state = 1;

    uint8_t updated = 0;
    if (*(module->enable)) {

        recalculate_feedback(module);       // Recalculate feedback from steps to units (needs FP)

        updated = update_servos(module, period);          // Update servos state
        updated = update_pwms(module, period) | updated;  // Update PWMs state

#ifdef CONNECTION_DEVICE
        if (module->device != 0) picnic_device_execute(module->device, module->state);
#endif

#if defined(CONNECTION_NETWORK) || defined(CONNECTION_USART)
        if (updated) write_output_buffer(module);   // Write state data to send buffer
        else {                                      // Request state only, because nothing changed
            state->send_buffer[0] = 0x00;
            state->send_len = 1;
        }
#endif

#ifdef CONNECTION_NETWORK
        if (module->network) network_send(module->network, state);	// Send data via network
#endif

#ifdef CONNECTION_USART
        if (module->usart) usart_send(module->usart, state);		// Send data via USART
#endif
    } else {
        reset_position(module);
    }

    exec_update_state = 0;
}

/*
 * RECEIVE DATA
 */
void update_feedback(void *arg, long period) {
    module_t *module = arg;

    // Return immediately if module is not active
    if (!*(module->enable)) return;

#ifdef CONNECTION_NETWORK
    // Receive data from network
    if (module->network) {
	if (network_recv(module->connection, module->state, false) > 0) {
	    read_input_buffer(module);
	}
    }
#endif

    // If module position feedback is device driven,
    // then we need to update commanded position
    // with feedback values on initialization.
    if (!module->state->initialized) {
	if (module->config.has_feedback) initialize_position(module);

#ifdef CONNECTION_GPIO
	gpio_command_buffer_init();
	gpio_update_step_dir_hold(module);
#endif

	module->state->initialized = true;
	*(module->online) = true;
    }
}

/**
 * Updates feedback position
 */
void inline recalculate_feedback(module_t *module) {
    servo_t *servo = module->servo;
    for (int n = 0; n < module->config.servo_channels; n++) {
        *(servo->pos_fb) = *(servo->pos_fb_steps) / servo->pos_scale;
        servo++;
    }
}

uint8_t inline update_pwms(module_t *module, long period) {
    pwm_t *pwm = module->pwms;

    uint8_t updated = 0;
    for (int n = 0; n < module->config.pwm_channels; n++) {
        // Calculate PWM duty
        hal_u32_t duty =  (hal_u32_t)(fabs(*pwm->value) * pwm->scale);

        // Limit PWM
        if (duty > MAX_PWM_DUTY) { duty = MAX_PWM_DUTY; }
        if (module->state->pwm[n].duty != duty) {
            module->state->pwm[n].duty = duty;
            updated = 1;
        }

        pwm++;
    }

    return updated;
}


/**
 * Updates servos state
 */
uint8_t inline update_servos(module_t *module, long period_ns) {
    servo_t *servo = module->servo;

    state_t *state = module->state;
    servo_state_t *servo_state = state->servo;

    if (!state->initialized) {
        servo_state->period = 0;
        servo_state->pulses = 0;
	servo_state->period_error = 0;
        return 0;
    }

#if defined(CONNECTION_NETWORK) || defined(CONNECTION_USART)
    // If fifo is full we should skip this iteration and don't update any positions.
    // FIFO never becoming full in normal controller operation. When FIFO get's full
    // multiple messages are being shown in console (or log), so user needs to fine
    // tune HAL parameters to match controller/network abilities.
    if (state->fifo_full) {
        for (int n = 0; n < module->config.servo_channels; n++) {
            servo_state->period = 0;
            servo_state->pulses = 0;
            servo_state->period_error = 0;
            servo_state++;
        }
        return 0;
    }
#endif

    uint8_t updated = 0;
    for (int n = 0; n < module->config.servo_channels; n++) {
        // Calculate delta include previous error value
        real_t pos_cmd = *(servo->pos_cmd);
        real_t pos_cmd_delta = pos_cmd - servo->prev_pos_cmd + servo->pos_error;
        int16_t pos_cmd_delta_steps = floor(pos_cmd_delta * servo->pos_scale);
        if (pos_cmd_delta_steps == 0) { // Nothing changed so no state change needed
            servo_state->period = 0;
            servo_state->pulses = 0;
            servo_state->period_error = 0;
        } else {
            bool direction = pos_cmd_delta_steps > 0 ? true : false;

            // Here dir_hold interval is applied to pulse series, this
            // is allowed as usually direction change starts from a few
            // first pulses of accelerated movement.
            if (servo_state->direction != direction) { // If direction changes then subtract DIR hold time
                servo_state->period -= module->servo->dir_hold / servo_state->pulses;
                servo_state->direction = direction;
            }

            // Save new state change: pulse period in microseconds minus step hold time in usecs
            // (thread period in nanoseconds / steps count / 1000 = step period in microseconds)
            float tPeriod = (float)(period_ns + servo_state->period_error * 1000.0f) / (1000.0f * labs(pos_cmd_delta_steps));
            //if (servo_state->period_error != 0) printf("servo %d period error %f\n", n, servo_state->period_error);

            if (tPeriod <= servo->step_hold) {
                // If this happens after joint following error then FERROR and MIN_FERROR
                // should be increased, because of network latency between commanded and feedback
                // position.
                printf("------------------ Pulse generator error --------------------\n");
                printf("Pulse period %fus is too low on axis %d in order to complete %d steps\n", n, tPeriod, pos_cmd_delta_steps);
                printf("pos_cmd = %f, prev_pos_cmd = %f, error = %f\n", *(servo->pos_cmd), servo->prev_pos_cmd, servo->pos_error);
                printf("pos_fb = %f, pos_fb_steps = %d, period_error = %d\n", *(servo->pos_fb), *(servo->pos_fb_steps), servo_state->period_error);
                printf("Step generator can't go that fast. In order to solve this issue you can:\n");
                printf("  1) decrease step hold time (currently %fus)\n", servo->step_hold);
                printf("  2) decrease steps/mm (currently %f)\n", servo->pos_scale);
                printf("-------------------------------------------------------------\n");

                servo_state->pulses = 0;
                servo_state->period = 0;
                servo_state->period_error = 0;
            } else {
                servo_state->period = tPeriod - servo->step_hold; // Subtract STEP hold time in microseconds
                servo_state->pulses = labs(pos_cmd_delta_steps);

                // Assume feedback (steps / scale), although real feedback can be received from controller device.
                if (!module->config.has_feedback) *(servo->pos_fb_steps) += pos_cmd_delta_steps;

                // Calculate new error value, (delta in units - delta in steps / scale)
                // Generally the error is length in units which can't be moved with one step as
                // it's too small. We mustn't neglect this error, therefore it will be added
                // to distance in next iteration.
                servo->pos_error = pos_cmd_delta - pos_cmd_delta_steps / servo->pos_scale;

                // Save previous value
                servo->prev_pos_cmd = pos_cmd;

                // Set updated flag
                updated = 1;
            }
        }

        servo++;
        servo_state++;
    }

    return updated;
}

/**
 * Initializes position when machine is turned on or connected
 */
void initialize_position(module_t *module) {
    servo_t *servo = module->servo;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Initialized at positions:\n", MODULE_NAME);
    for (int n = 0; n < module->config.servo_channels; n++) {
        double pos_fb = *(servo->pos_fb_steps) / servo->pos_scale;
        servo->pos_error = 0.0;
        servo->prev_pos_cmd = pos_fb;
        *(servo->pos_fb) = pos_fb;
        *(servo->pos_cmd) = pos_fb;
        rtapi_print_msg(RTAPI_MSG_INFO, "%s: Axis %d: pos_fb_steps = %d, pos_fb = %f\n", MODULE_NAME, n, *(servo->pos_fb_steps), pos_fb);
        servo++;
    }
}

/**
 * Resets position to 0 when machine is turned off or disconnected
 */
void reset_position(module_t *module) {
    servo_t *servo = module->servo;

    if (module->state->initialized) {

        *(module->online) = false;
        module->state->initialized = false;

        rtapi_print_msg(RTAPI_MSG_INFO, "%s: Shut down at positions:\n", MODULE_NAME);
        for (int n = 0; n < module->config.servo_channels; n++) {
            rtapi_print_msg(RTAPI_MSG_INFO, "%s: Axis %d: pos_fb_steps = %d, pos_fb = %f\n", MODULE_NAME, n, *(servo->pos_fb_steps), *(servo->pos_fb));
            servo->pos_error = 0.0;
            servo->prev_pos_cmd = 0.0;
            *(servo->pos_cmd) = 0.0;
            *(servo->pos_fb) = 0.0;
            *(servo->pos_fb_steps) = 0;
            servo++;
        }
    }
}
