#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include "picnic_main.h"
#include "picnic_dev.h"
#include "picnic_buffer.h"
#include "picnic_const.h"

// Module metadata
MODULE_AUTHOR("Denis Pavlov");
MODULE_DESCRIPTION("PiCNC controller driver");
MODULE_LICENSE("GPL");

#define DATA_WIDTH 16

picnic_t *picnc;
extern picnic_dev_t *picnic_dev;
extern picnic_buffer_t picnic_buffer;

static int picnic_write_data(__u16 data);
static int picnic_read_data(__u16 *data);

static int picnic_data_bus_set_output(void);
static int picnic_data_bus_set_input(void);

struct mutex buffer_send_mutex;
struct mutex reg_access_mutex;

static irqreturn_t picnic_buffer_empty_irq_handler(int irq, void *data) {
    if (mutex_is_locked(&buffer_send_mutex)) return IRQ_HANDLED;
    picnic_pulses_buffer_send();
    return IRQ_HANDLED;
}

int picnic_pulses_buffer_send() {
    mutex_lock(&buffer_send_mutex);

    if (!picnic_buffer_is_empty(&picnic_buffer) && picnic_is_empty()) {
	picnic_state_t st = picnic_buffer_pull(&picnic_buffer);
	if (st.update_flags & PICNIC_BUFFER_UPDATED_SERVOS) {
	    gpiod_set_value(picnc->pulse_buffer_lock, 1); // Lock buffer - we need all servos to be synchronized
	    for (__u8 i = 0; i < picnc->caps.servo_channels; i++) {
	        __u16 addr = picnc->caps.servo_channel_addrs[i];
	        if (addr != 0) {
		    picnic_write_register(addr, st.period[i]);
		    picnic_write_register(addr | 0x01, st.pulses[i]);
		}
	    }
	    gpiod_set_value(picnc->pulse_buffer_lock, 0); // Unlock buffer - fire a bunch of pulses
	}

	if (st.update_flags & PICNIC_BUFFER_UPDATED_PWMS) {
	    for (__u8 i = 0; i < picnc->caps.pwm_channels; i++) {
		picnic_write_register(picnc->caps.pwm_channel_addrs[i], st.pwm[i]);
	    }
	    st.update_flags &= ~PICNIC_BUFFER_UPDATED_PWMS;
	}

	if (st.update_flags & PICNIC_BUFFER_UPDATED_OUTPUTS) {
	    for (__u8 i = 0; i < picnc->caps.output_banks; i++) {
		picnic_write_register(picnc->caps.output_addrs[i], st.outputs[i]);
	    }
	    st.update_flags &= ~PICNIC_BUFFER_UPDATED_OUTPUTS;
	}

//	if (pulses_executed) picnic_buffer_next(&picnic_buffer);
//	else picnic_buffer_update(&picnic_buffer, st);
    }

    mutex_unlock(&buffer_send_mutex);

    return 0;
}

int picnic_probe(struct platform_device *pdev) {
    int ret = 0;
    struct device *dev = &pdev->dev;

    printk(KERN_INFO "Probing PiCNC device\n");

    picnc = devm_kzalloc(dev, sizeof(*picnc), GFP_KERNEL);
    if (!picnc) return -ENOMEM;

    picnc->data_rw = devm_gpiod_get(dev, "data-rw", GPIOD_OUT_LOW);
    if (IS_ERR(picnc->data_rw)) {
	printk(KERN_ERR "%s: Could not obtain 'data-rw' line\n", MODULE_NAME);
	return PTR_ERR(picnc->data_rw);
    }

    picnc->data_ready = devm_gpiod_get(dev, "data-ready", GPIOD_OUT_LOW);
    if (IS_ERR(picnc->data_ready)) {
	printk(KERN_ERR "%s: Could not obtain 'data-ready' line\n", MODULE_NAME);
	return PTR_ERR(picnc->data_ready);
    }

    picnc->data_addr = devm_gpiod_get(dev, "data-addr", GPIOD_OUT_LOW);
    if (IS_ERR(picnc->data_addr)) {
	printk(KERN_ERR "%s: Could not obtain 'data-addr' line\n", MODULE_NAME);
	return PTR_ERR(picnc->data_addr);
    }

    for (int i = 0; i < DATA_WIDTH; i++) {
	picnc->data[i] = devm_gpiod_get_index(dev, "data-bus", i, GPIOD_OUT_LOW);
	if (IS_ERR(picnc->data[i])) {
	    printk(KERN_ERR "%s: Could not obtain 'data-bus[%d]' line\n", MODULE_NAME, i);
	    return PTR_ERR(picnc->data[i]);
	}
    }

    // Pulse buffer lock line
    picnc->pulse_buffer_lock = devm_gpiod_get(dev, "empty-lock", GPIOD_OUT_LOW);
    if (IS_ERR(picnc->pulse_buffer_lock)) {
	printk(KERN_ERR "%s: Could not obtain 'pulse-buffer-lock' line\n", MODULE_NAME);
	return PTR_ERR(picnc->pulse_buffer_lock);
    }

    picnc->pulse_buffer_empty = devm_gpiod_get(dev, "empty", GPIOD_IN);
    if (IS_ERR(picnc->pulse_buffer_empty)) {
	printk(KERN_ERR "%s: Could not obtain 'pulse-buffer-empty' line\n", MODULE_NAME);
	return PTR_ERR(picnc->pulse_buffer_empty);
    }

    ret = gpiod_to_irq(picnc->pulse_buffer_empty);
    if (ret < 0) {
	printk(KERN_ERR "%s: Could not obtain interrupt for 'pulse-buffer-empty' signal\n", MODULE_NAME);
	return -EINVAL;
    }
    picnc->pulse_buffer_empty_irq = ret;

    // Lines allocated, set IO mode
    ret = devm_request_irq(dev, picnc->pulse_buffer_empty_irq, picnic_buffer_empty_irq_handler,
	IRQF_TRIGGER_RISING, "picnic", picnc);
    if (ret < 0) {
	printk(KERN_ERR "%s: Failed to acquire IRQ %d\n", MODULE_NAME, picnc->pulse_buffer_empty_irq);
	return -EINVAL;
    }

    // Output wires
    ret = gpiod_direction_output(picnc->data_ready, 0);
    if (ret < 0) {
	printk(KERN_ERR "%s: Can't set 'data-ready' to output mode\n", MODULE_NAME);
	return -EINVAL;
    }

    ret = gpiod_direction_output(picnc->data_rw, 0);
    if (ret < 0) {
	printk(KERN_ERR "%s: Can't set 'data-rw' to output mode\n", MODULE_NAME);
	return -EINVAL;
    }

    ret = gpiod_direction_output(picnc->data_addr, 0);
    if (ret < 0) {
	printk(KERN_ERR "%s: Can't set 'data-addr' to output mode\n", MODULE_NAME);
	return -EINVAL;
    }

    ret = gpiod_direction_output(picnc->pulse_buffer_lock, 0);
    if (ret < 0) {
	printk(KERN_ERR "%s: Can't set 'pulse-buffer-lock' to output mode\n", MODULE_NAME);
	return -EINVAL;
    }

    // Initialize device
    picnic_dev = picnic_device_init();
    if (picnic_dev == 0) {
	printk(KERN_ERR "%s: Unable to initialize device in /dev\n", MODULE_NAME);
	return ret;
    }

    // Set data lines for input
    picnic_data_bus_set_input();

    picnc->busy = 0;
    mutex_init(&reg_access_mutex);
    mutex_init(&buffer_send_mutex);

    // Read device ID
    __u16 device_id = 0;
    ret = picnic_read_register(0x00, &device_id);
    if (ret < 0) {
        printk(KERN_ERR "%s: Could not obtain PiCNC device ID: %d\n", MODULE_NAME, ret);
	return -EINVAL;
    }

    printk(KERN_INFO "%s: Got PiCNC controller ID 0x%X\n", MODULE_NAME, device_id);
    picnc->id = device_id;

    // Initialize capabilties structure
    ret = picnic_caps_init(device_id, &(picnc->caps));
    if (ret < 0) {
	printk(KERN_ERR "%s: Device ID = %4X is unknown", MODULE_NAME, device_id);
	return -EINVAL;
    }

    // Initialize send buffer
    picnc->channels = picnc->caps.servo_channels;
    picnic_buffer_init(&(picnic_buffer));

    // Write outputs set all to 'off' state
    for (int i = 0; i < picnc->caps.output_banks; i++) {
	picnic_write_register(picnc->caps.output_addrs[i], 0x0);
    }

    return 0;
}

int picnic_is_empty() {
    return gpiod_get_value(picnc->pulse_buffer_empty) & 0x01;
}

/**
 * Data output
 */
static int picnic_write_data(__u16 data) {
    for (int i = 0; i < DATA_WIDTH; i++)
	gpiod_set_value(picnc->data[i], (data >> i) & 0x01);

    // Trigger data-ready signal
    gpiod_set_value(picnc->data_ready, 0);
    gpiod_set_value(picnc->data_ready, 1);
    gpiod_set_value(picnc->data_ready, 0);

    return 0;
}

/**
 * Data input
 */
static int picnic_read_data(__u16 *data) {
    __u32 rv = 0;
    for (int i = 0; i < DATA_WIDTH; i++) {
	__u32 v = gpiod_get_value(picnc->data[i]);
	rv |= ((v & 0x01) << i);
    }

    *data = (__u16)(rv & 0xffff);

    return 0;
}

/**
 * Read register value
 */
int picnic_read_register(__u16 addr, __u16 *data) {
    mutex_lock(&reg_access_mutex);

    gpiod_set_value(picnc->data_addr, 1); // Writing address mode
    gpiod_set_value(picnc->data_rw, 1); // Data write mode to write address (sets data bus to Z mode)

    int ret = picnic_data_bus_set_output();
    if (ret != 0) {
	printk(KERN_ERR "picnic_read_register: Error setting data bus for output\n");
	return ret;
    }

    // Write address
    if (picnic_write_data(addr) != 0) {
	printk(KERN_ERR "picnic_read_register: Error writing register address\n");
	return -1;
    }

    // Set data lines to input
    if (picnic_data_bus_set_input() != 0) {
	printk(KERN_ERR "picnic_read_register: Error setting data bus for input\n");
	return -1;
    }

    // Data read mode
    gpiod_set_value(picnc->data_rw, 0);
    gpiod_set_value(picnc->data_addr, 0);

    // Read data
    __u16 rv = 0;
    ret = picnic_read_data(&rv);
    if (ret != 0) {
	printk(KERN_ERR "picnic_read_register: Error reading data bus\n");
	return ret;
    }

    gpiod_set_value(picnc->data_addr, 1); // Back to address mode

    mutex_unlock(&reg_access_mutex);

    *data = rv;

    return 0;
}

/**
 * Write register value
 */
int picnic_write_register(__u16 addr, __u16 data) {
    mutex_lock(&reg_access_mutex);

    gpiod_set_value(picnc->data_addr, 1); // Writing address mode
    gpiod_set_value(picnc->data_rw, 1); // Data write mode (sets data bus in Z)

    int ret = picnic_data_bus_set_output();
    if (ret != 0) {
	printk(KERN_ERR "picnic_write_register: Error setting data bus for output\n");
	return ret;
    }

    // Write address
    if (picnic_write_data(addr) != 0) {
	printk(KERN_ERR "picnic_write_register: Error writing register address\n");
	return -1;
    }

    gpiod_set_value(picnc->data_addr, 0);

    if (picnic_write_data(data) != 0) {
	printk(KERN_ERR "picnic_write_register: Error writing register data\n");
	return -1;
    }

    gpiod_set_value(picnc->data_addr, 1); // Back to address mode

    mutex_unlock(&reg_access_mutex);

    return 0;
}

/**
 * Sets data lines for output
 */
static int picnic_data_bus_set_output() {
    int ret = 0;
    if (picnc->data_mode != DATA_MODE_WRITE) {
	for (int i = 0; i < DATA_WIDTH; i++) {
	    ret = gpiod_direction_output(picnc->data[i], GPIOD_OUT_LOW);
	    if (ret < 0) {
		picnc->data_mode = -1; // Data mode undefined (inconsistent)
		return ret;
	    }
	}
	picnc->data_mode = DATA_MODE_WRITE;
    }
    return ret;
}

/**
 * Sets data lines for input
 */
static int picnic_data_bus_set_input() {
    int ret = 0;
    if (picnc->data_mode != DATA_MODE_READ) {
	for (int i = 0; i < DATA_WIDTH; i++) {
	    ret = gpiod_direction_input(picnc->data[i]);
	    if (ret < 0) {
		picnc->data_mode = -1; // Data mode undefined (inconsistent)
		return ret;
	    }
	}
	picnc->data_mode = DATA_MODE_READ;
    }
    return ret;
}
