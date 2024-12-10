#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/spinlock.h>

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

static int picnic_write_data(uint16_t data);
static int picnic_read_data(uint16_t *data);

static int picnic_data_bus_set_output(void);
static int picnic_data_bus_set_input(void);

spinlock_t reg_access_lock;

static uint16_t picnic_get_channel_addr(uint8_t channel) {
    switch(channel) {
	case 0: return GPIO_REG_PULSEGEN_1;
	case 1: return GPIO_REG_PULSEGEN_2;
	case 2: return GPIO_REG_PULSEGEN_3;
	case 3: return GPIO_REG_PULSEGEN_4;
	default: return 0;
    }
}

static irqreturn_t picnic_buffer_empty_irq_handler(int irq, void *data) {
//    printk(KERN_INFO "buffer empty interrupt\n");
    picnic_pulses_buffer_send();
//    printk(KERN_INFO "buffer empty interrupt end\n");
    return IRQ_HANDLED;
}

int picnic_pulses_buffer_send() {
//    disable_irq(picnc->pulse_buffer_empty_irq);
    if (picnc->busy) return 0;

    picnc->busy = 1;
    if (picnic_buffer_is_empty(&(picnic_buffer))) {
	picnc->busy = 0;
//	enable_irq(picnc->pulse_buffer_empty_irq);
	return 0;
    }

    picnic_state_t st = picnic_buffer_pull(&(picnic_buffer));
    gpiod_set_value(picnc->pulse_buffer_lock, 1);
    for (uint8_t i = 0; i < picnc->channels; i++) {
	uint16_t addr = picnic_get_channel_addr(i);
	if (addr != 0) {
	    picnic_write_register(addr, st.period[i]);
	    picnic_write_register(addr | 0x01, st.pulses[i]);
	}
	//picnic_write_register(GPIO_REG_OUTPUTS, st.outputs); // Do not write outputs at this moment
    }
    gpiod_set_value(picnc->pulse_buffer_lock, 0);
    picnc->busy = 0;
//    enable_irq(picnc->pulse_buffer_empty_irq);
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
	printk(KERN_ERR "Could not obtain 'data-rw' line\n");
	return PTR_ERR(picnc->data_rw);
    }

    picnc->data_ready = devm_gpiod_get(dev, "data-ready", GPIOD_OUT_LOW);
    if (IS_ERR(picnc->data_ready)) {
	printk(KERN_ERR "Could not obtain 'data-ready' line\n");
	return PTR_ERR(picnc->data_ready);
    }

    picnc->data_addr = devm_gpiod_get(dev, "data-addr", GPIOD_OUT_LOW);
    if (IS_ERR(picnc->data_addr)) {
	printk(KERN_ERR "Could not obtain 'data-addr' line\n");
	return PTR_ERR(picnc->data_addr);
    }

    for (int i = 0; i < DATA_WIDTH; i++) {
	picnc->data[i] = devm_gpiod_get_index(dev, "data-bus", i, GPIOD_OUT_LOW);
	if (IS_ERR(picnc->data[i])) {
	    printk(KERN_ERR "Could not obtain 'data-bus[%d]' line\n", i);
	    return PTR_ERR(picnc->data[i]);
	}
    }

    // Pulse buffer lock line
    picnc->pulse_buffer_lock = devm_gpiod_get(dev, "empty-lock", GPIOD_OUT_LOW);
    if (IS_ERR(picnc->pulse_buffer_lock)) {
	printk(KERN_ERR "Could not obtain 'empty-lock' line\n");
	return PTR_ERR(picnc->pulse_buffer_lock);
    }

    picnc->pulse_buffer_empty = devm_gpiod_get(dev, "empty", GPIOD_IN);
    if (IS_ERR(picnc->pulse_buffer_empty)) {
	printk(KERN_ERR "Could not obtain 'pulse-buffer-empty' line\n");
	return PTR_ERR(picnc->pulse_buffer_empty);
    }

    ret = gpiod_to_irq(picnc->pulse_buffer_empty);
    if (ret < 0) {
	printk(KERN_ERR "Could not obtain interrupt for 'pulse-buffer-empty' signal\n");
	return -EINVAL;
    }
    picnc->pulse_buffer_empty_irq = ret;

    // Lines allocated, set IO mode
    ret = devm_request_irq(dev, picnc->pulse_buffer_empty_irq, picnic_buffer_empty_irq_handler,
	IRQF_TRIGGER_RISING, "picnic", picnc);
    if (ret < 0) {
	printk(KERN_ERR "Failed to acquire IRQ %d\n", picnc->pulse_buffer_empty_irq);
	return -EINVAL;
    }

    // Output wires
    ret = gpiod_direction_output(picnc->data_ready, 0);
    if (ret < 0) {
	printk(KERN_ERR "Can't set 'data-ready' to output mode\n");
	return -EINVAL;
    }

    ret = gpiod_direction_output(picnc->data_rw, 0);
    if (ret < 0) {
	printk(KERN_ERR "Can't set 'data-rw' to output mode\n");
	return -EINVAL;
    }

    ret = gpiod_direction_output(picnc->data_addr, 0);
    if (ret < 0) {
	printk(KERN_ERR "Can't set 'data-addr' to output mode\n");
	return -EINVAL;
    }

    ret = gpiod_direction_output(picnc->pulse_buffer_lock, 0);
    if (ret < 0) {
	printk(KERN_ERR "Can't set 'lock' to output mode\n");
	return -EINVAL;
    }

    // Initialize device
    picnic_dev = picnic_device_init();
    if (picnic_dev == 0) {
	printk(KERN_ERR "Unable to initialize device in /dev\n");
	return ret;
    }

    // Set data lines for input
    picnic_data_bus_set_input();

    picnc->busy = 0;
    spin_lock_init(&reg_access_lock);

    // Read device ID
    uint16_t device_id = 0;
    ret = picnic_read_register(GPIO_REG_ADDR_DEVICE_ID, &device_id);
    if (ret < 0) {
        printk(KERN_ERR "Could not obtain device ID: %d\n", ret);
	return -EINVAL;
    }

    printk(KERN_INFO "Got PiCNC controller ID 0x%X\n", device_id);

    // Initialize send buffer
    picnc->channels = 4;
    picnic_buffer_init(&(picnic_buffer));

    // Write outputs set all to 'off' state
    picnic_write_register(GPIO_REG_OUTPUTS, 0x0);

    return 0;
}

int picnic_is_empty() {
    return gpiod_get_value(picnc->pulse_buffer_empty) & 0x01;
}

/**
 * Data output
 */
static int picnic_write_data(uint16_t data) {
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
static int picnic_read_data(uint16_t *data) {
    __u32 rv = 0;
    for (int i = 0; i < DATA_WIDTH; i++) {
	uint32_t v = gpiod_get_value(picnc->data[i]);
	rv |= ((v & 0x01) << i);
    }

    *data = (uint16_t)(rv & 0xffff);

    return 0;
}

/**
 * Read register value
 */
int picnic_read_register(uint16_t addr, uint16_t *data) {
//    unsigned long flags = 1;
//    spin_lock_irqsave(&reg_access_lock, flags);

    // Data write mode to write address (sets data bus to Z mode)
    gpiod_set_value(picnc->data_rw, 1);

     int ret = picnic_data_bus_set_output();
    if (ret != 0) {
	printk(KERN_ERR "picnic_read_register: Error setting data bus for output\n");
	return ret;
    }

    // Writing address mode
    gpiod_set_value(picnc->data_addr, 1);

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

    /********************** Check written address ************************/
/*    if (verify) {
	uint16_t rv = 0;
	ret = cnc_gpio_data_input(cnc, &rv);
	if (rv != addr) {
	    fprintf(stderr, "cnc_gpio_command_read: Address 0b%B is wrong expected 0b%B\n", rv, addr);
	    return -1;
	}
    }*/

    /************************* Reading data *****************************/
    gpiod_set_value(picnc->data_addr, 0);

    // Read data
    uint16_t rv = 0;
    ret = picnic_read_data(&rv);
    if (ret != 0) {
	printk(KERN_ERR "picnic_read_register: Error reading data bus\n");
	return ret;
    }

    // Back to write mode
    gpiod_set_value(picnc->data_rw, 1);

//    spin_unlock_irqrestore(&reg_access_lock, flags);

    *data = rv;

    return 0;
}

/**
 * Write register value
 */
int picnic_write_register(uint16_t addr, uint16_t data) {
//    unsigned long flags = 1;
//    spin_lock_irqsave(&reg_access_lock, flags);

    gpiod_set_value(picnc->data_rw, 1); // Data write mode (sets data bus in Z)

     int ret = picnic_data_bus_set_output();
    if (ret != 0) {
	printk(KERN_ERR "picnic_write_register: Error setting data bus for output\n");
	return ret;
    }

    gpiod_set_value(picnc->data_addr, 1); // Writing address mode

    // Write address
    if (picnic_write_data(addr) != 0) {
	printk(KERN_ERR "picnic_write_register: Error writing register address\n");
	return -1;
    }

    /********************** Check written address ************************/
/*    if (verify) {
	ret = cnc_gpio_data_port_set_input(cnc); // Set data lines to input
	if (ret != 0) {
	    fprintf(stderr, "cnc_gpio_command_write: Error setting port for input: %s\n", strerror(errno));
	    return -1;
	}

	ret = cnc_gpio_line_set_value(cnc, GPIO_PIN_DATA_RW, 0); // Data read mode (sets data bus driven by device)
	if (ret != 0) {
	    fprintf(stderr, "cnc_gpio_command_write: Error setting line %d GPIO_PIN_DATA_RW: %s\n", GPIO_PIN_DATA_RW, strerror(errno));
	    return -1;
        }

	// Read written address
	uint16_t rv = 0;
	ret = cnc_gpio_data_input(cnc, &rv);
	if (rv != addr) {
	    fprintf(stderr, "cnc_gpio_command_write: Address 0b%B is wrong expected 0b%B\n", rv, addr);
	    return -1;
	}

	// Back to output mode - we need to output data later
	ret = cnc_gpio_line_set_value(cnc, GPIO_PIN_DATA_RW, 1);  // Data write mode (sets data bus in Z)
	if (ret != 0) {
	    fprintf(stderr, "cnc_gpio_command_write: Error setting line %d GPIO_PIN_DATA_RW: %s\n", GPIO_PIN_DATA_RW, strerror(errno));
	    return -1;
	}

	ret = cnc_gpio_data_port_set_output(cnc);
	if (ret != 0) {
	    fprintf(stderr, "cnc_gpio_command_send: Error setting port for output %s\n", strerror(errno));
	    return ret;
	}
    }*/

    /**************************** Data output ******************************/
    gpiod_set_value(picnc->data_addr, 0);

    if (picnic_write_data(data) != 0) {
	printk(KERN_ERR "picnic_write_register: Error writing register data\n");
	return -1;
    }

    /************************ Verify written data ***************************/
/*    if (verify) {
	// Set data lines to input
	ret = cnc_gpio_data_port_set_input(cnc);
	if (ret != 0) {
	    fprintf(stderr, "cnc_gpio_command_write: Error setting port for input: %s\n", strerror(errno));
	    return -1;
	}

	// Data read mode (sets data bus driven by device)
	ret = cnc_gpio_line_set_value(cnc, GPIO_PIN_DATA_RW, 0);
	if (ret != 0) {
	    fprintf(stderr, "cnc_gpio_command_write: Error setting line %d GPIO_PIN_DATA_RW: %s\n", GPIO_PIN_DATA_RW, strerror(errno));
	    return -1;
	}

	uint16_t rv = 0;
	ret = cnc_gpio_data_input(cnc, &rv);
	if (rv != data) {
	    fprintf(stderr, "cnc_gpio_command_write: Data 0b%B is wrong expected 0b%B\n", rv, data);
	    return -1;
	}
    }*/

    // Data write mode (sets data bus in Z)
    gpiod_set_value(picnc->data_rw, 1);

//    spin_unlock_irqrestore(&reg_access_lock, flags);

    return 0;
}

/**
 * Sets data lines for output
 */
static int picnic_data_bus_set_output() {
    int ret = 0;
    for (int i = 0; i < DATA_WIDTH; i++) {
	ret = gpiod_direction_output(picnc->data[i], GPIOD_OUT_LOW);
	if (ret < 0) return ret;
    }
    picnc->data_mode = 1;
    return ret;
}

/**
 * Sets data lines for input
 */
static int picnic_data_bus_set_input() {
    int ret = 0;
    for (int i = 0; i < DATA_WIDTH; i++) {
	ret = gpiod_direction_input(picnc->data[i]);
	if (ret < 0) return ret;
    }
    picnc->data_mode = 0;
    return ret;
}
