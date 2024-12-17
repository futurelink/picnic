#ifndef _PICNIC_MAIN_H_
#define _PICNIC_MAIN_H_

#include "picnic_caps.h"
#include <linux/platform_device.h>

#define DATA_MODE_READ  0
#define DATA_MODE_WRITE 1

typedef struct picnic_t {
    __u16 id;
    struct gpio_desc *data[16];
    struct gpio_desc *data_rw;
    struct gpio_desc *data_addr;
    struct gpio_desc *data_ready;
    struct gpio_desc *pulse_buffer_lock;
    struct gpio_desc *pulse_buffer_empty;

    int pulse_buffer_empty_irq;
    int data_mode; // 0 - input, 1 - output

    __u8 channels;
    volatile __u8 busy;

    /* Capabilities */
    picnic_caps_t caps;
} picnic_t;

int picnic_probe(struct platform_device *pdev);
int picnic_pulses_buffer_send(void);

int picnic_is_empty(void);
int picnic_write_register(__u16 addr, __u16 data);
int picnic_read_register(__u16 addr, __u16 *data);

#endif