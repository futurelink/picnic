#ifndef _PICNIC_MAIN_H_
#define _PICNIC_MAIN_H_

#include <linux/platform_device.h>

typedef struct picnic_t {
    struct gpio_desc *data[16];
    struct gpio_desc *data_rw;
    struct gpio_desc *data_addr;
    struct gpio_desc *data_ready;
    struct gpio_desc *pulse_buffer_lock;
    struct gpio_desc *pulse_buffer_empty;

    int pulse_buffer_empty_irq;
    int data_mode; // 0 - input, 1 - output

    uint8_t channels;
    volatile __u8 busy;
} picnic_t;

int picnic_probe(struct platform_device *pdev);
int picnic_pulses_buffer_send(void);

int picnic_is_empty(void);
int picnic_write_register(uint16_t addr, uint16_t data);
int picnic_read_register(uint16_t addr, uint16_t *data);

#endif