#ifndef _PICNIC_DEV_H_
#define _PICNIC_DEV_H_

#include <linux/cdev.h>
#include <linux/device.h>

typedef struct picnic_dev_t {
    int number;
    struct class *class;
    struct device *device;
    struct cdev cdev;
    char *recv_buffer;
    char *send_buffer;
    int send_buffer_len;
} picnic_dev_t;

picnic_dev_t *picnic_device_init(void);
void picnic_device_deinit(picnic_dev_t *);

#endif