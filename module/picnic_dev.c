#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "picnic_dev.h"
#include "picnic_main.h"
#include "picnic_buffer.h"
#include "picnic_const.h"

picnic_dev_t *picnic_dev;
extern picnic_buffer_t picnic_buffer;

// Function prototypes
static int dev_open(struct inode*, struct file*);
static int dev_release(struct inode*, struct file*);
static ssize_t dev_read(struct file*, char*, size_t, loff_t*);
static ssize_t dev_write(struct file*, const char*, size_t, loff_t*);

static void picnic_send_ok(void);
static void picnic_send_error(uint8_t code);

static struct file_operations fops = {
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};

int dev_open(struct inode* inodep, struct file* filep) {
    if (picnic_dev->recv_buffer != 0 || picnic_dev->send_buffer != 0)
	return -EBUSY;

    picnic_dev->recv_buffer = kmalloc(K_BUFFER_LEN, GFP_KERNEL);
    if (picnic_dev->recv_buffer == 0) {
        printk(KERN_INFO "Device buffer allocation failed\n");
        return -1;
    }

    picnic_dev->send_buffer = kmalloc(K_BUFFER_LEN, GFP_KERNEL);
    if (picnic_dev->send_buffer == 0) {
        printk(KERN_INFO "Device buffer allocation failed\n");
        return -1;
    }

    return 0;
}

int dev_release(struct inode* inodep, struct file* filep) {
    if (picnic_dev->recv_buffer != 0) kfree(picnic_dev->recv_buffer);
    if (picnic_dev->send_buffer != 0) kfree(picnic_dev->send_buffer);
    picnic_dev->recv_buffer = 0;
    picnic_dev->send_buffer = 0;
    picnic_dev->send_buffer_len = 0;
    return 0;
}

ssize_t dev_read(struct file* filep, char* buffer, size_t len, loff_t* offset) {
    // Output data to user-space
    if (copy_to_user(buffer, picnic_dev->send_buffer, picnic_dev->send_buffer_len)) {
	printk(KERN_ERR "Failed to copy data to userspace\n");
	return -EFAULT;
    }

    int l = picnic_dev->send_buffer_len;
    picnic_dev->send_buffer_len = 0;

    return l;
}

ssize_t dev_write(struct file* filep, const char* buffer, size_t len, loff_t* offset) {
    size_t l = (len > K_BUFFER_LEN) ? K_BUFFER_LEN : len;
    if (copy_from_user(picnic_dev->recv_buffer, buffer, l)) {
	printk(KERN_ERR "Failed to copy data from userspace\n");
	return -EFAULT;
    }

    // There's data in send buffer that hasn't been read yet.
    // New data can't be sent until old data has been read out.
    if (picnic_dev->send_buffer_len != 0) return 0;

    uint16_t reg = 0;
    uint16_t v = 0;
    switch (picnic_dev->recv_buffer[0]) {
	case PICNIC_PROTO_CMD_READ_DEVICE_ID:
	    picnic_read_register(GPIO_REG_ADDR_DEVICE_ID, &v);
	    for (int i = 0; i < 2; i++) {
		picnic_dev->send_buffer[i] = v >> (i << 3);
	    }
	    picnic_dev->send_buffer_len = 2;
	    break;

	case PICNIC_PROTO_CMD_READ_INPUTS:
	    picnic_read_register(GPIO_REG_INPUTS, &v);
	    for (int i = 0; i < 2; i++) {
		picnic_dev->send_buffer[i] = v >> (i << 3);
	    }
	    picnic_dev->send_buffer_len = 2;
	    break;

	case PICNIC_PROTO_CMD_READ_ENCODERS:
	    picnic_dev->send_buffer_len = 0; // Not supported yet
	    break;

	case PICNIC_PROTO_CMD_READ_OUTPUTS:
	    picnic_read_register(GPIO_REG_OUTPUTS, &v);
	    for (int i = 0; i < 2; i++) {
		picnic_dev->send_buffer[i] = v >> (i << 3);
	    }
	    picnic_dev->send_buffer_len = 2;
	    break;

	case PICNIC_PROTO_CMD_READ_SERVOS:
	    picnic_dev->send_buffer_len = 0; // Not supported yet
	    break;

	case PICNIC_PROTO_CMD_READ_PWMS:
	    picnic_dev->send_buffer_len = 0; // Not supported yet
	    break;

	case PICNIC_PROTO_CMD_WRITE_OUTPUTS:
	    if (l != 3) {
		picnic_send_error(0xFF);
		break;
	    }
	    reg = (picnic_dev->recv_buffer[1] << 8 & 0xff00) | (picnic_dev->recv_buffer[2] & 0xff);
	    picnic_write_register(GPIO_REG_OUTPUTS, reg);
	    picnic_send_ok();
	    break;

	case PICNIC_PROTO_CMD_WRITE_DIR_HOLD:
	    if (l != 3) {
		picnic_send_error(0xFF);
		break;
	    }
	    reg = (picnic_dev->recv_buffer[1] << 8 & 0xff00) | (picnic_dev->recv_buffer[2] & 0xff);
	    picnic_write_register(GPIO_REG_PULSEGEN_DIR_HOLD, reg);
	    picnic_send_ok();
	    break;

	case PICNIC_PROTO_CMD_WRITE_STEP_HOLD:
	    if (l != 3) {
		picnic_send_error(0xFF);
		break;
	    }
	    reg = (picnic_dev->recv_buffer[1] << 8 & 0xff00) | (picnic_dev->recv_buffer[2] & 0xff);
	    picnic_write_register(GPIO_REG_PULSEGEN_STEP_HOLD, reg);
	    picnic_send_ok();
	    break;

	case PICNIC_PROTO_CMD_WRITE_SERVOS:
	    if (l <= 2) {
		picnic_send_error(0xFF); // Error code - command is invalid
		break;
	    }

	    int servo_count = picnic_dev->recv_buffer[1]; // get number of servo channels
	    if (servo_count > PICNIC_MAX_SERVO_CHANNELS) {
		picnic_send_error(0x01); // Error code - servo channels too big
		break;
	    }

	    if (l != servo_count * 4 + 2) {
		picnic_send_error(0x02); // Error code - servo channels and data size mismatch
		break;
	    }

	    if (picnic_buffer_is_full(&picnic_buffer)) {
		printk(KERN_ERR "Buffer is full: tail = %d, head = %d, head_next = %d\n", picnic_buffer.tail, picnic_buffer.head, picnic_buffer.head_next);
		if (picnic_is_empty()) picnic_pulses_buffer_send();
		picnic_send_error(0x03); // Error code - pulses buffer is full
		break;
	    }

	    // Push received data to pulses buffer
	    picnic_state_t st;
	    for (int i = 0; i < servo_count; i++) {
		st.period[i] = (picnic_dev->recv_buffer[i * 4 + 2] << 8 & 0xff00) | (picnic_dev->recv_buffer[i * 4 + 3] & 0xff);
		st.pulses[i] = (picnic_dev->recv_buffer[i * 4 + 4] << 8 & 0xff00) | (picnic_dev->recv_buffer[i * 4 + 5] & 0xff);
	    }
	    //st.outputs = 0xffff;
	    picnic_buffer_push(&picnic_buffer, st);

	    // Execute immediately if possible
	    if (picnic_is_empty()) picnic_pulses_buffer_send();

	    picnic_send_ok();
	    break;

	default:
	    picnic_dev->send_buffer_len = 0;
	    break;
    }

    return l;
}

void picnic_send_ok() {
    picnic_dev->send_buffer[0] = 0x00; // OK marker
    picnic_dev->send_buffer_len = 1;
}

void picnic_send_error(uint8_t code) {
    picnic_dev->send_buffer[0] = 0xFF; // Error marker
    picnic_dev->send_buffer[1] = code;
    picnic_dev->send_buffer_len = 2;
}


picnic_dev_t *picnic_device_init() {
    int ret = 0;
    picnic_dev_t *dev;

    dev = kmalloc(sizeof(picnic_dev_t), GFP_KERNEL);
    if (!dev) {
        printk(KERN_INFO "Failed to allocate space for device\n");
        return 0;
    }

    dev->recv_buffer = 0;
    dev->send_buffer = 0;
    dev->send_buffer_len = 0;

    ret = alloc_chrdev_region(&dev->number, 0, 1, PICNIC_DEVICE_NAME);
    if (ret != 0) {
	printk(KERN_ALERT "Error getting device major number: %d\n", ret);
	return 0;
    }

    dev->class = class_create(PICNIC_CLASS_NAME);
    if (IS_ERR(dev->class)) {
	unregister_chrdev_region(MAJOR(dev->number), 1);
	printk(KERN_ALERT "Error creating device class\n");
	return 0;
    }

    dev->device = device_create(dev->class, NULL, MKDEV(MAJOR(dev->number), 0), NULL, PICNIC_DEVICE_NAME);
    if (IS_ERR(dev->device)) {
	class_destroy(dev->class);
	unregister_chrdev_region(MAJOR(dev->number), 1);
	printk(KERN_ALERT "Error creating device\n");
	return 0;
    }

    cdev_init(&(dev->cdev), &fops);
    ret = cdev_add(&(dev->cdev), MKDEV(MAJOR(dev->number), 0), 1);
    if (ret != 0) {
	device_destroy(dev->class, MKDEV(MAJOR(dev->number), 0));
	class_destroy(dev->class);
	unregister_chrdev_region(MAJOR(dev->number), 1);
        printk(KERN_ALERT "Failed to add character device: %d\n", ret);
        return 0;
    }

    return dev;
}

void picnic_device_deinit(picnic_dev_t *dev) {
    device_destroy(dev->class, MKDEV(MAJOR(dev->number), 0));
    class_destroy(dev->class);
    unregister_chrdev_region(MAJOR(dev->number), 1);
    kfree(picnic_dev);
}
