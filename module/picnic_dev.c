#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "picnic_dev.h"
#include "picnic_main.h"
#include "picnic_buffer.h"
#include "picnic_const.h"

picnic_dev_t *picnic_dev;

/* External global variables */
extern picnic_t *picnc;
extern picnic_buffer_t picnic_buffer;

// Function prototypes
static int dev_open(struct inode*, struct file*);
static int dev_release(struct inode*, struct file*);
static ssize_t dev_read(struct file*, char*, size_t, loff_t*);
static ssize_t dev_write(struct file*, const char*, size_t, loff_t*);
static __u8 dev_exec_command(const char *buffer, __u8 *recv_offset, __u8 send_offset, picnic_state_t *st);

static __u8 picnic_send_ok(__u8 cmd, __u8 send_offset);
static __u8 picnic_send_status(__u8 cmd, __u8 send_offset, uint8_t code);


static struct file_operations fops = {
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};

int dev_open(struct inode* inodep, struct file* filep) {
    return 0;
}

int dev_release(struct inode* inodep, struct file* filep) {
    return 0;
}

ssize_t dev_read(struct file* filep, char* buffer, size_t len, loff_t* offset) {
    // Output data to user-space
    if (copy_to_user(buffer, picnic_dev->send_buffer, picnic_dev->send_buffer_len)) {
	printk(KERN_ERR "%s: Failed to copy data to userspace\n", MODULE_NAME);
	return -EFAULT;
    }

    int l = picnic_dev->send_buffer_len;
    picnic_dev->send_buffer_len = 0;

    return l;
}

ssize_t dev_write(struct file* filep, const char* buffer, size_t len, loff_t* offset) {
    size_t l = (len > K_BUFFER_LEN) ? K_BUFFER_LEN : len;
    if (copy_from_user(picnic_dev->recv_buffer, buffer, l)) {
	printk(KERN_ERR "%s: Failed to copy data from userspace\n", MODULE_NAME);
	return -EFAULT;
    }

    // There's data in send buffer that hasn't been read yet.
    // New data can't be sent until old data has been read out.
    if (picnic_dev->send_buffer_len != 0) return 0;

    __u8 recv_offset = 0;
    __u8 sLen = 0;
    __u8 command_cnt = picnic_dev->recv_buffer[recv_offset++];
    picnic_state_t st;
    memset(&st, 0, sizeof(picnic_state_t));
    for (__u8 cmd = 0; cmd < command_cnt; cmd++) {
	sLen += dev_exec_command(picnic_dev->recv_buffer, &recv_offset, sLen, &st);
	if (recv_offset > l) { // Must be error!
	    printk(KERN_ERR "%s; Device command corrupted\n", MODULE_NAME);
	    break;
	}
    }

    // If state was updated during command batch execution,
    // then it needs to be send.
    if (st.update_flags) {
	picnic_buffer_push(&picnic_buffer, st);
	picnic_pulses_buffer_send(); // Execute immediately if possible
    }

    picnic_dev->send_buffer_len = sLen;

    return l;
}

__u8 dev_exec_command(const char *buffer, __u8 *recv_offset, __u8 send_offset, picnic_state_t *st) {
    int sLen = 0;
    __u16 reg = 0;
    __u16 v = 0;
    __u8 recv_offset_t = *recv_offset;
    __u8 cmd = picnic_dev->recv_buffer[recv_offset_t++];
    switch (cmd) {
        case PICNIC_PROTO_CMD_READ_DEVICE_ID:
	    picnic_read_register(PICNIC_DEVICE_ID_REGISTER, &v);
	    picnic_dev->send_buffer[send_offset++] = cmd;
	    for (int i = 0; i < 2; i++) {
		picnic_dev->send_buffer[send_offset++] = v >> (i << 3);
		sLen++;
	    }
	    break;

	case PICNIC_PROTO_CMD_READ_INPUTS:
	    picnic_dev->send_buffer[send_offset++] = cmd;
	    picnic_dev->send_buffer[send_offset++] = picnc->caps.input_banks;
	    for (__u8 bank = 0; bank < picnc->caps.input_banks; bank++) {
		picnic_read_register(picnc->caps.input_addrs[bank], &v);
		for (int i = 0; i < 2; i++) {
		    picnic_dev->send_buffer[send_offset++] = v >> (i << 3);
		    sLen++;
		}
	    }
	    break;

	case PICNIC_PROTO_CMD_READ_ENCODERS: // Not supported yet
	    sLen = picnic_send_status(cmd, send_offset, 0xFF); // Error code - command is invalid
	    break;

	case PICNIC_PROTO_CMD_READ_OUTPUTS:
	    picnic_dev->send_buffer[send_offset++] = cmd;
	    picnic_dev->send_buffer[send_offset++] = picnc->caps.output_banks;
	    for (__u8 bank = 0; bank < picnc->caps.output_banks; bank++) {
		picnic_read_register(picnc->caps.output_addrs[bank], &v);
		for (int i = 0; i < 2; i++) {
		    picnic_dev->send_buffer[send_offset++] = v >> (i << 3);
		    sLen++;
		}
	    }
	    break;

	case PICNIC_PROTO_CMD_READ_SERVOS: // Not supported yet
	    sLen = picnic_send_status(cmd, send_offset, 0xFF); // Error code - command is invalid
	    break;

	case PICNIC_PROTO_CMD_READ_PWMS: // Not supported yet
	    sLen = picnic_send_status(cmd, send_offset, 0xFF); // Error code - command is invalid
	    break;

	case PICNIC_PROTO_CMD_WRITE_DIR_HOLD:
/*	    if (l != 3) {
		sLen = picnic_send_status(cmd, send_offset, 0xFF);
		break;
	    }*/
	    reg = (picnic_dev->recv_buffer[recv_offset_t] << 8 & 0xff00) | (picnic_dev->recv_buffer[recv_offset_t + 1] & 0xff);
	    recv_offset_t += 2;
	    picnic_write_register(picnc->caps.dir_hold_addr, reg);
	    sLen = picnic_send_ok(cmd, send_offset);
	    break;

	case PICNIC_PROTO_CMD_WRITE_STEP_HOLD:
/*	    if (l != 3) {
		sLen = picnic_send_status(cmd, send_offset, 0xFF);
		break;
	    }*/
	    reg = (picnic_dev->recv_buffer[recv_offset_t] << 8 & 0xff00) | (picnic_dev->recv_buffer[recv_offset_t + 1] & 0xff);
	    recv_offset_t += 2;
	    picnic_write_register(picnc->caps.step_hold_addr, reg);
	    sLen = picnic_send_ok(cmd, send_offset);
	    break;

	case PICNIC_PROTO_CMD_WRITE_OUTPUTS:
	    __u8 bank = picnic_dev->recv_buffer[recv_offset_t++];
	    if (bank > picnc->caps.output_banks) { // Command is invalid, bank is invalid
		sLen = picnic_send_status(cmd, send_offset, 0xFF);
		break;
	    }
	    reg = (picnic_dev->recv_buffer[recv_offset_t] << 8 & 0xff00) | (picnic_dev->recv_buffer[recv_offset_t + 1] & 0xff);
	    recv_offset_t += 2;
	    //picnic_write_register(picnc->caps.output_addrs[bank], reg);
	    st->outputs[bank] = reg;
	    st->update_flags |= PICNIC_BUFFER_UPDATED_OUTPUTS;
	    sLen = picnic_send_ok(cmd, send_offset);
	    break;

	case PICNIC_PROTO_CMD_WRITE_SERVOS:
	    /*if (l <= 2) {
		sLen = picnic_send_status(cmd, send_offset, 0xFF); // Error code - command is invalid
		break;
	    }*/

	    int servo_count = picnic_dev->recv_buffer[recv_offset_t++]; // get number of servo channels
	    if (servo_count > picnc->caps.servo_channels) {
		sLen = picnic_send_status(cmd, send_offset, 0x01); // Error code - servo channels too big
		break;
	    }

	    /*if (l != servo_count * 4 + 2) {
		sLen = picnic_send_status(cmd, send_offset, 0x02); // Error code - servo channels and data size mismatch
		break;
	    }*/

	    if (picnic_buffer_is_full(&picnic_buffer)) {
		printk(KERN_ERR "%s: Buffer is full (tail = %d, head = %d, head_next = %d)\n", MODULE_NAME, picnic_buffer.tail, picnic_buffer.head, picnic_buffer.head_next);
		picnic_pulses_buffer_send(); // Try to send buffer data to free some space
		sLen = picnic_send_status(cmd, send_offset, 0x03); // Error code - pulses buffer is full
		break;
	    }

	    // Push received data to pulses buffer
	    for (int i = 0; i < servo_count; i++) {
		st->period[i] = (picnic_dev->recv_buffer[recv_offset_t + i * 4] << 8 & 0xff00) | (picnic_dev->recv_buffer[recv_offset_t + i * 4 + 1] & 0xff);
		st->pulses[i] = (picnic_dev->recv_buffer[recv_offset_t + i * 4 + 2] << 8 & 0xff00) | (picnic_dev->recv_buffer[recv_offset_t + i * 4 + 3] & 0xff);
	    }
	    recv_offset_t += 4 * servo_count;
	    st->update_flags |= PICNIC_BUFFER_UPDATED_SERVOS;

	    sLen = picnic_send_ok(cmd, send_offset);
	    break;

	default: break;
    }

    *recv_offset = recv_offset_t;

    return sLen;
}

__u8 picnic_send_status(__u8 cmd, __u8 send_offset, uint8_t code) {
    picnic_dev->send_buffer[send_offset++] = cmd;
    picnic_dev->send_buffer[send_offset++] = code;
    return 2;
}

__u8 picnic_send_ok(__u8 cmd, __u8 send_offset) {
    return picnic_send_status(cmd, send_offset, 0x00);
}


picnic_dev_t *picnic_device_init() {
    int ret = 0;
    picnic_dev_t *dev;

    dev = kmalloc(sizeof(picnic_dev_t), GFP_KERNEL);
    if (!dev) {
        printk(KERN_INFO "%s: Failed to allocate space for device\n", MODULE_NAME);
        return 0;
    }

    // Allocate buffers
    dev->recv_buffer = kmalloc(K_BUFFER_LEN, GFP_KERNEL);
    if (dev->recv_buffer == 0) {
        printk(KERN_INFO "%s: Device recieve buffer allocation failed\n", MODULE_NAME);
        return 0;
    }

    dev->send_buffer = kmalloc(K_BUFFER_LEN, GFP_KERNEL);
    if (dev->send_buffer == 0) {
        printk(KERN_INFO "%s: Device send buffer allocation failed\n", MODULE_NAME);
        return 0;
    }

    dev->send_buffer_len = 0;

    // Create device
    ret = alloc_chrdev_region(&dev->number, 0, 1, PICNIC_DEVICE_NAME);
    if (ret != 0) {
	printk(KERN_ALERT "%s: Error getting device major number: %d\n", MODULE_NAME, ret);
	return 0;
    }

    dev->class = class_create(PICNIC_CLASS_NAME);
    if (IS_ERR(dev->class)) {
	unregister_chrdev_region(MAJOR(dev->number), 1);
	printk(KERN_ALERT "%s: Error creating device class\n", MODULE_NAME);
	return 0;
    }

    dev->device = device_create(dev->class, NULL, MKDEV(MAJOR(dev->number), 0), NULL, PICNIC_DEVICE_NAME);
    if (IS_ERR(dev->device)) {
	class_destroy(dev->class);
	unregister_chrdev_region(MAJOR(dev->number), 1);
	printk(KERN_ALERT "%s: Error creating device\n", MODULE_NAME);
	return 0;
    }

    cdev_init(&(dev->cdev), &fops);
    ret = cdev_add(&(dev->cdev), MKDEV(MAJOR(dev->number), 0), 1);
    if (ret != 0) {
	device_destroy(dev->class, MKDEV(MAJOR(dev->number), 0));
	class_destroy(dev->class);
	unregister_chrdev_region(MAJOR(dev->number), 1);
        printk(KERN_ALERT "%s: Failed to add character device: %d\n", MODULE_NAME, ret);
        return 0;
    }


    return dev;
}

void picnic_device_deinit(picnic_dev_t *dev) {
    if (dev == 0) return;

    // Close and remove device
    device_destroy(dev->class, MKDEV(MAJOR(dev->number), 0));
    class_destroy(dev->class);
    unregister_chrdev_region(MAJOR(dev->number), 1);

    // Free buffers
    if (dev->recv_buffer != 0) kfree(dev->recv_buffer);
    if (dev->send_buffer != 0) kfree(dev->send_buffer);
    dev->recv_buffer = 0;
    dev->send_buffer = 0;
    dev->send_buffer_len = 0;

    // Free device structure
    kfree(dev);
}
