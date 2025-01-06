#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "picnic_dev.h"
#include "picnic_main.h"
#include "picnic_buffer.h"
#include "picnic_const.h"

uint8_t crc_table[256] = {
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

/* Command lenghts */
uint8_t cmd_lengths[256];

picnic_dev_t *picnic_dev;

/* External global variables */
extern picnic_t *picnc;
extern picnic_buffer_t picnic_buffer;

// Function prototypes
static int dev_open(struct inode*, struct file*);
static int dev_release(struct inode*, struct file*);
static ssize_t dev_read(struct file*, char*, size_t, loff_t*);
static ssize_t dev_write(struct file*, const char*, size_t, loff_t*);
static void dev_exec_command(const char *buffer, __u8 *recv_offset, __u8 *send_offset, picnic_state_t *st);

static void picnic_send_ok(__u8 cmd, __u8 *send_offset);
static void picnic_send_status(__u8 cmd, __u8 *send_offset, uint8_t code);

static int dev_check_crc(const char *buffer, size_t len);
static uint8_t crc8(const uint8_t bytes[], size_t length);

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

    // Check command batch integrity
    if (dev_check_crc(picnic_dev->recv_buffer, len) != 0) {
	printk(KERN_ERR "%s: Command received with CRC error\n", MODULE_NAME);
	return 0;
    }

    // Parse and execute command batch
    __u8 recv_offset = 0;
    __u8 send_offset = 0;
    __u8 command_cnt = picnic_dev->recv_buffer[recv_offset++];
    picnic_state_t st;
    memset(&st, 0, sizeof(picnic_state_t));

    for (__u8 cmd = 0; cmd < command_cnt; cmd++) {
	dev_exec_command(picnic_dev->recv_buffer, &recv_offset, &send_offset, &st);
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

    picnic_dev->send_buffer_len = send_offset;

    return l;
}

int dev_check_crc(const char *buffer, size_t len) {
    int l = 1; // Skip 1st byte which is command count
    while (l < len) {
	uint8_t cmd_len = cmd_lengths[(uint8_t)buffer[l]];
	if (cmd_len == 0) return -1;
	uint8_t *cmd_t = (uint8_t *)&buffer[l];
	if (crc8(cmd_t, cmd_len) != cmd_t[cmd_len]) return -1;
	l += cmd_len + 1;
    }
    return 0;
}

void dev_exec_command(const char *buffer, __u8 *recv_offset, __u8 *send_offset, picnic_state_t *st) {
    __u16 reg = 0;
    __u16 v = 0;
    __u8 bank = 0;
    __u8 recv_offset_t = *recv_offset;
    __u8 send_offset_t = *send_offset;
    __u8 cmd = picnic_dev->recv_buffer[recv_offset_t++];
    switch (cmd) {
        case PICNIC_PROTO_CMD_READ_DEVICE_ID:
	    if (picnic_read_register(PICNIC_DEVICE_ID_REGISTER, &v) == 0) {
		picnic_send_ok(cmd, &send_offset_t);
		picnic_dev->send_buffer[send_offset_t++] = (v >> 8) & 0xff;
		picnic_dev->send_buffer[send_offset_t++] = (v) & 0xff;
	    } else {
		picnic_send_status(cmd, &send_offset_t, 0xFF);
	    }
	    break;

	/* Read input values */
	case PICNIC_PROTO_CMD_READ_INPUTS:
	    bank = picnic_dev->recv_buffer[recv_offset_t++];
	    if (picnc->caps.input_banks == 0) {
		picnic_send_status(cmd, &send_offset_t, 0xFF); // No input banks
		break;
	    }
	    picnic_send_ok(cmd, &send_offset_t);
	    picnic_dev->send_buffer[send_offset_t++] = picnc->caps.input_banks;
	    for (__u8 bank = 0; bank < picnc->caps.input_banks; bank++) {
		if (picnic_read_register(picnc->caps.input_addrs[bank], &v) == 0) {
		    picnic_dev->send_buffer[send_offset_t++] = (v >> 8) & 0xff;
		    picnic_dev->send_buffer[send_offset_t++] = (v) & 0xff;
		} else {
		    send_offset_t = *send_offset; // Roll back send offset for current command
		    picnic_send_status(cmd, &send_offset_t, 0xFF);
		}
	    }
	    break;

	/* Read current output values */
	case PICNIC_PROTO_CMD_READ_OUTPUTS:
	    if (picnc->caps.output_banks == 0) {
		picnic_send_status(cmd, &send_offset_t, 0xFF); // No output banks
		break;
	    }

	    bank = picnic_dev->recv_buffer[recv_offset_t++];
	    picnic_send_ok(cmd, &send_offset_t);
	    picnic_dev->send_buffer[send_offset_t++] = picnc->caps.output_banks;
	    for (__u8 bank = 0; bank < picnc->caps.output_banks; bank++) {
	        if (picnic_read_register(picnc->caps.output_addrs[bank], &v) == 0) {
		    picnic_dev->send_buffer[send_offset_t++] = (v >> 8) & 0xff;
		    picnic_dev->send_buffer[send_offset_t++] = (v) & 0xff;
		} else {
		    send_offset_t = *send_offset; // Roll back send offset for current command
		    picnic_send_status(cmd, &send_offset_t, 0xFF);
		}
	    }
	    break;

	/* Read current encoder values */
	case PICNIC_PROTO_CMD_READ_ENCODERS: // Not supported yet
	    if (picnc->caps.encoder_channels == 0) {
		picnic_send_status(cmd, &send_offset_t, 0xFF); // No encoder channels available
		break;
	    }
	    picnic_send_status(cmd, &send_offset_t, 0xFF); // Error code - command is invalid
	    break;

	/* Read current servo pulses generation values */
	case PICNIC_PROTO_CMD_READ_SERVOS: // Not supported yet
	    if (picnc->caps.servo_channels == 0) {
		picnic_send_status(cmd, &send_offset_t, 0xFF); // No servo channels available
		break;
	    }
	    picnic_send_status(cmd, &send_offset_t, 0xFF); // Error code - command is invalid
	    break;

	/* Read current PWM values */
	case PICNIC_PROTO_CMD_READ_PWMS: // Not supported yet
	    if (picnc->caps.pwm_channels == 0) {
		picnic_send_status(cmd, &send_offset_t, 0xFF); // Error code - command is invalid
		break;
	    }
	    picnic_send_status(cmd, &send_offset_t, 0xFF); // Error code - command is invalid
	    break;

	/* Gets current positions either from kernel driver or hardware (not supported yet) */
	case PICNIC_PROTO_CMD_READ_POSITIONS:
	    if (picnc->caps.servo_channels == 0) {
		picnic_send_status(cmd, &send_offset_t, 0xFF); // No servo channels available
		break;
	    }

	    if (!picnc->caps.servo_holds_position) {
		picnic_send_ok(cmd, &send_offset_t);
		picnic_dev->send_buffer[send_offset_t++] = picnc->caps.servo_channels;
		for (int n = 0; n < picnc->caps.servo_channels; n++) {
		    picnic_dev->send_buffer[send_offset_t++] = (picnc->servo_positions[n] >> 24) & 0xFF;
		    picnic_dev->send_buffer[send_offset_t++] = (picnc->servo_positions[n] >> 16) & 0xFF;
		    picnic_dev->send_buffer[send_offset_t++] = (picnc->servo_positions[n] >> 8) & 0xFF;
		    picnic_dev->send_buffer[send_offset_t++] = picnc->servo_positions[n] & 0xFF;
		}
	    } else {
		picnic_send_status(cmd, &send_offset_t, 0x01); // No positions available
	    }

	    break;

	/* Read settings from either device or kernel driver */
	case PICNIC_PROTO_CMD_READ_SETTINGS:
	    __u8 setting = picnic_dev->recv_buffer[recv_offset_t++];
	    switch (setting) {
		case PICNIC_PROTO_CMD_READ_SETTING_SERVO_CHANNELS: // Servo channels number
		    picnic_send_ok(cmd, &send_offset_t);
		    picnic_dev->send_buffer[send_offset_t++] = picnc->caps.servo_channels;
		    break;
		case PICNIC_PROTO_CMD_READ_SETTING_PWM_CHANNELS: // PWM channels number
		    picnic_send_ok(cmd, &send_offset_t);
		    picnic_dev->send_buffer[send_offset_t++] = picnc->caps.pwm_channels;
		    break;
		case PICNIC_PROTO_CMD_READ_SETTING_OUTPUTS: // Outputs count (x16 per bank)
		    picnic_send_ok(cmd, &send_offset_t);
		    picnic_dev->send_buffer[send_offset_t++] = picnc->caps.output_banks * 16;
		    break;
		case PICNIC_PROTO_CMD_READ_SETTING_INPUTS: // Inputs count (x16 per bank)
		    picnic_send_ok(cmd, &send_offset_t);
		    picnic_dev->send_buffer[send_offset_t++] = picnc->caps.input_banks * 16;
		    break;
		case PICNIC_PROTO_CMD_READ_SETTING_DIR_HOLD: // DIR hold in ticks
		    if (picnic_read_register(picnc->caps.dir_hold_addr, &v) == 0) {
			picnic_send_ok(cmd, &send_offset_t);
			picnic_dev->send_buffer[send_offset_t++] = v;
		    } else {
			picnic_send_status(cmd, &send_offset_t, 0xFF); // DIR hold can't be FF (255) so this is treated as error
		    }
		    break;
		case PICNIC_PROTO_CMD_READ_SETTING_STEP_HOLD: // STEP hold in ticks
		    if (picnic_read_register(picnc->caps.step_hold_addr, &v) == 0) {
			picnic_send_ok(cmd, &send_offset_t);
			picnic_dev->send_buffer[send_offset_t++] = v;
		    } else {
			picnic_send_status(cmd, &send_offset_t, 0xFF); // STEP hold can't be FF (255) so this is treated as error
		    }
		    break;
		default:
		    picnic_send_status(cmd, &send_offset_t, 0xFF); // Error code - command is invalid
	    }
	    break;

	/* Get buffer utilization */
	case PICNIC_PROTO_CMD_READ_BUFFER_UTIL:
	    picnic_send_ok(cmd, &send_offset_t);
	    picnic_dev->send_buffer[send_offset_t++] = picnic_buffer_get_utilization(&picnic_buffer);
	    break;

	case PICNIC_PROTO_CMD_WRITE_DIR_HOLD:
/*	    if (l != 3) {
		picnic_send_status(cmd, &send_offset_t, 0xFF);
		break;
	    }*/
	    reg = (picnic_dev->recv_buffer[recv_offset_t] << 8 & 0xff00) | (picnic_dev->recv_buffer[recv_offset_t + 1] & 0xff);
	    recv_offset_t += 2;
	    if (picnic_write_register(picnc->caps.dir_hold_addr, reg) == 0) {
		picnic_send_ok(cmd, &send_offset_t);
	    } else {
		picnic_send_status(cmd, &send_offset_t, 0xFF); // Error code 0xFF
	    }
	    break;

	case PICNIC_PROTO_CMD_WRITE_STEP_HOLD:
/*	    if (l != 3) {
		picnic_send_status(cmd, &send_offset_t, 0xFF);
		break;
	    }*/
	    reg = (picnic_dev->recv_buffer[recv_offset_t] << 8 & 0xff00) | (picnic_dev->recv_buffer[recv_offset_t + 1] & 0xff);
	    recv_offset_t += 2;
	    if (picnic_write_register(picnc->caps.step_hold_addr, reg) == 0) {
		picnic_send_ok(cmd, &send_offset_t);
	    } else {
		picnic_send_status(cmd, &send_offset_t, 0xFF); // Error code 0xFF
	    }
	    break;

	case PICNIC_PROTO_CMD_WRITE_OUTPUTS:
	    bank = picnic_dev->recv_buffer[recv_offset_t++];
	    if (bank > picnc->caps.output_banks) { // Command is invalid, bank is invalid
		picnic_send_status(cmd, &send_offset_t, 0xFF);
		break;
	    }
	    reg = (picnic_dev->recv_buffer[recv_offset_t] << 8 & 0xff00) | (picnic_dev->recv_buffer[recv_offset_t + 1] & 0xff);
	    recv_offset_t += 2;
	    st->outputs[bank] = reg;
	    st->update_flags |= PICNIC_BUFFER_UPDATED_OUTPUTS;
	    picnic_send_ok(cmd, &send_offset_t);
	    break;

	case PICNIC_PROTO_CMD_WRITE_SERVOS:
	    if (picnic_buffer_is_full(&picnic_buffer)) {
		printk(KERN_ERR "%s: Buffer is full (tail = %d, head = %d, head_next = %d)\n", MODULE_NAME, picnic_buffer.tail, picnic_buffer.head, picnic_buffer.head_next);
		picnic_pulses_buffer_send(); // Try to send buffer data to free some space
		picnic_send_status(cmd, &send_offset_t, 0x03); // Error code - pulses buffer is full
	    } else {
		// Push received data to pulses buffer
		for (int i = 0; i < picnc->caps.servo_channels; i++) {
		    st->period[i] = (picnic_dev->recv_buffer[recv_offset_t + i * 4] << 8 & 0xff00) | (picnic_dev->recv_buffer[recv_offset_t + i * 4 + 1] & 0xff);
		    st->pulses[i] = (picnic_dev->recv_buffer[recv_offset_t + i * 4 + 2] << 8 & 0xff00) | (picnic_dev->recv_buffer[recv_offset_t + i * 4 + 3] & 0xff);
		}
		st->update_flags |= PICNIC_BUFFER_UPDATED_SERVOS;
	    }
	    recv_offset_t += picnc->caps.servo_channels * 4;
	    picnic_send_ok(cmd, &send_offset_t);
	    break;

	default:
	    picnic_send_status(cmd, &send_offset_t, 0xFF); // Command is unknown
	    break;
    }

    recv_offset_t++; // Skip command CRC byte

    *recv_offset = recv_offset_t;
    *send_offset = send_offset_t;
}

void picnic_send_status(__u8 cmd, __u8 *send_offset, uint8_t code) {
    __u8 offset = *(send_offset);
    picnic_dev->send_buffer[offset] = cmd;
    picnic_dev->send_buffer[offset+1] = code;
    *(send_offset) = offset + 2;
}

void picnic_send_ok(__u8 cmd, __u8 *send_offset) {
    picnic_send_status(cmd, send_offset, 0x00);
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

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,4,0)
    dev->class = class_create(THIS_MODULE, PICNIC_CLASS_NAME);
#else
    dev->class = class_create(PICNIC_CLASS_NAME);
#endif
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

    /* Fill command lengths array */
    cmd_lengths[PICNIC_PROTO_CMD_READ_DEVICE_ID] = 1;
    cmd_lengths[PICNIC_PROTO_CMD_READ_INPUTS] = 2;
    cmd_lengths[PICNIC_PROTO_CMD_READ_ENCODERS] = 1;
    cmd_lengths[PICNIC_PROTO_CMD_READ_OUTPUTS] = 2;
    cmd_lengths[PICNIC_PROTO_CMD_READ_SERVOS] = 1;
    cmd_lengths[PICNIC_PROTO_CMD_READ_PWMS] = 1;
    cmd_lengths[PICNIC_PROTO_CMD_READ_SETTINGS] = 2;
    cmd_lengths[PICNIC_PROTO_CMD_READ_POSITIONS] = 1;
    cmd_lengths[PICNIC_PROTO_CMD_READ_BUFFER_UTIL] = 1;
    cmd_lengths[PICNIC_PROTO_CMD_WRITE_OUTPUTS] = 4;
    cmd_lengths[PICNIC_PROTO_CMD_WRITE_SERVOS] = 1 + picnc->caps.servo_channels * 4;
    cmd_lengths[PICNIC_PROTO_CMD_WRITE_PWMS] = 1 + picnc->caps.pwm_channels * 2;
    cmd_lengths[PICNIC_PROTO_CMD_WRITE_DIR_HOLD] = 3;
    cmd_lengths[PICNIC_PROTO_CMD_WRITE_STEP_HOLD] = 3;
    cmd_lengths[PICNIC_PROTO_CMD_IS_EMPTY] = 1;

    printk(KERN_INFO "%s: Initialized with %d servo & %d PWM channels", MODULE_NAME, picnc->caps.servo_channels, picnc->caps.pwm_channels);

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

uint8_t crc8(const uint8_t bytes[], size_t length) {
    uint8_t crc = 0;
    for (int i = 0; i < length; i++) {
        uint8_t data = (uint8_t)(bytes[i] ^ crc);  /* XOR-in next input byte */
        crc = (uint8_t)(crc_table[data]); /* get current CRC value = remainder */
    }

    return crc;
}
