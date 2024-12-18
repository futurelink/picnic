#ifndef _PICNIC_CONST_H_
#define _PICNIC_CONST_H_

#define PICNIC_BUFFER_LEN 512

#define MODULE_NAME "picnc"
#define PICNIC_DEVICE_NAME "picnc"
#define PICNIC_CLASS_NAME "picnc"

#define K_BUFFER_LEN 256

/* PiCNC protocol commands */
#define PICNIC_PROTO_CMD_READ_DEVICE_ID		0x00
#define PICNIC_PROTO_CMD_READ_INPUTS		0x01
#define PICNIC_PROTO_CMD_READ_ENCODERS		0x02
#define PICNIC_PROTO_CMD_READ_OUTPUTS		0x03
#define PICNIC_PROTO_CMD_READ_SERVOS		0x04
#define PICNIC_PROTO_CMD_READ_PWMS		0x05
#define PICNIC_PROTO_CMD_READ_SETTINGS		0x06
#define PICNIC_PROTO_CMD_WRITE_OUTPUTS		0x20
#define PICNIC_PROTO_CMD_WRITE_SERVOS		0x21
#define PICNIC_PROTO_CMD_WRITE_PWMS		0x22
#define PICNIC_PROTO_CMD_WRITE_DIR_HOLD		0x23
#define PICNIC_PROTO_CMD_WRITE_STEP_HOLD	0x2C
#define PICNIC_PROTO_CMD_IS_EMPTY		0xFF

/* Read settings */
#define PICNIC_PROTO_CMD_READ_SETTING_SERVO_CHANNELS    0x01
#define PICNIC_PROTO_CMD_READ_SETTING_PWM_CHANNELS      0x02
#define PICNIC_PROTO_CMD_READ_SETTING_ENCODER_CHANNELS  0x03
#define PICNIC_PROTO_CMD_READ_SETTING_OUTPUTS           0x04
#define PICNIC_PROTO_CMD_READ_SETTING_INPUTS            0x05
#define PICNIC_PROTO_CMD_READ_SETTING_DIR_HOLD          0x06
#define PICNIC_PROTO_CMD_READ_SETTING_STEP_HOLD         0x07

#endif

