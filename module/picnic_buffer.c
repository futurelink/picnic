#include "picnic_buffer.h"

// Global buffer
picnic_buffer_t picnic_buffer;

void picnic_buffer_init(picnic_buffer_t *buffer) {
    buffer->head = buffer->tail = 0;
    buffer->head_next = 1;
}

int picnic_buffer_is_full(picnic_buffer_t *buffer) {
    return buffer->head_next == buffer->tail;
}

int picnic_buffer_is_empty(picnic_buffer_t *buffer) {
    return buffer->head == buffer->tail;
}

int picnic_buffer_push(picnic_buffer_t *buffer, picnic_state_t cmd) {
    if (picnic_buffer_is_full(buffer)) return -1;

    unsigned short int next = buffer->head_next + 1;
    if (next == PICNIC_BUFFER_LEN) next = 0;
    buffer->data[buffer->head] = cmd;
    buffer->head = buffer->head_next;
    buffer->head_next = next;

    return 0;
}

picnic_state_t picnic_buffer_pull(picnic_buffer_t *buffer) {
    unsigned short int next = buffer->tail + 1;
    if (next == PICNIC_BUFFER_LEN) next = 0;
    picnic_state_t cmd = buffer->data[buffer->tail];
    buffer->tail = next;
    return cmd;
}
