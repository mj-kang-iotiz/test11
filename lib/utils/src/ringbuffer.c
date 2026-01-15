#include <string.h>
#include "ringbuffer.h"
#include "dev_assert.h"

#ifndef TAG
    #define TAG "ringbuffer"
#endif

#include "log.h"

void ringbuffer_init(ringbuffer_t *rb, char *buffer, size_t size) {
    DEV_ASSERT(rb != NULL);
    DEV_ASSERT(buffer != NULL);
    DEV_ASSERT(size > 0);

    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
    rb->is_overflow = false;
    rb->overflow_cnt = 0;
}

void ringbuffer_deinit(ringbuffer_t *rb) {
    DEV_ASSERT(rb != NULL);

    memset(rb, 0, sizeof(ringbuffer_t));
}

void ringbuffer_reset(ringbuffer_t *rb) {
    DEV_ASSERT(rb != NULL);

    rb->head = 0;
    rb->tail = 0;
    rb->is_overflow = false;
    rb->overflow_cnt = 0;
}

bool ringbuffer_is_empty(ringbuffer_t *rb) {
    DEV_ASSERT(rb != NULL);

    return ((rb->head == rb->tail));
}

bool ringbuffer_is_full(ringbuffer_t *rb) {
    DEV_ASSERT(rb != NULL);

    return ((rb->head + 1) % rb->size) == rb->tail;
}

bool ringbuffer_is_overflow(ringbuffer_t *rb) {
    DEV_ASSERT(rb != NULL);

    return rb->is_overflow;
}

size_t ringbuffer_get_overflow_count(ringbuffer_t *rb) {
    DEV_ASSERT(rb != NULL);

    return rb->overflow_cnt;
}

size_t ringbuffer_capacity(ringbuffer_t *rb) {
    DEV_ASSERT(rb != NULL);

    return rb->size;
}

size_t ringbuffer_size(ringbuffer_t *rb) {
    DEV_ASSERT(rb != NULL);

    size_t size = rb->size - 1;

    if (!ringbuffer_is_full(rb)) {
        if (rb->head >= rb->tail) {
            size = rb->head - rb->tail;
        } else {
            size = rb->size + rb->head - rb->tail;
        }
    }

    return size;
}

size_t ringbuffer_free_size(ringbuffer_t *rb) {
    DEV_ASSERT(rb != NULL);

    return rb->size - ringbuffer_size(rb) - 1;
}

void ringbuffer_write(ringbuffer_t *rb, const char *data, size_t len) {
    DEV_ASSERT(rb != NULL);
    DEV_ASSERT(data != NULL);
    DEV_ASSERT(len != 0 && len < ringbuffer_capacity(rb));
    
    size_t free_space = ringbuffer_free_size(rb);
    
    if(len > free_space)
    {
        size_t overflow_len = len - free_space;
        rb->tail = (rb->tail + overflow_len) %  rb->size;
        rb->is_overflow = true;
        rb->overflow_cnt += overflow_len;
    }

    size_t first_chunk = rb->size - rb->head;

    if(first_chunk >= len)
    {
        memcpy(&rb->buffer[rb->head], data, len);
    }
    else
    {
        memcpy(&rb->buffer[rb->head], data, first_chunk);
        memcpy(&rb->buffer[0], data + first_chunk, len - first_chunk);
    }

    rb->head = (rb->head + len) % rb->size;
}

void ringbuffer_write_byte(ringbuffer_t *rb, char data)
{
    DEV_ASSERT(rb != NULL);

    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->size;

    if(rb->head == rb->tail)
    {
        rb->tail = (rb->tail + 1) % rb->size;
        rb->is_overflow = true;
        rb->overflow_cnt++;
    }
}

size_t ringbuffer_read(ringbuffer_t *rb, char *data, size_t len)
{
    DEV_ASSERT(rb != NULL);
    DEV_ASSERT(data != NULL);
    DEV_ASSERT(len != 0 && len < ringbuffer_capacity(rb));

    size_t available = ringbuffer_size(rb);
    size_t read_len = (len <= available) ? len : available;

    if (read_len == 0) {
        return 0;
    }

    size_t first_chunk = rb->size - rb->tail;

    if (first_chunk >= read_len) {
        memcpy(data, &rb->buffer[rb->tail], read_len);
    } else {
        memcpy(data, &rb->buffer[rb->tail], first_chunk);
        memcpy(data + first_chunk, &rb->buffer[0], read_len - first_chunk);
    }

    rb->tail = (rb->tail + read_len) % rb->size;

    return read_len;
}

bool ringbuffer_read_byte(ringbuffer_t *rb, char *data) {
    DEV_ASSERT(rb != NULL);
    DEV_ASSERT(data != NULL);
    
    if (ringbuffer_is_empty(rb)) {
        return false;
    }

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;

    return true;
}

bool ringbuffer_peek(ringbuffer_t *rb, char *data, size_t len, size_t offset) {
    DEV_ASSERT(rb != NULL);
    DEV_ASSERT(data != NULL);

    // len == 0이면 아무것도 안 함
    if (len == 0) {
        return true;
    }

    size_t current_size = ringbuffer_size(rb);

    // 데이터 부족하면 false 리턴 (assert로 죽이지 않음)
    if (offset >= current_size || len > (current_size - offset)) {
        return false;
    }

    size_t peek_start_idx = (rb->tail + offset) % rb->size;
    size_t need_wrap = rb->size - peek_start_idx < len;

    if (!need_wrap) {
        memcpy(data, &rb->buffer[peek_start_idx], len);
    } else {
        size_t first_chunk = rb->size - peek_start_idx;
        size_t second_chunk = len - first_chunk;
        memcpy(data, &rb->buffer[peek_start_idx], first_chunk);
        memcpy(data + first_chunk, &rb->buffer[0], second_chunk);
    }

    return true;
}

bool ringbuffer_peek_byte(ringbuffer_t *rb, char *data) {
    DEV_ASSERT(rb != NULL);
    DEV_ASSERT(data != NULL);

    if (ringbuffer_is_empty(rb)) {
        return false;
    }

    *data = rb->buffer[rb->tail];
    return true;
}

bool ringbuffer_advance(ringbuffer_t *rb, size_t len)
{
    DEV_ASSERT(rb != NULL);
    DEV_ASSERT(len < ringbuffer_capacity(rb));

    size_t available = ringbuffer_size(rb);
    
    if(len > available)
    {
        ringbuffer_reset(rb);
        return false;
    }

    rb->tail = (rb->tail + len) % rb->size;
    return true;
}

