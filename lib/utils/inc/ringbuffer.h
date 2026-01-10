#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 링 버퍼 구조체
 * 
 */
typedef struct {
    char *buffer;
    volatile size_t head;
    volatile size_t tail;
    size_t size;
    bool is_overflow;
    size_t overflow_cnt;
} ringbuffer_t;

/**
 * @brief 링버퍼 초기화
 * 
 * @param rb 링버퍼 핸들
 * @param buffer 버퍼
 * @param size 버퍼 크기
 */
void ringbuffer_init(ringbuffer_t *rb, char *buffer, size_t size);

/**
 * @brief 링버퍼 해제
 * 
 * @param rb 링버퍼 핸들
 */
void ringbuffer_deinit(ringbuffer_t *rb);

/**
 * @brief 링버퍼 리셋
 * 
 * @param rb 링버퍼 핸들
 */
void ringbuffer_reset(ringbuffer_t *rb);

/**
 * @brief 버퍼가 비어있는지 확인
 * 
 * @param rb 링버퍼 핸들
 * @return true 비어있음
 * @return false 비어있지 않음
 */
bool ringbuffer_is_empty(ringbuffer_t *rb);

/**
 * @brief 버퍼가 가득차 있는지 확인
 * 
 * @param rb 링버퍼 핸들
 * @return true 가득차 있음
 * @return false 공간 있음
 */
bool ringbuffer_is_full(ringbuffer_t *rb);

/**
 * @brief 오버플로우 상태 확인
 * 
 * @param rb 링버퍼 핸들
 * @return true 오버플로우 발생
 * @return false 오버플로우 없음
 */
bool ringbuffer_is_overflow(ringbuffer_t *rb);

/**
 * @brief 오버플로우 횟수 반환
 * 
 * @param rb 링버퍼 핸들
 * @return size_t 오버플로우 횟수
 */
size_t ringbuffer_get_overflow_count(ringbuffer_t *rb);

/**
 * @brief 버퍼 사이즈 반환
 * 
 * @param rb 링버퍼 핸들
 * @return size_t 버퍼 사이즈
 */
size_t ringbuffer_capacity(ringbuffer_t *rb);

/**
 * @brief 버퍼에 채워져 있는 데이터 길이 반환
 * 
 * @param rb 링버퍼 핸들
 * @return size_t 버퍼에 채워져 있는 데이터 길이
 */
size_t ringbuffer_size(ringbuffer_t *rb);

/**
 * @brief 버퍼에 남아있는 여유공간 길이 반환
 * 
 * @param rb 링버퍼 핸들
 * @return size_t 버퍼에 남아있는 여유공간 길이
 */
size_t ringbuffer_free_size(ringbuffer_t *rb);

/**
 * @brief 링버퍼 다중 바이트 쓰기
 * 
 * @param rb 링버퍼 핸들
 * @param data 데이터 버퍼
 * @param len 데이터 길이
 */
void ringbuffer_write(ringbuffer_t *rb, const char *data, size_t len);

/**
 * @brief 링버퍼 1바이트 쓰기
 * 
 * @param rb 링버퍼 핸들
 * @param data 데이터
 */
void ringbuffer_write_byte(ringbuffer_t *rb, char data);

/**
 * @brief 링버퍼 다중 바이트 읽기
 * 
 * @param rb 링버퍼 핸들
 * @param data 데이터 버퍼
 * @param len 읽을 바이트 수
 * @return size_t 읽은 바이트 수
 */
size_t ringbuffer_read(ringbuffer_t *rb, char *data, size_t len);

/**
 * @brief 링버퍼 1바이트 읽기
 * 
 * @param rb 링버퍼 핸들
 * @param data 데이터 버퍼
 * @return true 읽기 성공
 * @return false 읽기 실패
 */
bool ringbuffer_read_byte(ringbuffer_t *rb, char *data);

bool ringbuffer_peek(ringbuffer_t *rb, char *data, size_t len, size_t offset);

/**
 * @brief 링버퍼 오래된 데이터 1바이트 읽기
 * 
 * @param rb 링버퍼 핸들
 * @param data 데이터 버퍼
 * @return true 읽기 성공
 * @return false 읽기 실패
 */
bool ringbuffer_peek_byte(ringbuffer_t *rb, char *data);

bool ringbuffer_advance(ringbuffer_t *rb, size_t len);

#endif