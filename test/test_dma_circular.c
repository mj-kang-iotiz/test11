#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

// 테스트용 DMA 버퍼 (실제와 동일하게 2048)
#define DMA_BUF_SIZE 2048
static char dma_buf[DMA_BUF_SIZE];
static size_t old_pos = 0;

// 테스트용 출력 버퍼 (ringbuffer 대신)
static char output_buf[8192];
static size_t output_len = 0;

// ringbuffer_write 시뮬레이션
static void ringbuffer_write(const char *data, size_t len) {
    memcpy(&output_buf[output_len], data, len);
    output_len += len;
}

// 테스트용 DMA 위치 (LL_DMA_GetDataLength 시뮬레이션)
static size_t simulated_dma_pos = 0;

static size_t get_dma_pos(void) {
    return simulated_dma_pos;
}

// ========== 원래 방식 (잘못된 방식) ==========
static void process_data_old_way(void) {
    size_t len = get_dma_pos();  // sizeof(buf) - GetDataLength
    if (len > 0) {
        ringbuffer_write(dma_buf, len);  // 항상 0부터!
    }
}

// ========== MaJerle 방식 (올바른 방식) ==========
static void process_data_majerle(void) {
    size_t pos = get_dma_pos();

    if (pos != old_pos) {
        if (pos > old_pos) {
            // 선형: old_pos ~ pos
            ringbuffer_write(&dma_buf[old_pos], pos - old_pos);
        } else {
            // 순환: old_pos ~ 끝, 0 ~ pos
            ringbuffer_write(&dma_buf[old_pos], DMA_BUF_SIZE - old_pos);
            if (pos > 0) {
                ringbuffer_write(dma_buf, pos);
            }
        }
        old_pos = pos;
    }
}

// DMA가 데이터를 수신하는 것을 시뮬레이션
static void simulate_dma_receive(const char *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        dma_buf[simulated_dma_pos] = data[i];
        simulated_dma_pos = (simulated_dma_pos + 1) % DMA_BUF_SIZE;
    }
}

// ========== 테스트 케이스 ==========

void test_linear_case(void) {
    printf("\n=== TEST 1: 선형 케이스 (pos > old_pos) ===\n");

    // 초기화
    memset(dma_buf, 0, sizeof(dma_buf));
    memset(output_buf, 0, sizeof(output_buf));
    old_pos = 0;
    simulated_dma_pos = 0;
    output_len = 0;

    // 100바이트 수신
    char data1[100];
    for (int i = 0; i < 100; i++) data1[i] = 'A';
    simulate_dma_receive(data1, 100);

    // IDLE 인터럽트 발생
    process_data_majerle();

    printf("1차 처리 후: output_len = %zu (expected: 100)\n", output_len);

    // 추가 150바이트 수신
    char data2[150];
    for (int i = 0; i < 150; i++) data2[i] = 'B';
    simulate_dma_receive(data2, 150);

    // 다시 IDLE 인터럽트
    process_data_majerle();

    printf("2차 처리 후: output_len = %zu (expected: 250)\n", output_len);

    // 검증
    int count_A = 0, count_B = 0;
    for (size_t i = 0; i < output_len; i++) {
        if (output_buf[i] == 'A') count_A++;
        if (output_buf[i] == 'B') count_B++;
    }
    printf("A 개수: %d (expected: 100), B 개수: %d (expected: 150)\n", count_A, count_B);
    printf("결과: %s\n", (count_A == 100 && count_B == 150) ? "PASS ✓" : "FAIL ✗");
}

void test_circular_case(void) {
    printf("\n=== TEST 2: 순환 케이스 (pos < old_pos) ===\n");

    // 초기화
    memset(dma_buf, 0, sizeof(dma_buf));
    memset(output_buf, 0, sizeof(output_buf));
    old_pos = 0;
    simulated_dma_pos = 0;
    output_len = 0;

    // DMA 위치를 버퍼 끝 근처로 이동 (1900)
    simulated_dma_pos = 1900;
    old_pos = 1900;

    // 200바이트 수신 (1900~2047 + 0~51 = 순환!)
    char data[200];
    for (int i = 0; i < 200; i++) data[i] = 'C';
    simulate_dma_receive(data, 200);

    printf("DMA 시작: 1900, 200바이트 수신 후 DMA pos: %zu\n", simulated_dma_pos);
    printf("예상: 1900 + 200 = 2100 -> 2100 %% 2048 = 52\n");

    // IDLE 인터럽트
    process_data_majerle();

    printf("처리 후: output_len = %zu (expected: 200)\n", output_len);

    int count_C = 0;
    for (size_t i = 0; i < output_len; i++) {
        if (output_buf[i] == 'C') count_C++;
    }
    printf("C 개수: %d (expected: 200)\n", count_C);
    printf("결과: %s\n", (count_C == 200) ? "PASS ✓" : "FAIL ✗");
}

void test_old_way_duplication(void) {
    printf("\n=== TEST 3: 원래 방식의 중복 문제 ===\n");

    // 초기화
    memset(dma_buf, 0, sizeof(dma_buf));
    memset(output_buf, 0, sizeof(output_buf));
    simulated_dma_pos = 0;
    output_len = 0;

    // 100바이트 수신
    char data1[100];
    for (int i = 0; i < 100; i++) data1[i] = 'X';
    simulate_dma_receive(data1, 100);

    // 원래 방식으로 처리
    process_data_old_way();
    printf("1차 처리: output_len = %zu\n", output_len);

    // 추가 50바이트 수신
    char data2[50];
    for (int i = 0; i < 50; i++) data2[i] = 'Y';
    simulate_dma_receive(data2, 50);

    // 다시 원래 방식으로 처리
    process_data_old_way();
    printf("2차 처리: output_len = %zu (원래 방식: 0부터 다시 복사!)\n", output_len);

    int count_X = 0, count_Y = 0;
    for (size_t i = 0; i < output_len; i++) {
        if (output_buf[i] == 'X') count_X++;
        if (output_buf[i] == 'Y') count_Y++;
    }
    printf("X 개수: %d (expected: 100, 원래방식: 200 - 중복!)\n", count_X);
    printf("Y 개수: %d (expected: 50)\n", count_Y);
    printf("결과: %s (원래 방식은 중복 발생)\n", (count_X == 100) ? "PASS" : "FAIL - 중복 발생!");
}

void test_ht_tc_idle_sequence(void) {
    printf("\n=== TEST 4: HT/TC/IDLE 연속 호출 시뮬레이션 ===\n");

    // 초기화
    memset(dma_buf, 0, sizeof(dma_buf));
    memset(output_buf, 0, sizeof(output_buf));
    old_pos = 0;
    simulated_dma_pos = 0;
    output_len = 0;

    // GPS 데이터 시뮬레이션: $GPGGA,...\r\n 형태
    const char *gps_sentence = "$GPGGA,123456.00,3723.2475,N,12158.3416,W,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    size_t sentence_len = strlen(gps_sentence);

    printf("GPS 문장 길이: %zu bytes\n", sentence_len);

    // 여러 문장 연속 수신 시뮬레이션
    int total_sentences = 30;  // 30개 문장
    for (int i = 0; i < total_sentences; i++) {
        simulate_dma_receive(gps_sentence, sentence_len);

        // 매번 처리 (실제로는 HT/TC/IDLE 중 하나가 트리거)
        process_data_majerle();
    }

    printf("총 수신: %d sentences x %zu bytes = %zu bytes\n",
           total_sentences, sentence_len, total_sentences * sentence_len);
    printf("처리된 데이터: %zu bytes\n", output_len);
    printf("결과: %s\n",
           (output_len == total_sentences * sentence_len) ? "PASS ✓" : "FAIL ✗");

    // 데이터 무결성 확인 (첫 번째 문장)
    printf("\n첫 번째 처리된 문장:\n%.70s\n", output_buf);
}

void test_burst_with_wraparound(void) {
    printf("\n=== TEST 5: 대용량 버스트 + 순환 ===\n");

    // 초기화
    memset(dma_buf, 0, sizeof(dma_buf));
    memset(output_buf, 0, sizeof(output_buf));
    old_pos = 0;
    simulated_dma_pos = 0;
    output_len = 0;

    // 버퍼 크기보다 큰 데이터 수신 (여러 번 순환)
    size_t total_to_send = 5000;  // 2048 * 2 + 904
    char *big_data = malloc(total_to_send);
    for (size_t i = 0; i < total_to_send; i++) {
        big_data[i] = 'D' + (i % 4);  // D, E, F, G 반복
    }

    // 500바이트씩 나눠서 수신 + 처리
    size_t chunk_size = 500;
    size_t sent = 0;
    while (sent < total_to_send) {
        size_t to_send = (total_to_send - sent < chunk_size) ? (total_to_send - sent) : chunk_size;
        simulate_dma_receive(&big_data[sent], to_send);
        process_data_majerle();  // HT 또는 TC 또는 IDLE
        sent += to_send;
    }

    printf("총 전송: %zu bytes\n", total_to_send);
    printf("처리된 데이터: %zu bytes\n", output_len);

    // 데이터 무결성 확인
    int match = 1;
    for (size_t i = 0; i < output_len && i < total_to_send; i++) {
        if (output_buf[i] != ('D' + (i % 4))) {
            match = 0;
            printf("Mismatch at %zu: expected %c, got %c\n", i, 'D' + (i % 4), output_buf[i]);
            break;
        }
    }

    printf("결과: %s\n",
           (output_len == total_to_send && match) ? "PASS ✓" : "FAIL ✗");

    free(big_data);
}

int main(void) {
    printf("========================================\n");
    printf("DMA Circular Buffer 테스트\n");
    printf("========================================\n");

    test_linear_case();
    test_circular_case();
    test_old_way_duplication();
    test_ht_tc_idle_sequence();
    test_burst_with_wraparound();

    printf("\n========================================\n");
    printf("테스트 완료\n");
    printf("========================================\n");

    return 0;
}
