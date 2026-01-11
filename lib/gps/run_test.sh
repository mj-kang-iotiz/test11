#!/bin/bash

# GPS 테스트 빌드 및 실행 스크립트

set -e

echo "================================"
echo "GPS X-Macro Test Build & Run"
echo "================================"
echo ""

# 컴파일러 설정
CC=gcc
CFLAGS="-Wall -Wextra -I. -I../../include -I../ringbuffer -I../../config -DUSE_STORE_RAW_GGA"
TEST_FILE="gps_test.c"
OUTPUT="gps_test"

# 테스트 컴파일
echo "🔨 Compiling test..."
$CC $CFLAGS -o $OUTPUT $TEST_FILE

if [ $? -eq 0 ]; then
    echo "✓ Compilation successful"
    echo ""

    # 테스트 실행
    echo "🚀 Running tests..."
    echo ""
    ./$OUTPUT

    TEST_RESULT=$?
    echo ""

    if [ $TEST_RESULT -eq 0 ]; then
        echo "✅ All tests PASSED!"
    else
        echo "❌ Some tests FAILED!"
        exit 1
    fi

    # 정리
    rm -f $OUTPUT
else
    echo "❌ Compilation failed!"
    exit 1
fi
