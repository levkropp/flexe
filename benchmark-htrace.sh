#!/bin/bash

# Benchmark script to compare performance with/without hierarchical trace

EMU="./build/Release/xtensa-emu.exe"
FIRMWARE="C:/Users/26200.7462/cyd-emulator/test-firmware/10-freertos-minimal/build/freertos-minimal-test.bin"
ELF="C:/Users/26200.7462/cyd-emulator/test-firmware/10-freertos-minimal/build/freertos-minimal-test.elf"

echo "=========================================="
echo "Hierarchical Trace Performance Benchmark"
echo "=========================================="
echo ""

# Test configurations
CYCLES_SHORT=10000000     # 10M cycles
CYCLES_MEDIUM=50000000    # 50M cycles
CYCLES_LONG=100000000     # 100M cycles

echo "Test 1: 10M cycles (short test)"
echo "--------------------------------"

echo -n "WITHOUT hierarchical trace: "
time_output=$( { time $EMU -q -s "$ELF" -c $CYCLES_SHORT "$FIRMWARE" > /dev/null 2>&1; } 2>&1 )
echo "$time_output" | grep real

echo -n "WITH hierarchical trace:    "
time_output=$( { time $EMU -H -q -s "$ELF" -c $CYCLES_SHORT "$FIRMWARE" > /dev/null 2>&1; } 2>&1 )
echo "$time_output" | grep real

echo ""
echo "Test 2: 50M cycles (medium test)"
echo "---------------------------------"

echo -n "WITHOUT hierarchical trace: "
time_output=$( { time $EMU -q -s "$ELF" -c $CYCLES_MEDIUM "$FIRMWARE" > /dev/null 2>&1; } 2>&1 )
echo "$time_output" | grep real

echo -n "WITH hierarchical trace:    "
time_output=$( { time $EMU -H -q -s "$ELF" -c $CYCLES_MEDIUM "$FIRMWARE" > /dev/null 2>&1; } 2>&1 )
echo "$time_output" | grep real

echo ""
echo "Test 3: 100M cycles (long test)"
echo "--------------------------------"

echo -n "WITHOUT hierarchical trace: "
time_output=$( { time $EMU -q -s "$ELF" -c $CYCLES_LONG "$FIRMWARE" > /dev/null 2>&1; } 2>&1 )
echo "$time_output" | grep real

echo -n "WITH hierarchical trace:    "
time_output=$( { time $EMU -H -q -s "$ELF" -c $CYCLES_LONG "$FIRMWARE" > /dev/null 2>&1; } 2>&1 )
echo "$time_output" | grep real

echo ""
echo "=========================================="
echo "Memory usage test"
echo "=========================================="
echo ""

# Run a brief test with -H to show memory statistics
echo "Running with hierarchical trace to measure memory usage..."
$EMU -H -q -s "$ELF" -c 1000000 "$FIRMWARE" 2>&1 | grep -A 20 "HIERARCHICAL TRACE"

echo ""
echo "Memory calculation:"
echo "  Entry size: 24 bytes"
echo "  Entries per level: 65536"
echo "  Levels: 16"
echo "  Total: 24 * 65536 * 16 = 25,165,824 bytes (~24 MB)"
