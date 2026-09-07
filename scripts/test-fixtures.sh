#!/bin/sh
# Compile real Arduino-ESP32 fixtures and run their host harnesses with both
# execution engines. With no arguments, run the complete hardware gate set.
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
arduino_cli=${ARDUINO_CLI:-arduino-cli}
arduino_config=${FLEXE_ARDUINO_CONFIG:-}
fqbn=${FLEXE_ARDUINO_FQBN:-esp32:esp32:d32:PartitionScheme=min_spiffs}
host_build=${FLEXE_BUILD_DIR:-"$repo_dir/build"}
fixture_build_root=${FLEXE_FIXTURE_BUILD_ROOT:-}

all_fixtures='analog_dac
deep_sleep
emac_driver
emac_hal
esp_timer
event_group
frc_timer
frt_timer
gpio_isr
guest_clock
i2c_slave
i2c_wire
i2s_dma
ledc_pwm
mcpwm_motor
pcnt_pulse
rmt_tx
rtc_gpio
rtc_i2c
scheduler
sdio_slave
sdmmc_host
sigmadelta
spi_master
task_queue
task_wdt
timer_group
touch_pad
twai_bus
uhci_dma
wifi_client'

usage() {
    echo "usage: $0 [all | FIXTURE ...]" >&2
    echo "fixtures:" >&2
    printf '  %s\n' $all_fixtures >&2
}

case "${1:-}" in
    -h|--help) usage; exit 0 ;;
esac

if [ "$#" -eq 0 ] || { [ "$#" -eq 1 ] && [ "$1" = all ]; }; then
    set -- $all_fixtures
fi

fixture_build=
remove_fixture_build=0
cleanup() {
    if [ "$remove_fixture_build" -eq 1 ] && [ -n "$fixture_build" ]; then
        rm -rf -- "$fixture_build"
    fi
    fixture_build=
    remove_fixture_build=0
}
trap cleanup EXIT HUP INT TERM

status=0
for requested in "$@"; do
    fixture=$(printf '%s\n' "$requested" | tr '-' '_')
    known_fixture=0
    for candidate in $all_fixtures; do
        if [ "$fixture" = "$candidate" ]; then
            known_fixture=1
            break
        fi
    done
    if [ "$known_fixture" -ne 1 ]; then
        echo "error: unknown fixture: $requested" >&2
        usage
        exit 2
    fi

    slug=$(printf '%s\n' "$fixture" | tr '_' '-')
    target="flexe-$slug-test"
    driver_mode=0
    if [ "$fixture" = emac_driver ]; then
        target=flexe-emac-hal-test
        driver_mode=1
    fi

    if [ -n "$fixture_build_root" ]; then
        fixture_build="$fixture_build_root/$fixture"
        mkdir -p -- "$fixture_build"
    else
        fixture_build=$(mktemp -d "${TMPDIR:-/tmp}/flexe-$slug.XXXXXX")
        remove_fixture_build=1
    fi

    echo "==> compiling $slug"
    if [ -n "$arduino_config" ]; then
        if ! "$arduino_cli" --config-file "$arduino_config" compile \
                --fqbn "$fqbn" \
                --build-path "$fixture_build" \
                --build-property compiler.optimization_flags=-Os \
                "$repo_dir/tests/fixtures/$fixture"; then
            status=1
            cleanup
            continue
        fi
    else
        if ! "$arduino_cli" compile \
                --fqbn "$fqbn" \
                --build-path "$fixture_build" \
                --build-property compiler.optimization_flags=-Os \
                "$repo_dir/tests/fixtures/$fixture"; then
            status=1
            cleanup
            continue
        fi
    fi

    if ! cmake --build "$host_build" --target "$target" -j; then
        status=1
        cleanup
        continue
    fi
    runner="$host_build/$target"
    firmware="$fixture_build/$fixture.ino.bin"
    symbols="$fixture_build/$fixture.ino.elf"

    echo "==> testing $slug (JIT)"
    if [ "$driver_mode" -eq 1 ]; then
        if ! "$runner" --driver "$firmware" "$symbols"; then status=1; fi
    else
        if ! "$runner" "$firmware" "$symbols"; then status=1; fi
    fi

    echo "==> testing $slug (interpreter)"
    if [ "$driver_mode" -eq 1 ]; then
        if ! "$runner" --no-jit --driver "$firmware" "$symbols"; then
            status=1
        fi
    else
        if ! "$runner" --no-jit "$firmware" "$symbols"; then status=1; fi
    fi

    cleanup
done

exit "$status"
