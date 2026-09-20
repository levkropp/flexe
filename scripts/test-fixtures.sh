#!/bin/sh
# Compile real Arduino-ESP32 fixtures and run their host harnesses with both
# execution engines. With no arguments, run the complete hardware gate set.
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
arduino_cli=${ARDUINO_CLI:-arduino-cli}
arduino_config=${FLEXE_ARDUINO_CONFIG:-}
fqbn=${FLEXE_ARDUINO_FQBN:-esp32:esp32:d32:PartitionScheme=min_spiffs}
host_build=${FLEXE_BUILD_DIR:-"$repo_dir/build"}
fixture_build_root=${FLEXE_FIXTURE_BUILD_ROOT:-"$host_build/arduino-fixtures"}
fixture_rebuild=${FLEXE_FIXTURE_REBUILD:-0}
if [ "$fixture_build_root" = temporary ]; then
    fixture_build_root=
fi
case "$fixture_rebuild" in
    0|1) ;;
    *)
        echo "error: FLEXE_FIXTURE_REBUILD must be 0 or 1" >&2
        exit 2
        ;;
esac

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

# Validate the complete request before doing work, then let the build tool
# compile all required host runners in one parallel dependency graph. Calling
# CMake once per fixture serialized expensive links, especially under LTO or
# sanitizers. The names below come only from all_fixtures, so intentional word
# splitting is safe.
host_targets=
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
    if [ "$fixture" = emac_driver ]; then
        target=flexe-emac-hal-test
    fi
    case " $host_targets " in
        *" $target "*) ;;
        *) host_targets="$host_targets $target" ;;
    esac
done

echo "==> building requested host fixture runners"
# shellcheck disable=SC2086 -- target names were validated above.
cmake --build "$host_build" --target $host_targets -j

# A no-change arduino-cli invocation still spends tens of seconds walking the
# core and dependency graph on macOS. Persisted build paths let us skip that
# process entirely when every input that selects the firmware is identical.
# The installed core version is sufficient for packaged cores; a custom CLI
# wrapper and an explicit config file are hashed as inputs too.
arduino_executable=$(command -v "$arduino_cli" 2>/dev/null || true)
arduino_version=$("$arduino_cli" version)
if [ -n "$arduino_config" ]; then
    arduino_cores=$("$arduino_cli" --config-file "$arduino_config" core list |
        awk 'NR > 1 { print $1 "=" $2 }')
else
    arduino_cores=$("$arduino_cli" core list |
        awk 'NR > 1 { print $1 "=" $2 }')
fi
arduino_executable_hash=missing
if [ -n "$arduino_executable" ] && [ -f "$arduino_executable" ]; then
    arduino_executable_hash=$(
        openssl dgst -sha256 "$arduino_executable" | awk '{ print $NF }')
fi
arduino_config_hash=default
if [ -n "$arduino_config" ]; then
    arduino_config_hash=$(
        openssl dgst -sha256 "$arduino_config" | awk '{ print $NF }')
fi

fixture_fingerprint() {
    fixture_dir=$1
    {
        echo 'flexe-arduino-fixture-v1'
        printf 'fqbn=%s\n' "$fqbn"
        printf 'optimization=%s\n' '-Os'
        printf 'cli=%s\n' "$arduino_version"
        printf 'cli-path=%s\n' "$arduino_executable"
        printf 'cli-sha256=%s\n' "$arduino_executable_hash"
        printf 'cores=%s\n' "$arduino_cores"
        printf 'config-sha256=%s\n' "$arduino_config_hash"
        find "$fixture_dir" -type f -print | LC_ALL=C sort |
            while IFS= read -r input; do
                relative=${input#"$fixture_dir"/}
                digest=$(openssl dgst -sha256 "$input" |
                    awk '{ print $NF }')
                printf 'source=%s:%s\n' "$relative" "$digest"
            done
    } | openssl dgst -sha256 | awk '{ print $NF }'
}

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

    firmware="$fixture_build/$fixture.ino.bin"
    symbols="$fixture_build/$fixture.ino.elf"
    stamp="$fixture_build/.flexe-fixture.sha256"
    fingerprint=$(fixture_fingerprint "$repo_dir/tests/fixtures/$fixture")
    cached_fingerprint=
    if [ -f "$stamp" ]; then
        IFS= read -r cached_fingerprint < "$stamp" || true
    fi

    if [ "$fixture_rebuild" -eq 0 ] && [ -s "$firmware" ] &&
       [ -s "$symbols" ] && [ "$cached_fingerprint" = "$fingerprint" ]; then
        echo "==> reusing unchanged $slug firmware"
    else
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
        if [ ! -s "$firmware" ] || [ ! -s "$symbols" ]; then
            echo "error: Arduino compile produced no firmware for $slug" >&2
            status=1
            cleanup
            continue
        fi
        printf '%s\n' "$fingerprint" > "$stamp.tmp"
        mv "$stamp.tmp" "$stamp"
    fi

    runner="$host_build/$target"

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
