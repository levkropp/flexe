#!/usr/bin/env bash
# Compile real Arduino-ESP32 fixtures and run their host harnesses with both
# execution engines. With no arguments, run the complete hardware gate set.
set -euo pipefail

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
. "$repo_dir/scripts/fixture-gate-pool.sh"

arduino_cli=${ARDUINO_CLI:-arduino-cli}
arduino_config=${FLEXE_ARDUINO_CONFIG:-}
fqbn=${FLEXE_ARDUINO_FQBN:-esp32:esp32:d32:PartitionScheme=min_spiffs}
host_build=${FLEXE_BUILD_DIR:-"$repo_dir/build"}
fixture_build_root=${FLEXE_FIXTURE_BUILD_ROOT:-"$host_build/arduino-fixtures"}
fixture_rebuild=${FLEXE_FIXTURE_REBUILD:-0}
engine_jobs=${FLEXE_FIXTURE_ENGINE_JOBS:-2}
build_jobs=$(flexe_fixture_build_jobs) || exit $?
gate_jobs=$(flexe_fixture_gate_jobs) || exit $?
processors=$(flexe_fixture_processor_count)

case "$fixture_rebuild" in
0|1) ;;
*)
    echo "error: FLEXE_FIXTURE_REBUILD must be 0 or 1" >&2
    exit 2
    ;;
esac
case "$engine_jobs" in
1|2) ;;
*)
    echo "error: FLEXE_FIXTURE_ENGINE_JOBS must be 1 or 2" >&2
    exit 2
    ;;
esac

all_fixtures=(
    analog_dac
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
    wifi_client
)

usage() {
    echo "usage: $0 [all | FIXTURE ...]" >&2
    echo "fixtures:" >&2
    printf '  %s\n' "${all_fixtures[@]}" >&2
}

case "${1:-}" in
-h|--help) usage; exit 0 ;;
esac
if [[ $# -eq 0 || ( $# -eq 1 && "$1" == all ) ]]; then
    set -- "${all_fixtures[@]}"
elif [[ " $* " == *" all "* ]]; then
    echo "error: 'all' cannot be combined with fixture names" >&2
    exit 2
fi

fixtures=()
slugs=()
driver_modes=()
for requested in "$@"; do
    fixture=${requested//-/_}
    known_fixture=0
    for candidate in "${all_fixtures[@]}"; do
        if [[ "$fixture" == "$candidate" ]]; then
            known_fixture=1
            break
        fi
    done
    if [[ "$known_fixture" -ne 1 ]]; then
        echo "error: unknown fixture: $requested" >&2
        usage
        exit 2
    fi
    for candidate in "${fixtures[@]-}"; do
        if [[ "$fixture" == "$candidate" ]]; then
            echo "error: duplicate fixture: $requested" >&2
            exit 2
        fi
    done
    slug=${fixture//_/-}
    driver_mode=0
    if [[ "$fixture" == emac_driver ]]; then
        driver_mode=1
    fi
    fixtures+=("$fixture")
    slugs+=("$slug")
    driver_modes+=("$driver_mode")
done

echo "==> building requested host fixture runners"
host_build_log="$host_build/.flexe-fixture-build.log"
if ! cmake --build "$host_build" --target flexe-fixture-test -j \
        >"$host_build_log" 2>&1; then
    echo "error: host fixture runner build failed (full log: $host_build_log)" >&2
    tail -n 200 "$host_build_log" >&2
    exit 1
fi

# A no-change Arduino CLI invocation spends tens of seconds walking the core
# and dependency graph. Persist final artifacts behind a complete content
# fingerprint and keep the compiled core in one explicit cache on every host.
arduino_executable=$(command -v "$arduino_cli" 2>/dev/null || true)
[[ -n "$arduino_executable" && -x "$arduino_executable" ]] || {
    echo "error: Arduino CLI is not executable: $arduino_cli" >&2
    exit 1
}
arduino=("$arduino_executable")
if [[ -n "$arduino_config" ]]; then
    [[ -f "$arduino_config" ]] || {
        echo "error: Arduino CLI config does not exist: $arduino_config" >&2
        exit 1
    }
    arduino+=(--config-file "$arduino_config")
fi
arduino_version=$("${arduino[@]}" version)
arduino_cores=$("${arduino[@]}" core list |
    awk 'NR > 1 { print $1 "=" $2 }')
arduino_executable_hash=$(openssl dgst -sha256 "$arduino_executable" |
    awk '{ print $NF }')
arduino_config_hash=default
if [[ -n "$arduino_config" ]]; then
    arduino_config_hash=$(openssl dgst -sha256 "$arduino_config" |
        awk '{ print $NF }')
fi

if [[ -n "${XDG_CACHE_HOME:-}" ]]; then
    cache_home=$XDG_CACHE_HOME
elif [[ -n "${HOME:-}" ]]; then
    cache_home=$HOME/.cache
else
    cache_home=${TMPDIR:-/tmp}/flexe-cache
fi
core_cache=${ARDUINO_BUILD_CACHE_PATH:-"$cache_home/flexe/arduino-classic-core"}
mkdir -p -- "$core_cache"
core_cache=$(CDPATH= cd -- "$core_cache" && pwd -P)
mkdir -p -- "$core_cache/cores"
core_fingerprint=$({
    printf 'fqbn=%s\n' "$fqbn"
    printf 'optimization=%s\n' '-Os'
    printf 'cli=%s\n' "$arduino_version"
    printf 'cli-sha256=%s\n' "$arduino_executable_hash"
    printf 'cores=%s\n' "$arduino_cores"
    printf 'config-sha256=%s\n' "$arduino_config_hash"
} | openssl dgst -sha256 | awk '{ print $NF }')
core_stamp="$core_cache/.flexe-core-$core_fingerprint"
core_ready=0
if [[ -f "$core_stamp" ]] &&
   find "$core_cache/cores" -name core.a -type f -size +0c \
       -print -quit | grep -q .; then
    core_ready=1
fi

temporary_build_root=
if [[ "$fixture_build_root" == temporary ]]; then
    temporary_build_root=$(mktemp -d \
        "${TMPDIR:-/tmp}/flexe-fixtures.XXXXXX")
    fixture_build_root=$temporary_build_root
fi
mkdir -p -- "$fixture_build_root"

cleanup() {
    flexe_fixture_gate_pool_abort
    if [[ -n "$temporary_build_root" ]]; then
        rm -rf -- "$temporary_build_root"
        temporary_build_root=
    fi
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

fixture_fingerprint() {
    local fixture_dir=$1 input relative digest
    {
        echo 'flexe-arduino-fixture-v1'
        printf 'fqbn=%s\n' "$fqbn"
        printf 'optimization=%s\n' '-Os'
        printf 'cli=%s\n' "$arduino_version"
        printf 'cli-path=%s\n' "$arduino_executable"
        printf 'cli-sha256=%s\n' "$arduino_executable_hash"
        printf 'cores=%s\n' "$arduino_cores"
        printf 'config-sha256=%s\n' "$arduino_config_hash"
        find "$fixture_dir" -path "$fixture_dir/build" -prune -o \
            -type f -print | LC_ALL=C sort |
            while IFS= read -r input; do
                relative=${input#"$fixture_dir"/}
                digest=$(openssl dgst -sha256 "$input" |
                    awk '{ print $NF }')
                printf 'source=%s:%s\n' "$relative" "$digest"
            done
    } | openssl dgst -sha256 | awk '{ print $NF }'
}

compile_cleanup() {
    if [[ -n "${COMPILE_SOURCE_ROOT:-}" ]]; then
        rm -rf -- "$COMPILE_SOURCE_ROOT"
        COMPILE_SOURCE_ROOT=
    fi
    if [[ -n "${COMPILE_CACHE_ROOT:-}" ]]; then
        rm -rf -- "$COMPILE_CACHE_ROOT"
        COMPILE_CACHE_ROOT=
    fi
}

compile_fixture() {
    local fixture=$1 slug=$2 project=$3 output_dir=$4 firmware=$5
    local symbols=$6 stamp=$7 fingerprint=$8 jobs=$9 source status
    COMPILE_SOURCE_ROOT=$(mktemp -d \
        "${TMPDIR:-/tmp}/flexe-$slug-source.XXXXXX") || return
    COMPILE_CACHE_ROOT=$(mktemp -d \
        "${TMPDIR:-/tmp}/flexe-$slug-cache.XXXXXX") || {
        compile_cleanup
        return 1
    }
    trap compile_cleanup EXIT
    trap 'exit 129' HUP
    trap 'exit 130' INT
    trap 'exit 143' TERM
    source="$COMPILE_SOURCE_ROOT/$fixture"
    cp -R "$project" "$source" || return
    mkdir -p -- "$output_dir" || return
    # Arduino CLI stores each sketch's full intermediate tree next to its
    # reusable core archive. Point a disposable cache at the shared cores so
    # a 20+ MiB sketch tree is not retained for every final 250 KiB fixture.
    ln -s "$core_cache/cores" "$COMPILE_CACHE_ROOT/cores" || return
    status=0
    env "ARDUINO_BUILD_CACHE_PATH=$COMPILE_CACHE_ROOT" \
        ARDUINO_BUILD_CACHE_COMPILATIONS_BEFORE_PURGE=0 \
        "${arduino[@]}" compile --jobs "$jobs" --fqbn "$fqbn" \
        --output-dir "$output_dir" \
        --build-property compiler.optimization_flags=-Os \
        "$source" || status=$?
    [[ "$status" -eq 0 ]] || return "$status"
    if [[ ! -s "$firmware" || ! -s "$symbols" ]]; then
        echo "error: Arduino compile produced no firmware for $slug" >&2
        return 1
    fi
    # The harness consumes only the application image and ELF. Arduino also
    # copies a roughly 11 MiB map, a merged flash image, and boot/partition
    # images into --output-dir; retaining those for every classic fixture used
    # hundreds of MiB without making a subsequent check any faster.
    rm -f -- "$output_dir/$fixture.ino.map" \
        "$output_dir/$fixture.ino.merged.bin" \
        "$output_dir/$fixture.ino.bootloader.bin" \
        "$output_dir/$fixture.ino.partitions.bin"
    printf '%s\n' "$fingerprint" > "$stamp.tmp" || return
    mv "$stamp.tmp" "$stamp" || return
    compile_cleanup
    trap - EXIT HUP INT TERM
}

projects=()
output_dirs=()
firmwares=()
symbols_files=()
stamps=()
fingerprints=()
runner_paths=()
stale_indices=()
for index in "${!fixtures[@]}"; do
    fixture=${fixtures[$index]}
    slug=${slugs[$index]}
    project="$repo_dir/tests/fixtures/$fixture"
    [[ -f "$project/$fixture.ino" ]] || {
        echo "error: fixture has no matching sketch: $project" >&2
        exit 1
    }
    output_dir="$fixture_build_root/$fixture"
    firmware="$output_dir/$fixture.ino.bin"
    symbols="$output_dir/$fixture.ino.elf"
    stamp="$output_dir/.flexe-fixture.sha256"
    fingerprint=$(fixture_fingerprint "$project")
    cached_fingerprint=
    if [[ -f "$stamp" ]]; then
        IFS= read -r cached_fingerprint < "$stamp" || true
    fi
    projects+=("$project")
    output_dirs+=("$output_dir")
    firmwares+=("$firmware")
    symbols_files+=("$symbols")
    stamps+=("$stamp")
    fingerprints+=("$fingerprint")
    runner_paths+=("$host_build/flexe-fixture-test")
    if [[ "$fixture_rebuild" -eq 0 && -s "$firmware" &&
          -s "$symbols" && "$cached_fingerprint" == "$fingerprint" ]]; then
        echo "==> reusing unchanged $slug firmware"
    else
        stale_indices+=("$index")
    fi
done

run_build_pool() {
    local outer_jobs=$1 inner_jobs=$2 status index
    shift 2
    local indices=("$@")
    [[ ${#indices[@]} -ne 0 ]] || return 0
    FLEXE_FIXTURE_POOL_VERB=compiling
    flexe_fixture_gate_pool_init "$outer_jobs" "${#indices[@]}"
    for index in "${indices[@]}"; do
        flexe_fixture_gate_pool_submit "${slugs[$index]}" \
            compile_fixture "${fixtures[$index]}" "${slugs[$index]}" \
            "${projects[$index]}" "${output_dirs[$index]}" \
            "${firmwares[$index]}" "${symbols_files[$index]}" \
            "${stamps[$index]}" "${fingerprints[$index]}" "$inner_jobs"
    done
    status=0
    flexe_fixture_gate_pool_wait || status=$?
    return "$status"
}

if [[ ${#stale_indices[@]} -ne 0 ]]; then
    # Prime the shared core once before concurrent cache misses. That keeps
    # cold builds from racing to populate the same core archive, after which
    # independent sketches can compile and link in parallel safely.
    remaining_indices=("${stale_indices[@]}")
    if [[ "$core_ready" -eq 0 ]]; then
        primer_jobs=$processors
        [[ "$primer_jobs" -le 8 ]] || primer_jobs=8
        run_build_pool 1 "$primer_jobs" "${remaining_indices[0]}"
        printf '%s\n' "$core_fingerprint" > "$core_stamp.tmp"
        mv "$core_stamp.tmp" "$core_stamp"
        remaining_indices=("${remaining_indices[@]:1}")
    fi
    if [[ ${#remaining_indices[@]} -ne 0 ]]; then
        actual_build_jobs=$build_jobs
        [[ "$actual_build_jobs" -le ${#remaining_indices[@]} ]] ||
            actual_build_jobs=${#remaining_indices[@]}
        compile_jobs=$((processors / actual_build_jobs))
        [[ "$compile_jobs" -ge 1 ]] || compile_jobs=1
        [[ "$compile_jobs" -le 8 ]] || compile_jobs=8
        run_build_pool "$actual_build_jobs" "$compile_jobs" \
            "${remaining_indices[@]}"
    fi
fi

for index in "${!fixtures[@]}"; do
    # Also migrate cache entries produced by older helper versions without
    # forcing a firmware rebuild.
    rm -f -- "${output_dirs[$index]}/${fixtures[$index]}.ino.map" \
        "${output_dirs[$index]}/${fixtures[$index]}.ino.merged.bin" \
        "${output_dirs[$index]}/${fixtures[$index]}.ino.bootloader.bin" \
        "${output_dirs[$index]}/${fixtures[$index]}.ino.partitions.bin"
    [[ -s "${firmwares[$index]}" && -s "${symbols_files[$index]}" ]] || {
        echo "error: missing compiled artifacts for ${slugs[$index]}" >&2
        exit 1
    }
    [[ -x "${runner_paths[$index]}" ]] || {
        echo "error: built runner is not executable: ${runner_paths[$index]}" >&2
        exit 1
    }
done

run_fixture_engine() {
    local engine=$1 runner=$2 entry=$3 firmware=$4 symbols=$5 driver_mode=$6
    if [[ "$engine" == interpreter ]]; then
        if [[ "$driver_mode" -eq 1 ]]; then
            "$runner" "$entry" --no-jit --driver "$firmware" "$symbols"
        else
            "$runner" "$entry" --no-jit "$firmware" "$symbols"
        fi
    elif [[ "$driver_mode" -eq 1 ]]; then
        "$runner" "$entry" --driver "$firmware" "$symbols"
    else
        "$runner" "$entry" "$firmware" "$symbols"
    fi
}

pair_cleanup() {
    if [[ -n "${PAIR_JIT_PID:-}" ]]; then
        flexe_fixture_process_tree_signal TERM "$PAIR_JIT_PID"
        wait "$PAIR_JIT_PID" 2>/dev/null || true
        PAIR_JIT_PID=
    fi
    if [[ -n "${PAIR_INTERP_PID:-}" ]]; then
        flexe_fixture_process_tree_signal TERM "$PAIR_INTERP_PID"
        wait "$PAIR_INTERP_PID" 2>/dev/null || true
        PAIR_INTERP_PID=
    fi
    if [[ -n "${PAIR_LOG_DIR:-}" ]]; then
        rm -f -- "$PAIR_LOG_DIR/jit.log" "$PAIR_LOG_DIR/interpreter.log"
        rmdir "$PAIR_LOG_DIR" 2>/dev/null || true
        PAIR_LOG_DIR=
    fi
}

run_fixture_pair() {
    local slug=$1 runner=$2 firmware=$3 symbols=$4 driver_mode=$5
    local pair_engine_jobs=$6 status=0 jit_status=0 interpreter_status=0
    if [[ "$pair_engine_jobs" -eq 1 ]]; then
        echo "==> testing $slug (JIT)"
        run_fixture_engine jit "$runner" "$slug" "$firmware" "$symbols" \
            "$driver_mode" || status=1
        echo "==> testing $slug (interpreter)"
        run_fixture_engine interpreter "$runner" "$slug" "$firmware" "$symbols" \
            "$driver_mode" || status=1
        return "$status"
    fi

    PAIR_LOG_DIR=$(mktemp -d \
        "${TMPDIR:-/tmp}/flexe-$slug-engines.XXXXXX") || return
    PAIR_JIT_PID=
    PAIR_INTERP_PID=
    trap pair_cleanup EXIT
    trap 'exit 129' HUP
    trap 'exit 130' INT
    trap 'exit 143' TERM
    run_fixture_engine jit "$runner" "$slug" "$firmware" "$symbols" \
        "$driver_mode" >"$PAIR_LOG_DIR/jit.log" 2>&1 &
    PAIR_JIT_PID=$!
    run_fixture_engine interpreter "$runner" "$slug" "$firmware" "$symbols" \
        "$driver_mode" >"$PAIR_LOG_DIR/interpreter.log" 2>&1 &
    PAIR_INTERP_PID=$!
    wait "$PAIR_JIT_PID" || jit_status=$?
    PAIR_JIT_PID=
    wait "$PAIR_INTERP_PID" || interpreter_status=$?
    PAIR_INTERP_PID=
    echo "==> testing $slug (JIT)"
    cat "$PAIR_LOG_DIR/jit.log"
    echo "==> testing $slug (interpreter)"
    cat "$PAIR_LOG_DIR/interpreter.log"
    if [[ "$jit_status" -ne 0 || "$interpreter_status" -ne 0 ]]; then
        status=1
    fi
    pair_cleanup
    trap - EXIT HUP INT TERM
    return "$status"
}

FLEXE_FIXTURE_POOL_VERB=testing
flexe_fixture_gate_pool_init "$gate_jobs" "${#fixtures[@]}"
for index in "${!fixtures[@]}"; do
    flexe_fixture_gate_pool_submit "${slugs[$index]}" \
        run_fixture_pair "${slugs[$index]}" "${runner_paths[$index]}" \
        "${firmwares[$index]}" "${symbols_files[$index]}" \
        "${driver_modes[$index]}" "$engine_jobs"
done
status=0
flexe_fixture_gate_pool_wait || status=$?
exit "$status"
