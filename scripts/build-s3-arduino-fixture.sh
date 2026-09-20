#!/usr/bin/env bash
# Build one or more pinned Arduino-ESP32 S3 fixtures outside the source tree.
# A content fingerprint bypasses Arduino CLI completely for unchanged inputs;
# cache misses still share Arduino's compiled-core cache.
set -euo pipefail

usage() {
    cat <<'EOF'
usage: build-s3-arduino-fixture.sh [--check] [--rebuild] [--verbose] NAME...

Names: adc, i2c-slave, i2c-wire, ledc, psram-opi, rmt-rx, spi-master, all

Environment:
  ARDUINO_CLI                         Arduino CLI executable
  FLEXE_ARDUINO_CONFIG                optional Arduino CLI config file
  FLEXE_S3_ARDUINO_BUILD_ROOT         persistent external artifact root
  FLEXE_S3_ARDUINO_CACHE_PATH         shared compiled-core cache
  FLEXE_FIXTURE_GATE_JOBS             concurrent read-only gates (default: host-aware)
  FLEXE_S3_ARDUINO_ALLOW_UNPINNED     1 permits a core other than 3.3.11
  FLEXE_BUILD_DIR                     host CMake build used by --check
  S3_ROM_ELF                          official S3 ROM ELF (auto-detected)
  RUNNER                              custom runner for a single --check gate
  SOURCE_DATE_EPOCH                   explicit artifact timestamp
EOF
}

run_checks=0
force_rebuild=0
verbose=0
while [[ $# -gt 0 ]]; do
    case "$1" in
    --check)
        run_checks=1
        shift
        ;;
    --rebuild)
        force_rebuild=1
        shift
        ;;
    --verbose)
        verbose=1
        shift
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    --)
        shift
        break
        ;;
    -*)
        echo "error: unknown option: $1" >&2
        usage >&2
        exit 2
        ;;
    *)
        break
        ;;
    esac
done
[[ $# -gt 0 ]] || { usage >&2; exit 2; }

all_fixtures=(adc i2c-slave i2c-wire ledc psram-opi rmt-rx spi-master)
if [[ $# -eq 1 && "$1" == all ]]; then
    set -- "${all_fixtures[@]}"
elif [[ " $* " == *" all "* ]]; then
    echo "error: 'all' cannot be combined with fixture names" >&2
    exit 2
fi

repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
. "$repo/scripts/fixture-gate-pool.sh"
gate_jobs=
if [[ "$run_checks" -eq 1 ]]; then
    gate_jobs=$(flexe_fixture_gate_jobs) || exit $?
fi
if [[ -n "${XDG_CACHE_HOME:-}" ]]; then
    cache_home=$XDG_CACHE_HOME
elif [[ -n "${HOME:-}" ]]; then
    cache_home=$HOME/.cache
else
    cache_home=${TMPDIR:-/tmp}/flexe-cache
fi

canonicalize_dir_target() {
    local candidate=${1%/} suffix= part
    [[ -n "$candidate" ]] || candidate=/
    [[ "$candidate" == /* ]] || candidate="$PWD/$candidate"
    while [[ ! -e "$candidate" ]]; do
        part=${candidate##*/}
        suffix="/$part$suffix"
        candidate=${candidate%/*}
        [[ -n "$candidate" ]] || candidate=/
    done
    [[ -d "$candidate" ]] || return 1
    candidate=$(CDPATH= cd -- "$candidate" && pwd -P)
    printf '%s%s\n' "$candidate" "$suffix"
}

build_root=${FLEXE_S3_ARDUINO_BUILD_ROOT:-"$cache_home/flexe/arduino-s3-fixtures"}
build_root=$(canonicalize_dir_target "$build_root") || {
    echo "error: FLEXE_S3_ARDUINO_BUILD_ROOT has a non-directory parent" >&2
    exit 1
}
case "$build_root/" in
"$repo/"*)
    echo "error: FLEXE_S3_ARDUINO_BUILD_ROOT must stay outside the repository" >&2
    exit 1
    ;;
esac
mkdir -p -- "$build_root"
build_root=$(CDPATH= cd -- "$build_root" && pwd -P)

core_cache=${FLEXE_S3_ARDUINO_CACHE_PATH:-"$cache_home/flexe/arduino-core"}
core_cache=$(canonicalize_dir_target "$core_cache") || {
    echo "error: FLEXE_S3_ARDUINO_CACHE_PATH has a non-directory parent" >&2
    exit 1
}
case "$core_cache/" in
"$repo/"*)
    echo "error: FLEXE_S3_ARDUINO_CACHE_PATH must stay outside the repository" >&2
    exit 1
    ;;
esac
mkdir -p -- "$core_cache"
core_cache=$(CDPATH= cd -- "$core_cache" && pwd -P)

arduino_cli=${ARDUINO_CLI:-arduino-cli}
arduino_executable=$(command -v "$arduino_cli" 2>/dev/null || true)
[[ -n "$arduino_executable" && -x "$arduino_executable" ]] || {
    echo "error: Arduino CLI is not executable: $arduino_cli" >&2
    exit 1
}
arduino=("$arduino_executable")
arduino_config=${FLEXE_ARDUINO_CONFIG:-}
if [[ -n "$arduino_config" ]]; then
    [[ -f "$arduino_config" ]] || {
        echo "error: Arduino CLI config does not exist: $arduino_config" >&2
        exit 1
    }
    arduino+=(--config-file "$arduino_config")
fi

arduino_version=$("${arduino[@]}" version)
arduino_core_list=$("${arduino[@]}" core list)
core_version=$(awk '$1 == "esp32:esp32" { print $2; exit }' <<<"$arduino_core_list")
expected_core=3.3.11
allow_unpinned=${FLEXE_S3_ARDUINO_ALLOW_UNPINNED:-0}
case "$allow_unpinned" in
0|1) ;;
*)
    echo "error: FLEXE_S3_ARDUINO_ALLOW_UNPINNED must be 0 or 1" >&2
    exit 2
    ;;
esac
if [[ -z "$core_version" ]]; then
    echo "error: Arduino core esp32:esp32 is not installed" >&2
    exit 1
fi
if [[ "$allow_unpinned" != 1 && "$core_version" != "$expected_core" ]]; then
    echo "error: Arduino core esp32:esp32 must be $expected_core (found $core_version)" >&2
    echo "       set FLEXE_S3_ARDUINO_ALLOW_UNPINNED=1 for an intentional upgrade" >&2
    exit 1
fi

arduino_executable_hash=$(openssl dgst -sha256 "$arduino_executable" |
    awk '{print $NF}')
arduino_config_dump=$("${arduino[@]}" config dump)
arduino_config_hash=$(printf '%s' "$arduino_config_dump" |
    openssl dgst -sha256 | awk '{print $NF}')
requested_epoch=${SOURCE_DATE_EPOCH:-}
if [[ -n "$requested_epoch" && ! "$requested_epoch" =~ ^[0-9]+$ ]]; then
    echo "error: SOURCE_DATE_EPOCH must be an integer" >&2
    exit 2
fi

names=()
projects=()
fqbns=()
prefixes=()
host_targets=()
sketches=()
for requested in "$@"; do
    normalized=$(printf '%s' "$requested" | tr '-' '_')
    case "$normalized" in
    adc|s3_adc)
        name=adc
        project="$repo/tests/fixtures/s3_adc"
        prefix=S3_ADC
        host_target=xtensa-emu
        fqbn=esp32:esp32:esp32s3
        ;;
    i2c_slave|s3_i2c_slave)
        name=i2c-slave
        project="$repo/tests/fixtures/i2c_slave"
        prefix=S3_I2C_SLAVE
        host_target=flexe-i2c-slave-test
        fqbn=esp32:esp32:esp32s3
        ;;
    i2c_wire|s3_i2c_wire)
        name=i2c-wire
        project="$repo/tests/fixtures/i2c_wire"
        prefix=S3_I2C
        host_target=flexe-i2c-wire-test
        fqbn=esp32:esp32:esp32s3
        ;;
    ledc|s3_ledc)
        name=ledc
        project="$repo/tests/fixtures/s3_ledc"
        prefix=S3_LEDC
        host_target=xtensa-emu
        fqbn=esp32:esp32:esp32s3
        ;;
    psram_opi|s3_psram_opi)
        name=psram-opi
        project="$repo/tests/fixtures/s3_psram_opi"
        prefix=S3_PSRAM
        host_target=xtensa-emu
        fqbn=esp32:esp32:esp32s3:FlashSize=8M,PSRAM=opi
        ;;
    rmt_rx|s3_rmt_rx)
        name=rmt-rx
        project="$repo/tests/fixtures/s3_rmt_rx"
        prefix=S3_RMT_RX
        host_target=flexe-s3-rmt-rx-test
        fqbn=esp32:esp32:esp32s3
        ;;
    spi_master|s3_spi_master)
        name=spi-master
        project="$repo/tests/fixtures/spi_master"
        prefix=S3_SPI
        host_target=flexe-spi-master-test
        fqbn=esp32:esp32:esp32s3
        ;;
    *)
        echo "error: unknown S3 Arduino fixture: $requested" >&2
        usage >&2
        exit 2
        ;;
    esac
    case " ${names[*]-} " in
    *" $name "*)
        echo "error: duplicate fixture: $requested" >&2
        exit 2
        ;;
    esac
    [[ -f "$project/$(basename "$project").ino" ]] || {
        echo "error: fixture has no matching sketch: $project" >&2
        exit 1
    }
    names+=("$name")
    projects+=("$project")
    fqbns+=("$fqbn")
    prefixes+=("$prefix")
    host_targets+=("$host_target")
    sketches+=("$(basename "$project")")
done

run_logged() {
    local description=$1 log=$2 status
    shift 2
    if [[ "$verbose" -eq 1 ]]; then
        "$@"
        return
    fi
    if "$@" >"$log" 2>&1; then
        return
    else
        status=$?
    fi
    echo "error: $description failed (full log: $log)" >&2
    echo "----- last 200 log lines -----" >&2
    tail -n 200 "$log" >&2
    return "$status"
}

host_build=
if [[ "$run_checks" -eq 1 ]]; then
    if [[ -z "${S3_ROM_ELF:-}" ]]; then
        tools_root=${IDF_TOOLS_PATH:-${HOME:+$HOME/.espressif}}
        rom_candidates=()
        if [[ -n "$tools_root" ]]; then
            shopt -s nullglob
            rom_candidates=(
                "$tools_root"/tools/esp-rom-elfs/*/esp32s3_rev0_rom.elf)
            shopt -u nullglob
        fi
        pinned_rom_hash=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
        for rom_candidate in "${rom_candidates[@]}"; do
            rom_hash=$(openssl dgst -sha256 "$rom_candidate" | awk '{print $NF}')
            if [[ "$rom_hash" == "$pinned_rom_hash" ]]; then
                S3_ROM_ELF=$rom_candidate
                break
            fi
        done
    fi
    [[ -n "${S3_ROM_ELF:-}" && -f "$S3_ROM_ELF" ]] || {
        echo "error: set S3_ROM_ELF to the official ESP32-S3 revision-0 ROM ELF" >&2
        exit 1
    }
    export S3_ROM_ELF
    if [[ -n "${RUNNER:-}" ]]; then
        [[ -x "$RUNNER" ]] || {
            echo "error: emulator runner is not executable: $RUNNER" >&2
            exit 1
        }
    else
        host_build=$(canonicalize_dir_target "${FLEXE_BUILD_DIR:-"$repo/build"}") || {
            echo "error: FLEXE_BUILD_DIR has a non-directory parent" >&2
            exit 1
        }
        if [[ ! -f "$host_build/CMakeCache.txt" ]]; then
            echo "==> configuring host runners"
            run_logged "host runner configuration" "$build_root/host-configure.log" \
                cmake -S "$repo" -B "$host_build"
        fi
        unique_targets=()
        for host_target in "${host_targets[@]}"; do
            case " ${unique_targets[*]-} " in
            *" $host_target "*) ;;
            *) unique_targets+=("$host_target") ;;
            esac
        done
        echo "==> building host runners"
        run_logged "host runner build" "$build_root/host-build.log" \
            cmake --build "$host_build" --target "${unique_targets[@]}" -j
    fi
fi

helper_version=1
for index in "${!names[@]}"; do
    name=${names[$index]}
    project=${projects[$index]}
    fqbn=${fqbns[$index]}
    prefix=${prefixes[$index]}
    host_target=${host_targets[$index]}
    sketch=${sketches[$index]}

    epoch=$requested_epoch
    if [[ -z "$epoch" ]]; then
        epoch=$(git -C "$repo" log -1 --format=%ct -- "$project")
    fi
    [[ "$epoch" =~ ^[0-9]+$ ]] || {
        echo "error: could not derive SOURCE_DATE_EPOCH for $project" >&2
        exit 1
    }

    fingerprint=$({
        printf 'flexe-s3-arduino-helper=%s\n' "$helper_version"
        printf 'fqbn=%s\n' "$fqbn"
        printf 'optimization=-Os\n'
        printf 'source-date-epoch=%s\n' "$epoch"
        printf 'cli=%s\n' "$arduino_version"
        printf 'cli-path=%s\n' "$arduino_executable"
        printf 'cli-sha256=%s\n' "$arduino_executable_hash"
        printf 'config-sha256=%s\n' "$arduino_config_hash"
        printf 'esp32-core=%s\n' "$core_version"
        find "$project" -type f -print | LC_ALL=C sort |
            while IFS= read -r input; do
                relative=${input#"$project"/}
                digest=$(openssl dgst -sha256 "$input" | awk '{print $NF}')
                printf 'source=%s:%s\n' "$relative" "$digest"
            done
    } | openssl dgst -sha256 | awk '{print $NF}')

    output_dir="$build_root/$name"
    firmware="$output_dir/$sketch.ino.merged.bin"
    symbols="$output_dir/$sketch.ino.elf"
    stamp="$output_dir/.flexe-fixture.sha256"
    cached_fingerprint=
    if [[ -f "$stamp" ]]; then
        IFS= read -r cached_fingerprint < "$stamp" || true
    fi

    if [[ "$force_rebuild" -eq 0 && -s "$firmware" && -s "$symbols" &&
          "$cached_fingerprint" == "$fingerprint" ]]; then
        echo "==> reusing unchanged $name firmware"
    else
        source_parent="$build_root/sources"
        source_root="$source_parent/$name-$fingerprint"
        source_copy="$source_root/$sketch"
        if [[ ! -d "$source_copy" ]]; then
            mkdir -p -- "$source_parent"
            source_stage=$(mktemp -d "$source_parent/.$name.XXXXXX")
            if ! cp -R "$project" "$source_stage/$sketch"; then
                rm -rf -- "$source_stage"
                exit 1
            fi
            if ! mv "$source_stage" "$source_root" 2>/dev/null; then
                rm -rf -- "$source_stage"
                [[ -d "$source_copy" ]] || {
                    echo "error: could not stage source for $name" >&2
                    exit 1
                }
            fi
        fi
        mkdir -p -- "$output_dir"
        build_log="$output_dir/.flexe-build.log"
        echo "==> compiling $name"
        compile=("${arduino[@]}" compile --jobs 0 --fqbn "$fqbn"
            --output-dir "$output_dir"
            --build-property compiler.optimization_flags=-Os "$source_copy")
        run_logged "$name compile" "$build_log" env \
            "SOURCE_DATE_EPOCH=$epoch" \
            "ARDUINO_BUILD_CACHE_PATH=$core_cache" \
            ARDUINO_BUILD_CACHE_COMPILATIONS_BEFORE_PURGE=0 \
            "${compile[@]}"
        [[ -s "$firmware" && -s "$symbols" ]] || {
            echo "error: Arduino compile produced no merged image/ELF for $name" >&2
            exit 1
        }
        printf '%s\n' "$fingerprint" > "$stamp.tmp"
        mv "$stamp.tmp" "$stamp"
    fi

    bin_hash=$(openssl dgst -sha256 "$firmware" | awk '{print $NF}')
    elf_hash=$(openssl dgst -sha256 "$symbols" | awk '{print $NF}')
    printf '    %s_BIN=%s\n' "$prefix" "$firmware"
    printf '    %s_ELF=%s\n' "$prefix" "$symbols"
    printf '    %s_BIN_SHA256=%s\n' "$prefix" "$bin_hash"
    printf '    %s_ELF_SHA256=%s\n' "$prefix" "$elf_hash"

    if [[ "$run_checks" -eq 1 ]]; then
        gate="$repo/scripts/check-s3-$name.sh"
        [[ -x "$gate" ]] || {
            echo "error: no executable gate for $name: $gate" >&2
            exit 1
        }
        if [[ -n "${RUNNER:-}" ]]; then
            gate_runner=$RUNNER
        else
            gate_runner="$host_build/$host_target"
        fi
        [[ -x "$gate_runner" ]] || {
            echo "error: built runner is not executable: $gate_runner" >&2
            exit 1
        }
        flexe_fixture_gate_queue_artifact "$name" "$gate" "$prefix" \
            "$firmware" "$symbols" "$bin_hash" "$elf_hash" \
            "$S3_ROM_ELF" "$gate_runner"
    fi
done

if [[ "$run_checks" -eq 1 ]]; then
    gate_status=0
    flexe_fixture_gate_run_queued "$gate_jobs" || gate_status=$?
    [[ "$gate_status" -eq 0 ]] || exit "$gate_status"
fi
