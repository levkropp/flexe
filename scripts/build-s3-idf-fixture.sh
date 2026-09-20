#!/usr/bin/env bash
# Build one or more pinned ESP-IDF S3 fixtures outside the source tree.
# Persistent build directories handle incremental rebuilds; a shared ccache
# avoids recompiling the same IDF components for every independent project.
set -euo pipefail

usage() {
    cat <<'EOF'
usage: build-s3-idf-fixture.sh [--check] NAME...

Names may be short aliases such as hello, crosscore, nvs, or sleep; an
in-tree s3_idf_* project name; or a path to another ESP-IDF project.

Environment:
  FLEXE_IDF_BUILD_ROOT       persistent external output root
  FLEXE_IDF_CCACHE           auto (default), 1 (required), or 0
  FLEXE_IDF_CCACHE_DIR       shared compiler-cache directory
  FLEXE_IDF_CCACHE_MAXSIZE   cache limit (default: 2G)
  FLEXE_IDF_ALLOW_UNPINNED   1 permits an IDF revision other than v5.3.2
  FLEXE_BUILD_DIR            host CMake build used by --check
  S3_ROM_ELF                 official ROM ELF required by --check
  RUNNER                     custom runner used by every requested check
EOF
}

run_checks=0
while [[ $# -gt 0 ]]; do
    case "$1" in
    --check)
        run_checks=1
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

repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
: "${IDF_PATH:?source the ESP-IDF export script before building fixtures}"
command -v idf.py >/dev/null 2>&1 || {
    echo "error: idf.py is not on PATH; source \"\$IDF_PATH/export.sh\"" >&2
    exit 1
}

# All pinned fixture evidence currently uses the official v5.3.2 tree. A
# deliberate toolchain update can opt out while producing fresh artifact
# hashes, but an accidental local checkout mismatch must not silently alter a
# compatibility result.
expected_idf_commit=9d7f2d69f50d1288526d4f1027108e314e8c879f
actual_idf_commit=$(git -C "$IDF_PATH" rev-parse HEAD 2>/dev/null || true)
if [[ "${FLEXE_IDF_ALLOW_UNPINNED:-0}" != 1 &&
      "$actual_idf_commit" != "$expected_idf_commit" ]]; then
    echo "error: ESP-IDF must be pinned to $expected_idf_commit" >&2
    echo "       found ${actual_idf_commit:-a non-git IDF tree}" >&2
    echo "       set FLEXE_IDF_ALLOW_UNPINNED=1 for an intentional upgrade" >&2
    exit 1
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

build_root=${FLEXE_IDF_BUILD_ROOT:-"${TMPDIR:-/tmp}/flexe-idf-fixtures"}
build_root=$(canonicalize_dir_target "$build_root") || {
    echo "error: FLEXE_IDF_BUILD_ROOT has a non-directory parent" >&2
    exit 1
}
case "$build_root/" in
"$repo/"*)
    echo "error: FLEXE_IDF_BUILD_ROOT must stay outside the repository" >&2
    exit 1
    ;;
esac
mkdir -p -- "$build_root"
build_root=$(CDPATH= cd -- "$build_root" && pwd -P)

ccache_mode=${FLEXE_IDF_CCACHE:-auto}
case "$ccache_mode" in
auto|0|1) ;;
*)
    echo "error: FLEXE_IDF_CCACHE must be auto, 0, or 1" >&2
    exit 2
    ;;
esac

idf_cache_args=()
cache_state=compiler
if [[ "$ccache_mode" != 0 ]] && command -v ccache >/dev/null 2>&1; then
    cache_home=${XDG_CACHE_HOME:-"${HOME}/.cache"}
    export CCACHE_DIR=${FLEXE_IDF_CCACHE_DIR:-"$cache_home/flexe/esp-idf"}
    export CCACHE_COMPRESS=1
    export CCACHE_MAXSIZE=${FLEXE_IDF_CCACHE_MAXSIZE:-2G}
    # IDF gives every project a distinct generated-config path and
    # -fmacro-prefix-map value. The compiler still receives those exact
    # options, but ccache hashes normalized build-relative paths and ignores
    # the path-only macro-map option. Direct-mode manifests still hash every
    # generated header, including sdkconfig.h, while project source paths stay
    # distinct; this permits fast, correct reuse of common IDF components.
    export CCACHE_IGNOREOPTIONS='-fmacro-prefix-map=*'
    export CCACHE_NAMESPACE="${CCACHE_NAMESPACE:+$CCACHE_NAMESPACE:}flexe-idf-v3"
    mkdir -p -- "$CCACHE_DIR"
    idf_cache_args=(--ccache)
    cache_state=ccache
    echo "==> shared ESP-IDF ccache: $CCACHE_DIR"
elif [[ "$ccache_mode" == 1 ]]; then
    echo "error: FLEXE_IDF_CCACHE=1 but ccache is not installed" >&2
    exit 1
else
    echo "warning: ccache not found; repeated clean fixture builds will be slow" >&2
    echo "         install ccache or set FLEXE_IDF_CCACHE=0 to silence this" >&2
fi

# Keep debug information reproducible across external build directories. This
# also lets ccache retain its safe default directory hashing while recognizing
# that the compiler's recorded working directory is intentionally canonical.
original_extra_cppflags=${EXTRA_CPPFLAGS:-}
helper_config_version=3

resolve_project() {
    local requested=$1 normalized
    if [[ -d "$requested" ]]; then
        project=$(CDPATH= cd -- "$requested" && pwd)
        key=$(basename "$project" | tr '-' '_')
        return
    fi
    normalized=$(printf '%s' "$requested" | tr '-' '_')
    case "$normalized" in
    hello|hello_world|s3_idf_hello)
        key=s3_idf_hello
        project="$IDF_PATH/examples/get-started/hello_world"
        ;;
    s3_idf_*)
        key=$normalized
        project="$repo/tests/fixtures/$key"
        ;;
    *)
        key="s3_idf_$normalized"
        project="$repo/tests/fixtures/$key"
        ;;
    esac
}

host_target_for_key() {
    case "$1" in
    s3_idf_gpio_isr) host_target=flexe-s3-idf-gpio-isr-test ;;
    s3_idf_i2c_master) host_target=flexe-s3-idf-i2c-master-test ;;
    s3_idf_usb_serial_jtag) host_target=flexe-s3-idf-usb-serial-jtag-test ;;
    *) host_target=xtensa-emu ;;
    esac
}

projects=()
keys=()
for requested in "$@"; do
    project=
    key=
    resolve_project "$requested"
    if [[ ! -f "$project/CMakeLists.txt" ]]; then
        echo "error: not an ESP-IDF project: $requested ($project)" >&2
        exit 2
    fi
    projects+=("$project")
    keys+=("$key")
done

host_build=
if [[ "$run_checks" -eq 1 ]]; then
    : "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF for --check}"
    for key in "${keys[@]}"; do
        gate="$repo/scripts/check-$(printf '%s' "$key" | tr '_' '-').sh"
        [[ -x "$gate" ]] || {
            echo "error: no executable gate for $key: $gate" >&2
            exit 1
        }
    done
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
            cmake -S "$repo" -B "$host_build"
        fi
        host_targets=()
        for key in "${keys[@]}"; do
            host_target_for_key "$key"
            case " ${host_targets[*]-} " in
            *" $host_target "*) ;;
            *) host_targets+=("$host_target") ;;
            esac
        done
        echo "==> building host runners"
        cmake --build "$host_build" --target "${host_targets[@]}" -j
    fi
fi

for index in "${!projects[@]}"; do
    project=${projects[$index]}
    key=${keys[$index]}

    build_dir="$build_root/$key-build"
    sdkconfig="$build_root/$key-sdkconfig"
    mkdir -p -- "$build_dir"
    helper_extra_cppflags="${original_extra_cppflags:+$original_extra_cppflags }-fdebug-prefix-map=$build_dir=."
    if [[ ${#idf_cache_args[@]} -ne 0 ]]; then
        export CCACHE_BASEDIR=$build_dir
    fi
    config_signature=$(printf '%s\n' "$helper_config_version" \
        "$actual_idf_commit" "$cache_state" "$helper_extra_cppflags" |
        openssl dgst -sha256 | awk '{print $NF}')
    config_stamp="$build_dir/.flexe-idf-helper-config"
    prior_signature=
    if [[ -f "$config_stamp" ]]; then
        IFS= read -r prior_signature < "$config_stamp" || true
    fi
    if [[ -f "$build_dir/CMakeCache.txt" &&
          "$prior_signature" != "$config_signature" ]]; then
        echo "==> refreshing $key configuration"
        EXTRA_CPPFLAGS="$helper_extra_cppflags" \
            idf.py "${idf_cache_args[@]}" -C "$project" -B "$build_dir" \
                -D "SDKCONFIG=$sdkconfig" -D IDF_TARGET=esp32s3 reconfigure
    fi
    echo "==> building $key"
    EXTRA_CPPFLAGS="$helper_extra_cppflags" \
        idf.py "${idf_cache_args[@]}" -C "$project" -B "$build_dir" \
            -D "SDKCONFIG=$sdkconfig" -D IDF_TARGET=esp32s3 build
    printf '%s\n' "$config_signature" > "$config_stamp"

    shopt -s nullglob
    elf_candidates=("$build_dir"/*.elf)
    shopt -u nullglob
    if [[ ${#elf_candidates[@]} -ne 1 ]]; then
        echo "error: expected one application ELF in $build_dir" >&2
        exit 1
    fi
    elf=${elf_candidates[0]}
    bin=${elf%.elf}.bin
    [[ -s "$bin" ]] || {
        echo "error: build produced no matching application image: $bin" >&2
        exit 1
    }
    bin_hash=$(openssl dgst -sha256 "$bin" | awk '{print $NF}')
    elf_hash=$(openssl dgst -sha256 "$elf" | awk '{print $NF}')
    prefix=$(printf '%s' "$key" | tr '[:lower:]' '[:upper:]')
    # Two early gates predate the fixture-directory naming convention. Keep
    # their public environment variables stable while still accepting the
    # natural project names here.
    case "$key" in
    s3_idf_socket_range) prefix=S3_IDF_SOCKET ;;
    s3_idf_usb_serial_jtag) prefix=S3_IDF_USJ ;;
    esac
    printf '    %s_BIN=%s\n' "$prefix" "$bin"
    printf '    %s_ELF=%s\n' "$prefix" "$elf"
    printf '    %s_BIN_SHA256=%s\n' "$prefix" "$bin_hash"
    printf '    %s_ELF_SHA256=%s\n' "$prefix" "$elf_hash"

    if [[ "$run_checks" -eq 1 ]]; then
        gate="$repo/scripts/check-$(printf '%s' "$key" | tr '_' '-').sh"
        gate_env=("${prefix}_BIN=$bin" "${prefix}_ELF=$elf"
            "${prefix}_BIN_SHA256=$bin_hash"
            "${prefix}_ELF_SHA256=$elf_hash" "S3_ROM_ELF=$S3_ROM_ELF")
        if [[ -n "${RUNNER:-}" ]]; then
            gate_env+=("RUNNER=$RUNNER")
        else
            host_target_for_key "$key"
            gate_runner="$host_build/$host_target"
            [[ -x "$gate_runner" ]] || {
                echo "error: built runner is not executable: $gate_runner" >&2
                exit 1
            }
            gate_env+=("RUNNER=$gate_runner")
        fi
        echo "==> checking $key"
        env "${gate_env[@]}" "$gate"
    fi
done

if [[ ${#idf_cache_args[@]} -ne 0 ]]; then
    echo "==> ccache summary"
    ccache --show-stats
fi
