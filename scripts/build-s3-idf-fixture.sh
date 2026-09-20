#!/usr/bin/env bash
# Build one or more pinned ESP-IDF S3 fixtures outside the source tree.
# Persistent build directories handle incremental rebuilds; an input/artifact
# fingerprint bypasses Ninja's always-run IDF post-build steps when nothing
# changed, and a shared ccache avoids recompiling common IDF components.
set -euo pipefail

usage() {
    cat <<'EOF'
usage: build-s3-idf-fixture.sh [--check] [--rebuild] [--verbose] NAME...

Names may be short aliases such as hello, crosscore, nvs, or sleep; an
in-tree s3_idf_* project name; a path to another ESP-IDF project; or all.

Environment:
  FLEXE_IDF_PATH             pinned ESP-IDF checkout (auto-detected otherwise)
  FLEXE_IDF_BUILD_ROOT       persistent external output root
  FLEXE_IDF_CCACHE           auto (default), 1 (required), or 0
  FLEXE_IDF_CCACHE_DIR       shared compiler-cache directory
  FLEXE_IDF_CCACHE_MAXSIZE   cache limit (default: 2G)
  FLEXE_FIXTURE_GATE_JOBS    concurrent read-only gates (default: host-aware)
  FLEXE_IDF_ALLOW_UNPINNED   1 permits an IDF revision other than v5.3.2
  FLEXE_IDF_VERBOSE          1 streams build output and ccache statistics
  FLEXE_BUILD_DIR            host CMake build used by --check
  S3_ROM_ELF                 official ROM ELF (auto-detected for --check)
  RUNNER                     custom runner used by every requested check
  SOURCE_DATE_EPOCH          explicit artifact timestamp
EOF
}

run_checks=0
force_rebuild=0
verbose=${FLEXE_IDF_VERBOSE:-0}
case "$verbose" in
0|1) ;;
*)
    echo "error: FLEXE_IDF_VERBOSE must be 0 or 1" >&2
    exit 2
    ;;
esac
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

all_fixtures=(hello crosscore nvs sleep gpio-wake gpio-isr \
    usb-serial-jtag i2c-master i2s-std sdmmc-host rmt-loopback socket-range)
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

# A normal fixture check should work from a fresh shell. Prefer an explicit
# checkout, then an existing ESP-IDF environment, then the conventional IDF
# install location. The exact revision is still enforced below.
if [[ -n "${FLEXE_IDF_PATH:-}" ]]; then
    idf_candidate=$FLEXE_IDF_PATH
elif [[ -n "${IDF_PATH:-}" ]]; then
    idf_candidate=$IDF_PATH
elif command -v idf.py >/dev/null 2>&1; then
    idf_candidate=$(dirname -- "$(command -v idf.py)")/..
elif [[ -n "${HOME:-}" && -f "$HOME/esp/esp-idf/export.sh" ]]; then
    idf_candidate=$HOME/esp/esp-idf
else
    echo "error: could not find ESP-IDF; set FLEXE_IDF_PATH or IDF_PATH" >&2
    exit 1
fi
IDF_PATH=$(CDPATH= cd -- "$idf_candidate" 2>/dev/null && pwd -P) || {
    echo "error: ESP-IDF directory does not exist: $idf_candidate" >&2
    exit 1
}
export IDF_PATH
idf_py=$IDF_PATH/tools/idf.py
[[ -x "$idf_py" && -f "$IDF_PATH/export.sh" ]] || {
    echo "error: not an ESP-IDF checkout: $IDF_PATH" >&2
    exit 1
}

# All pinned fixture evidence currently uses the official v5.3.2 tree. Check
# the checkout before sourcing any of its shell code. A deliberate toolchain
# update can opt out while producing fresh artifact hashes, but an accidental
# local checkout mismatch must not silently alter a compatibility result.
expected_idf_commit=9d7f2d69f50d1288526d4f1027108e314e8c879f
actual_idf_commit=$(git -C "$IDF_PATH" rev-parse HEAD 2>/dev/null || true)
if [[ "${FLEXE_IDF_ALLOW_UNPINNED:-0}" != 1 &&
      "$actual_idf_commit" != "$expected_idf_commit" ]]; then
    echo "error: ESP-IDF must be pinned to $expected_idf_commit" >&2
    echo "       found ${actual_idf_commit:-a non-git IDF tree}" >&2
    echo "       set FLEXE_IDF_ALLOW_UNPINNED=1 for an intentional upgrade" >&2
    exit 1
fi

# A warm artifact hit must not pay ESP-IDF's roughly half-second shell export.
# Initialize the Python environment and compiler lazily on the first cache
# miss. Capturing the export chatter keeps routine runs concise while a failure
# retains the complete diagnostic.
idf_tools_root=${IDF_TOOLS_PATH:-${HOME:+$HOME/.espressif}}
idf_environment_ready=0
initialize_idf_environment() {
    local active_idf_py export_log idf_export_status
    local python_env_candidate
    local python_env_candidates=()
    [[ "$idf_environment_ready" -eq 0 ]] || return 0
    active_idf_py=$(command -v idf.py 2>/dev/null || true)
    if [[ "$active_idf_py" != "$idf_py" ]] ||
       ! command -v xtensa-esp32s3-elf-gcc >/dev/null 2>&1; then
        if [[ -z "${IDF_PYTHON_ENV_PATH:-}" &&
              "$actual_idf_commit" == "$expected_idf_commit" &&
              -n "$idf_tools_root" ]]; then
            shopt -s nullglob
            python_env_candidates=(
                "$idf_tools_root"/python_env/idf5.3_py*_env)
            shopt -u nullglob
            for python_env_candidate in "${python_env_candidates[@]}"; do
                if [[ -x "$python_env_candidate/bin/python" &&
                      -f "$python_env_candidate/idf_version.txt" ]] &&
                   [[ "$(<"$python_env_candidate/idf_version.txt")" == 5.3 ]]; then
                    export IDF_PYTHON_ENV_PATH=$python_env_candidate
                    break
                fi
            done
        fi
        export_log=$(mktemp "${TMPDIR:-/tmp}/flexe-idf-export.XXXXXX")
        set +e
        set +u
        source "$IDF_PATH/export.sh" >"$export_log" 2>&1
        idf_export_status=$?
        set -u
        set -e
        if [[ "$idf_export_status" -ne 0 ]]; then
            echo "error: could not initialize ESP-IDF from $IDF_PATH" >&2
            cat "$export_log" >&2
            rm -f -- "$export_log"
            exit 1
        fi
        rm -f -- "$export_log"
    fi
    command -v xtensa-esp32s3-elf-gcc >/dev/null 2>&1 || {
        echo "error: ESP32-S3 compiler was not initialized by ESP-IDF" >&2
        exit 1
    }
    idf_environment_ready=1
}

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

build_root=${FLEXE_IDF_BUILD_ROOT:-"$cache_home/flexe/esp-idf-fixtures"}
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
    if [[ "$verbose" -eq 1 ]]; then
        echo "==> shared ESP-IDF ccache: $CCACHE_DIR"
    fi
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
requested_epoch=${SOURCE_DATE_EPOCH:-}
if [[ -n "$requested_epoch" && ! "$requested_epoch" =~ ^[0-9]+$ ]]; then
    echo "error: SOURCE_DATE_EPOCH must be an integer" >&2
    exit 2
fi
# These projects live below the Flexe checkout, but their fixed project
# versions make the parent repository revision irrelevant to their output.
# Stop Git discovery at the checkout root so ESP-IDF's generated Ninja graph
# does not depend on .git/HEAD and rebuild every fixture after a Flexe commit.
fixture_git_ceiling=$repo
if [[ -n "${GIT_CEILING_DIRECTORIES:-}" ]]; then
    fixture_git_ceiling="$repo:$GIT_CEILING_DIRECTORIES"
fi
# This version describes only inputs that affect ESP-IDF's generated build
# graph or firmware. Adding a fixture alias, host runner, or behavior gate must
# not invalidate every existing firmware artifact. Version 7 was accidentally
# used for the catalog-only I2S addition and is therefore metadata-compatible
# with version 6; accept it once and rewrite the stamp without rebuilding.
firmware_cache_version=6
compatible_firmware_cache_versions=(7)

idf_py_hash=$(openssl dgst -sha256 "$idf_py" | awk '{print $NF}')
cmake_version=$(cmake --version | sed -n '1p')
ninja_version=$(ninja --version 2>/dev/null || printf 'unavailable')
idf_tools_state=$({
    printf 'root=%s\n' "$idf_tools_root"
    for tools_metadata in "$IDF_PATH/tools/tools.json" \
            "$idf_tools_root/idf-env.json"; do
        if [[ -f "$tools_metadata" ]]; then
            tools_metadata_hash=$(openssl dgst -sha256 "$tools_metadata" |
                awk '{print $NF}')
            printf '%s=%s\n' "$tools_metadata" "$tools_metadata_hash"
        else
            printf '%s=missing\n' "$tools_metadata"
        fi
    done
} | openssl dgst -sha256 | awk '{print $NF}')
idf_tree_state=clean
if ! git -C "$IDF_PATH" diff --quiet HEAD -- 2>/dev/null; then
    idf_tree_state=$(git -C "$IDF_PATH" diff --binary HEAD -- |
        openssl dgst -sha256 | awk '{print $NF}')
fi
build_environment_state=$({
    printf 'idf-py=%s\n' "$idf_py_hash"
    printf 'idf-tools=%s\n' "$idf_tools_state"
    printf 'cmake=%s\n' "$cmake_version"
    printf 'ninja=%s\n' "$ninja_version"
    for build_variable in PROJECT_VER PROJECT_VER_NUMBER SDKCONFIG_DEFAULTS \
            EXTRA_CFLAGS EXTRA_CXXFLAGS EXTRA_CPPFLAGS CFLAGS CXXFLAGS \
            CPPFLAGS LDFLAGS; do
        build_value=$(printenv "$build_variable" 2>/dev/null || true)
        printf '%s=%s\n' "$build_variable" "$build_value"
    done
} | openssl dgst -sha256 | awk '{print $NF}')

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

fixture_fingerprint() {
    local project_dir=$1 sdkconfig_file=$2 config_signature=$3 epoch=$4
    local cache_version=$5
    local input relative digest variable value
    {
        printf 'flexe-s3-idf-helper=%s\n' "$cache_version"
        printf 'project=%s\n' "$project_dir"
        printf 'idf-path=%s\n' "$IDF_PATH"
        printf 'idf-commit=%s\n' "$actual_idf_commit"
        printf 'idf-tree=%s\n' "$idf_tree_state"
        printf 'idf-py-sha256=%s\n' "$idf_py_hash"
        printf 'idf-tools=%s\n' "$idf_tools_state"
        printf 'cmake=%s\n' "$cmake_version"
        printf 'ninja=%s\n' "$ninja_version"
        printf 'configuration=%s\n' "$config_signature"
        printf 'source-date-epoch=%s\n' "$epoch"
        for variable in PROJECT_VER PROJECT_VER_NUMBER SDKCONFIG_DEFAULTS \
                EXTRA_CFLAGS EXTRA_CXXFLAGS \
                EXTRA_CPPFLAGS CFLAGS CXXFLAGS CPPFLAGS LDFLAGS; do
            value=$(printenv "$variable" 2>/dev/null || true)
            printf 'environment-%s=%s\n' "$variable" "$value"
        done
        find "$project_dir" -type f \
                ! -path '*/.git/*' ! -path '*/build/*' -print |
            LC_ALL=C sort | while IFS= read -r input; do
                relative=${input#"$project_dir"/}
                digest=$(openssl dgst -sha256 "$input" | awk '{print $NF}')
                printf 'source=%s:%s\n' "$relative" "$digest"
            done
        if [[ -f "$sdkconfig_file" ]]; then
            digest=$(openssl dgst -sha256 "$sdkconfig_file" | awk '{print $NF}')
            printf 'sdkconfig=%s\n' "$digest"
        else
            printf 'sdkconfig=missing\n'
        fi
    } | openssl dgst -sha256 | awk '{print $NF}'
}

configuration_signature() {
    local cache_version=$1 extra_cppflags=$2 epoch=$3
    printf '%s\n' "$cache_version" "$actual_idf_commit" "$cache_state" \
        "$extra_cppflags" "$fixture_git_ceiling" "$epoch" |
        openssl dgst -sha256 | awk '{print $NF}'
}

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
    s3_idf_i2s_std) host_target=flexe-s3-idf-i2s-std-test ;;
    s3_idf_sdmmc_host) host_target=flexe-sdmmc-host-test ;;
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
    case " ${keys[*]-} " in
    *" $key "*)
        echo "error: duplicate fixture: $requested" >&2
        exit 2
        ;;
    esac
    projects+=("$project")
    keys+=("$key")
done

host_build=
if [[ "$run_checks" -eq 1 ]]; then
    if [[ -z "${S3_ROM_ELF:-}" ]]; then
        idf_tools_root=${IDF_TOOLS_PATH:-${HOME:+$HOME/.espressif}}
        rom_candidates=()
        if [[ -n "$idf_tools_root" ]]; then
            shopt -s nullglob
            rom_candidates=(
                "$idf_tools_root"/tools/esp-rom-elfs/*/esp32s3_rev0_rom.elf)
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
    if [[ -z "${S3_ROM_ELF:-}" || ! -f "$S3_ROM_ELF" ]]; then
        echo "error: set S3_ROM_ELF to the official ESP32-S3 revision-0 ROM ELF" >&2
        exit 1
    fi
    export S3_ROM_ELF
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
            run_logged "host runner configuration" \
                "$build_root/host-configure.log" \
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
        run_logged "host runner build" "$build_root/host-build.log" \
            cmake --build "$host_build" --target "${host_targets[@]}" -j
    fi
fi

for index in "${!projects[@]}"; do
    project=${projects[$index]}
    key=${keys[$index]}

    build_dir="$build_root/$key-build"
    sdkconfig="$build_root/$key-sdkconfig"
    mkdir -p -- "$build_dir"
    input_stamp="$build_dir/.flexe-idf-input.sha256"
    automatic_epoch_stamp="$build_dir/.flexe-source-date-epoch"
    epoch=$requested_epoch
    if [[ -z "$epoch" ]]; then
        automatic_epoch=
        if [[ -f "$automatic_epoch_stamp" ]]; then
            IFS= read -r automatic_epoch < "$automatic_epoch_stamp" || true
        fi
        # Migrate builds created before the dedicated epoch stamp. Retaining
        # their recorded value avoids a rebuild merely because the source was
        # committed after its pre-commit validation build.
        if [[ ! "$automatic_epoch" =~ ^[0-9]+$ && -f "$input_stamp" ]]; then
            automatic_epoch=$(sed -n '4p' "$input_stamp")
        fi
        # An interrupted legacy build has no input stamp yet. Preserve its old
        # project-commit policy so Ninja can resume instead of full-cleaning.
        if [[ ! "$automatic_epoch" =~ ^[0-9]+$ &&
              -f "$build_dir/CMakeCache.txt" ]]; then
            automatic_epoch=$(git -C "$project" log -1 --format=%ct -- . \
                2>/dev/null || true)
        fi
        # New build directories use the pinned toolchain epoch. Source content
        # is fingerprinted independently, while this value remains stable
        # across the edit/build/commit cycle.
        if [[ ! "$automatic_epoch" =~ ^[0-9]+$ ]]; then
            automatic_epoch=$(git -C "$IDF_PATH" show -s --format=%ct \
                "$actual_idf_commit" 2>/dev/null || true)
        fi
        epoch=$automatic_epoch
        if [[ "$epoch" =~ ^[0-9]+$ ]]; then
            printf '%s\n' "$epoch" > "$automatic_epoch_stamp.tmp"
            mv "$automatic_epoch_stamp.tmp" "$automatic_epoch_stamp"
        fi
    fi
    [[ "$epoch" =~ ^[0-9]+$ ]] || {
        echo "error: could not derive SOURCE_DATE_EPOCH for $project" >&2
        exit 1
    }
    helper_extra_cppflags="${original_extra_cppflags:+$original_extra_cppflags }-fdebug-prefix-map=$build_dir=."
    if [[ ${#idf_cache_args[@]} -ne 0 ]]; then
        export CCACHE_BASEDIR=$build_dir
    fi
    config_signature=$(configuration_signature "$firmware_cache_version" \
        "$helper_extra_cppflags" "$epoch")
    config_stamp="$build_dir/.flexe-idf-helper-config"
    pending_config_stamp="$build_dir/.flexe-idf-pending-config"
    prior_signature=
    using_pending_config=0
    if [[ -f "$config_stamp" ]]; then
        IFS= read -r prior_signature < "$config_stamp" || true
    elif [[ -f "$build_dir/CMakeCache.txt" &&
            -f "$pending_config_stamp" ]]; then
        # A configure-and-build command may have been interrupted after CMake
        # completed. Its pending stamp makes that partial tree safely resumable.
        IFS= read -r prior_signature < "$pending_config_stamp" || true
        using_pending_config=1
    fi
    prior_cache_version=
    if [[ "$prior_signature" == "$config_signature" ]]; then
        prior_cache_version=$firmware_cache_version
    else
        for compatible_version in \
                "${compatible_firmware_cache_versions[@]}"; do
            compatible_signature=$(configuration_signature \
                "$compatible_version" "$helper_extra_cppflags" "$epoch")
            if [[ "$prior_signature" == "$compatible_signature" ]]; then
                prior_cache_version=$compatible_version
                break
            fi
        done
    fi
    build_log="$build_dir/.flexe-build.log"
    input_fingerprint=$(fixture_fingerprint "$project" "$sdkconfig" \
        "$config_signature" "$epoch" "$firmware_cache_version")
    shopt -s nullglob
    elf_candidates=("$build_dir"/*.elf)
    shopt -u nullglob
    elf=
    bin=
    bin_hash=
    elf_hash=
    if [[ ${#elf_candidates[@]} -eq 1 ]]; then
        elf=${elf_candidates[0]}
        bin=${elf%.elf}.bin
        if [[ -s "$bin" && -s "$elf" ]]; then
            bin_hash=$(openssl dgst -sha256 "$bin" | awk '{print $NF}')
            elf_hash=$(openssl dgst -sha256 "$elf" | awk '{print $NF}')
        fi
    fi
    cached_fingerprint=
    cached_bin_hash=
    cached_elf_hash=
    cached_epoch=
    cached_build_environment=
    build_environment_changed=0
    if [[ -f "$input_stamp" ]]; then
        cached_fingerprint=$(sed -n '1p' "$input_stamp")
        cached_bin_hash=$(sed -n '2p' "$input_stamp")
        cached_elf_hash=$(sed -n '3p' "$input_stamp")
        cached_epoch=$(sed -n '4p' "$input_stamp")
        cached_build_environment=$(sed -n '5p' "$input_stamp")
    fi
    if [[ -n "$cached_build_environment" &&
          "$cached_build_environment" != "$build_environment_state" ]]; then
        build_environment_changed=1
    fi
    matching_fingerprint=$input_fingerprint
    if [[ -n "$prior_cache_version" &&
          "$prior_cache_version" != "$firmware_cache_version" ]]; then
        matching_fingerprint=$(fixture_fingerprint "$project" "$sdkconfig" \
            "$prior_signature" "$epoch" "$prior_cache_version")
    fi
    if [[ "$force_rebuild" -eq 0 && -f "$build_dir/CMakeCache.txt" &&
          -n "$prior_cache_version" && -n "$bin_hash" && -n "$elf_hash" &&
          "$cached_fingerprint" == "$matching_fingerprint" &&
          "$cached_bin_hash" == "$bin_hash" &&
          "$cached_elf_hash" == "$elf_hash" &&
          "$cached_epoch" == "$epoch" ]]; then
        echo "==> reusing unchanged $key firmware"
        if [[ "$prior_cache_version" != "$firmware_cache_version" ||
              "$using_pending_config" -eq 1 ]]; then
            printf '%s\n' "$config_signature" > "$config_stamp.tmp"
            mv "$config_stamp.tmp" "$config_stamp"
            rm -f -- "$pending_config_stamp"
        fi
        if [[ -z "$cached_build_environment" ||
              "$prior_cache_version" != "$firmware_cache_version" ]]; then
            printf '%s\n%s\n%s\n%s\n%s\n' "$input_fingerprint" \
                "$bin_hash" "$elf_hash" "$epoch" \
                "$build_environment_state" > "$input_stamp.tmp"
            mv "$input_stamp.tmp" "$input_stamp"
        fi
    else
        rm -f -- "$input_stamp"
        initialize_idf_environment
        if [[ -f "$build_dir/CMakeCache.txt" &&
              ( "$force_rebuild" -eq 1 ||
                -z "$prior_cache_version" ||
                "$build_environment_changed" -eq 1 ) ]]; then
            if [[ "$force_rebuild" -eq 1 ]]; then
                echo "==> rebuilding $key from a clean configuration"
            else
                echo "==> refreshing $key build configuration"
            fi
            run_logged "$key full clean" "$build_root/$key-fullclean.log" \
                env "GIT_CEILING_DIRECTORIES=$fixture_git_ceiling" \
                "SOURCE_DATE_EPOCH=$epoch" \
                "$idf_py" -C "$project" -B "$build_dir" fullclean
            prior_signature=
        fi
        if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
            echo "==> configuring and building $key"
            printf '%s\n' "$config_signature" > "$pending_config_stamp.tmp"
            mv "$pending_config_stamp.tmp" "$pending_config_stamp"
            run_logged "$key configure/build" "$build_log" \
                env "GIT_CEILING_DIRECTORIES=$fixture_git_ceiling" \
                "SOURCE_DATE_EPOCH=$epoch" \
                "EXTRA_CPPFLAGS=$helper_extra_cppflags" \
                "$idf_py" "${idf_cache_args[@]}" -C "$project" \
                    -B "$build_dir" -D "SDKCONFIG=$sdkconfig" \
                    -D IDF_TARGET=esp32s3 build
        else
            echo "==> building $key"
            run_logged "$key build" "$build_log" \
                env "GIT_CEILING_DIRECTORIES=$fixture_git_ceiling" \
                "SOURCE_DATE_EPOCH=$epoch" \
                cmake --build "$build_dir" --parallel
        fi
        printf '%s\n' "$config_signature" > "$config_stamp.tmp"
        mv "$config_stamp.tmp" "$config_stamp"
        rm -f -- "$pending_config_stamp"

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
        input_fingerprint=$(fixture_fingerprint "$project" "$sdkconfig" \
            "$config_signature" "$epoch" "$firmware_cache_version")
        printf '%s\n%s\n%s\n%s\n%s\n' "$input_fingerprint" "$bin_hash" \
            "$elf_hash" "$epoch" "$build_environment_state" \
            > "$input_stamp.tmp"
        mv "$input_stamp.tmp" "$input_stamp"
    fi

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
        if [[ -n "${RUNNER:-}" ]]; then
            gate_runner=$RUNNER
        else
            host_target_for_key "$key"
            gate_runner="$host_build/$host_target"
            [[ -x "$gate_runner" ]] || {
                echo "error: built runner is not executable: $gate_runner" >&2
                exit 1
            }
        fi
        flexe_fixture_gate_queue_artifact "$key" "$gate" "$prefix" \
            "$bin" "$elf" "$bin_hash" "$elf_hash" \
            "$S3_ROM_ELF" "$gate_runner"
    fi
done

if [[ "$run_checks" -eq 1 ]]; then
    gate_status=0
    flexe_fixture_gate_run_queued "$gate_jobs" || gate_status=$?
    [[ "$gate_status" -eq 0 ]] || exit "$gate_status"
fi

if [[ ${#idf_cache_args[@]} -ne 0 && "$verbose" -eq 1 ]]; then
    echo "==> ccache summary"
    ccache --show-stats
fi
