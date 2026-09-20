#!/usr/bin/env bash
# Bounded process pool for independent fixture work. S3 builders use it after
# all mutable builds are complete; the classic builder also uses it for
# cache-miss compiles after priming its shared core. Each behavior gate already
# runs interpreter and JIT together, so its default reserves two logical CPUs
# and caps concurrency to keep emulator address spaces from dominating memory.

flexe_fixture_processor_count() {
    local processors
    processors=1
    if [[ "$(uname -s 2>/dev/null || true)" == Darwin ]] &&
       command -v sysctl >/dev/null 2>&1; then
        processors=$(sysctl -n hw.logicalcpu 2>/dev/null || printf '1\n')
    elif command -v getconf >/dev/null 2>&1; then
        processors=$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf '1\n')
    elif command -v nproc >/dev/null 2>&1; then
        processors=$(nproc 2>/dev/null || printf '1\n')
    fi
    case "$processors" in
    ''|*[!0-9]*|0) processors=1 ;;
    esac
    printf '%s\n' "$processors"
}

flexe_fixture_gate_default_jobs() {
    local processors jobs
    processors=$(flexe_fixture_processor_count)
    jobs=$((processors / 2))
    [[ "$jobs" -ge 1 ]] || jobs=1
    [[ "$jobs" -le 4 ]] || jobs=4
    printf '%s\n' "$jobs"
}

flexe_fixture_gate_jobs() {
    local jobs=${FLEXE_FIXTURE_GATE_JOBS:-$(
        flexe_fixture_gate_default_jobs)}
    case "$jobs" in
    ''|*[!0-9]*|0)
        echo "error: FLEXE_FIXTURE_GATE_JOBS must be a positive integer" >&2
        return 2
        ;;
    esac
    printf '%s\n' "$jobs"
}

# An Arduino/ESP-IDF compiler already parallelizes within one firmware build.
# Give each outer build at least three logical CPUs, while still allowing
# independent cache misses to overlap on larger developer machines.
flexe_fixture_build_default_jobs() {
    local processors jobs
    processors=$(flexe_fixture_processor_count)
    jobs=$((processors / 3))
    [[ "$jobs" -ge 1 ]] || jobs=1
    [[ "$jobs" -le 4 ]] || jobs=4
    printf '%s\n' "$jobs"
}

flexe_fixture_build_jobs() {
    local jobs=${FLEXE_FIXTURE_BUILD_JOBS:-$(
        flexe_fixture_build_default_jobs)}
    case "$jobs" in
    ''|*[!0-9]*|0)
        echo "error: FLEXE_FIXTURE_BUILD_JOBS must be a positive integer" >&2
        return 2
        ;;
    esac
    printf '%s\n' "$jobs"
}

flexe_fixture_gate_pool_init() {
    local requested=$1 count=$2 fifo token
    case "$requested" in ''|*[!0-9]*|0) return 2 ;; esac
    [[ "$count" -ge 1 ]] || return 0
    if [[ "$requested" -gt "$count" ]]; then
        requested=$count
    fi
    FLEXE_GATE_POOL_DIR=$(mktemp -d \
        "${TMPDIR:-/tmp}/flexe-fixture-gates.XXXXXX")
    fifo=$FLEXE_GATE_POOL_DIR/tokens
    mkfifo "$fifo"
    exec 9<>"$fifo"
    rm -f -- "$fifo"
    FLEXE_GATE_POOL_PIDS=()
    FLEXE_GATE_POOL_LABELS=()
    FLEXE_GATE_POOL_LOGS=()
    FLEXE_GATE_POOL_ACTIVE=1
    token=ready
    for ((count = 0; count < requested; count++)); do
        printf '%s\n' "$token" >&9
    done
}

flexe_fixture_gate_pool_submit() {
    local label=$1 token index log verb
    shift
    IFS= read -r token <&9
    index=${#FLEXE_GATE_POOL_PIDS[@]}
    log=$FLEXE_GATE_POOL_DIR/$index.log
    FLEXE_GATE_POOL_LABELS+=("$label")
    FLEXE_GATE_POOL_LOGS+=("$log")
    verb=${FLEXE_FIXTURE_POOL_VERB:-checking}
    echo "==> $verb $label"
    (
        local status=0
        "$@" >"$log" 2>&1 || status=$?
        printf 'ready\n' >&9
        exit "$status"
    ) &
    FLEXE_GATE_POOL_PIDS+=("$!")
}

flexe_fixture_gate_pool_cleanup() {
    local log
    [[ "${FLEXE_GATE_POOL_ACTIVE:-0}" -eq 1 ]] || return 0
    exec 9>&-
    for log in "${FLEXE_GATE_POOL_LOGS[@]}"; do
        rm -f -- "$log"
    done
    rmdir "$FLEXE_GATE_POOL_DIR" 2>/dev/null || true
    FLEXE_GATE_POOL_ACTIVE=0
}

flexe_fixture_process_tree_signal() {
    local signal=$1 pid=$2 child
    local children=()
    if command -v pgrep >/dev/null 2>&1; then
        while IFS= read -r child; do
            [[ -n "$child" ]] && children+=("$child")
        done < <(pgrep -P "$pid" 2>/dev/null || true)
    fi
    kill -s "$signal" "$pid" 2>/dev/null || true
    for child in "${children[@]}"; do
        flexe_fixture_process_tree_signal "$signal" "$child"
    done
}

flexe_fixture_gate_pool_abort() {
    local pid attempt alive
    [[ "${FLEXE_GATE_POOL_ACTIVE:-0}" -eq 1 ]] || return 0
    for pid in "${FLEXE_GATE_POOL_PIDS[@]}"; do
        flexe_fixture_process_tree_signal TERM "$pid"
    done
    # A gate may have installed a TERM trap that only removes temporary files.
    # Bound shutdown so such a child cannot keep CI or an interrupted local
    # validation alive indefinitely, then reap every wrapper below.
    for ((attempt = 0; attempt < 20; attempt++)); do
        alive=0
        for pid in "${FLEXE_GATE_POOL_PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                alive=1
                break
            fi
        done
        [[ "$alive" -eq 1 ]] || break
        sleep 0.05
    done
    for pid in "${FLEXE_GATE_POOL_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            flexe_fixture_process_tree_signal KILL "$pid"
        fi
    done
    for pid in "${FLEXE_GATE_POOL_PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    flexe_fixture_gate_pool_cleanup
}

flexe_fixture_gate_pool_wait() {
    local index status gate_status verb
    status=0
    verb=${FLEXE_FIXTURE_POOL_VERB:-checking}
    for index in "${!FLEXE_GATE_POOL_PIDS[@]}"; do
        gate_status=0
        if wait "${FLEXE_GATE_POOL_PIDS[$index]}"; then
            :
        else
            gate_status=$?
            status=1
        fi
        cat "${FLEXE_GATE_POOL_LOGS[$index]}"
        if [[ "$gate_status" -ne 0 ]]; then
            echo "error: $verb ${FLEXE_GATE_POOL_LABELS[$index]} failed "\
"with status $gate_status" >&2
        fi
    done
    flexe_fixture_gate_pool_cleanup
    return "$status"
}

# Both S3 fixture builders expose the same image/ELF/hash contract to their
# behavior gates. Queue that metadata while firmware is being built, then run
# it only after compilation has finished so the emulator pool never competes
# with a compiler already using every host core.
FLEXE_GATE_QUEUE_LABELS=()
FLEXE_GATE_QUEUE_GATES=()
FLEXE_GATE_QUEUE_PREFIXES=()
FLEXE_GATE_QUEUE_BINS=()
FLEXE_GATE_QUEUE_ELFS=()
FLEXE_GATE_QUEUE_BIN_HASHES=()
FLEXE_GATE_QUEUE_ELF_HASHES=()
FLEXE_GATE_QUEUE_ROMS=()
FLEXE_GATE_QUEUE_RUNNERS=()
FLEXE_GATE_QUEUE_RUNNER_ENTRIES=()

flexe_fixture_gate_queue_artifact() {
    FLEXE_GATE_QUEUE_LABELS+=("$1")
    FLEXE_GATE_QUEUE_GATES+=("$2")
    FLEXE_GATE_QUEUE_PREFIXES+=("$3")
    FLEXE_GATE_QUEUE_BINS+=("$4")
    FLEXE_GATE_QUEUE_ELFS+=("$5")
    FLEXE_GATE_QUEUE_BIN_HASHES+=("$6")
    FLEXE_GATE_QUEUE_ELF_HASHES+=("$7")
    FLEXE_GATE_QUEUE_ROMS+=("$8")
    FLEXE_GATE_QUEUE_RUNNERS+=("$9")
    FLEXE_GATE_QUEUE_RUNNER_ENTRIES+=("${10:-}")
}

flexe_fixture_gate_run_queued() {
    local jobs=$1 index prefix status
    [[ ${#FLEXE_GATE_QUEUE_LABELS[@]} -ne 0 ]] || return 0
    flexe_fixture_gate_pool_init \
        "$jobs" "${#FLEXE_GATE_QUEUE_LABELS[@]}"
    trap 'flexe_fixture_gate_pool_abort' EXIT
    for index in "${!FLEXE_GATE_QUEUE_LABELS[@]}"; do
        prefix=${FLEXE_GATE_QUEUE_PREFIXES[$index]}
        flexe_fixture_gate_pool_submit \
            "${FLEXE_GATE_QUEUE_LABELS[$index]}" env \
            "${prefix}_BIN=${FLEXE_GATE_QUEUE_BINS[$index]}" \
            "${prefix}_ELF=${FLEXE_GATE_QUEUE_ELFS[$index]}" \
            "${prefix}_BIN_SHA256=${FLEXE_GATE_QUEUE_BIN_HASHES[$index]}" \
            "${prefix}_ELF_SHA256=${FLEXE_GATE_QUEUE_ELF_HASHES[$index]}" \
            "S3_ROM_ELF=${FLEXE_GATE_QUEUE_ROMS[$index]}" \
            "RUNNER=${FLEXE_GATE_QUEUE_RUNNERS[$index]}" \
            "FLEXE_FIXTURE_RUNNER_ENTRY=${FLEXE_GATE_QUEUE_RUNNER_ENTRIES[$index]}" \
            "${FLEXE_GATE_QUEUE_GATES[$index]}"
    done
    status=0
    flexe_fixture_gate_pool_wait || status=$?
    return "$status"
}
