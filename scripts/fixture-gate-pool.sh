#!/usr/bin/env bash
# Bounded process pool for independent firmware gates. Source this file from a
# fixture builder after all mutable build work is complete, then submit gates
# that only read immutable artifacts. Each gate already runs its interpreter
# and JIT together, so the default reserves two logical CPUs per gate and caps
# the pool to keep emulator address spaces from dominating host memory.

flexe_fixture_gate_default_jobs() {
    local processors jobs
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
    local label=$1 token index log
    shift
    IFS= read -r token <&9
    index=${#FLEXE_GATE_POOL_PIDS[@]}
    log=$FLEXE_GATE_POOL_DIR/$index.log
    FLEXE_GATE_POOL_LABELS+=("$label")
    FLEXE_GATE_POOL_LOGS+=("$log")
    echo "==> checking $label"
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

flexe_fixture_gate_pool_abort() {
    local pid
    [[ "${FLEXE_GATE_POOL_ACTIVE:-0}" -eq 1 ]] || return 0
    for pid in "${FLEXE_GATE_POOL_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    for pid in "${FLEXE_GATE_POOL_PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    flexe_fixture_gate_pool_cleanup
}

flexe_fixture_gate_pool_wait() {
    local index status gate_status
    status=0
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
            echo "error: ${FLEXE_GATE_POOL_LABELS[$index]} gate failed "\
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
            "${FLEXE_GATE_QUEUE_GATES[$index]}"
    done
    status=0
    flexe_fixture_gate_pool_wait || status=$?
    return "$status"
}
