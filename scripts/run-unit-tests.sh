#!/usr/bin/env bash
# Keep successful unit-test runs compact without losing failure diagnostics.
set -euo pipefail

repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_dir=${FLEXE_BUILD_DIR:-"$repo/build"}
test_binary=${FLEXE_TEST_BINARY:-"$build_dir/xtensa-tests"}

if [[ ! -x "$test_binary" ]]; then
    echo "error: unit-test binary is not executable: $test_binary" >&2
    echo "       build it with: cmake --build $build_dir --target xtensa-tests -j" >&2
    exit 2
fi

stdout_log=$(mktemp "${TMPDIR:-/tmp}/flexe-unit-stdout.XXXXXX")
stderr_log=$(mktemp "${TMPDIR:-/tmp}/flexe-unit-stderr.XXXXXX")
cleanup() {
    rm -f -- "$stdout_log" "$stderr_log"
}
trap cleanup EXIT

set +e
"$test_binary" --quiet "$@" >"$stdout_log" 2>"$stderr_log"
status=$?
set -e

if [[ "$status" -eq 0 ]]; then
    cat "$stdout_log"
    exit 0
fi

echo "error: unit tests failed with status $status; captured output follows" >&2
cat "$stdout_log"
cat "$stderr_log" >&2
exit "$status"
