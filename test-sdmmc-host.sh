#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
arduino_cli=${ARDUINO_CLI:-arduino-cli}
fqbn=${FLEXE_ARDUINO_FQBN:-esp32:esp32:d32:PartitionScheme=min_spiffs}
host_build=${FLEXE_BUILD_DIR:-"$repo_dir/build"}

remove_fixture_build=0
if [ -n "${FLEXE_SDMMC_BUILD_DIR:-}" ]; then
    fixture_build=$FLEXE_SDMMC_BUILD_DIR
    mkdir -p -- "$fixture_build"
else
    fixture_build=$(mktemp -d "${TMPDIR:-/tmp}/flexe-sdmmc-host.XXXXXX")
    remove_fixture_build=1
fi

cleanup() {
    if [ "$remove_fixture_build" -eq 1 ]; then
        rm -rf -- "$fixture_build"
    fi
}
trap cleanup EXIT HUP INT TERM

"$arduino_cli" compile \
    --fqbn "$fqbn" \
    --build-path "$fixture_build" \
    --build-property compiler.optimization_flags=-Os \
    "$repo_dir/tests/fixtures/sdmmc_host"

cmake --build "$host_build" --target flexe-sdmmc-host-test -j
"$host_build/flexe-sdmmc-host-test" \
    "$fixture_build/sdmmc_host.ino.bin" \
    "$fixture_build/sdmmc_host.ino.elf"
"$host_build/flexe-sdmmc-host-test" --no-jit \
    "$fixture_build/sdmmc_host.ino.bin" \
    "$fixture_build/sdmmc_host.ino.elf"
