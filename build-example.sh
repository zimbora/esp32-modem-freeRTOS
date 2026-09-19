#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./build-example.sh <example>

Builds an example sketch with arduino-cli.

Environment variables:
  BOARD_FQBN           Arduino board FQBN (default: esp32:esp32:esp32c5)
  BUILD_PARTITIONS     Partition table (default: min_spiffs)
  UPLOAD_MAXIMUM_SIZE  Maximum sketch size (default: 1966080)
  BUILD_DIR            Build output directory root (default: ./build)
EOF
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -ne 1 ]]; then
  usage
  exit 1
fi

example="$1"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sketch="${script_dir}/examples/${example}/${example}.ino"

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "Error: arduino-cli is not installed or not available in PATH." >&2
  exit 1
fi

if [[ ! -f "${sketch}" ]]; then
  echo "Error: example sketch not found: ${sketch}" >&2
  exit 1
fi

board_fqbn="${BOARD_FQBN:-esp32:esp32:esp32c5}"
build_partitions="${BUILD_PARTITIONS:-min_spiffs}"
upload_maximum_size="${UPLOAD_MAXIMUM_SIZE:-1966080}"
build_root="${BUILD_DIR:-${script_dir}/build}"
build_path="${build_root}/${example}"

echo "Step 1/1: Building '${example}' for '${board_fqbn}'..."

arduino-cli compile -b "${board_fqbn}" \
  --build-property "build.partitions=${build_partitions}" \
  --build-property "upload.maximum_size=${upload_maximum_size}" \
  --build-path "${build_path}" \
  "${sketch}"

echo "Build output: ${build_path}"
