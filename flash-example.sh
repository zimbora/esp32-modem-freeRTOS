#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./flash-example.sh <example> <port>

Flashes a merged firmware image with esptool.

Environment variables:
  BUILD_DIR     Build output directory root (default: ./build)
  FLASH_BAUD    Flash baud rate (default: 460800)
  FLASH_OFFSET  Flash offset (default: 0x0)
EOF
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -ne 2 ]]; then
  usage
  exit 1
fi

example="$1"
port="$2"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_root="${BUILD_DIR:-${script_dir}/build}"
flash_baud="${FLASH_BAUD:-460800}"
flash_offset="${FLASH_OFFSET:-0x0}"
filename="${build_root}/${example}/${example}.ino.merged.bin"

if ! command -v esptool >/dev/null 2>&1; then
  echo "Error: esptool is not installed or not available in PATH." >&2
  exit 1
fi

if [[ ! -f "${filename}" ]]; then
  echo "Error: firmware image not found: ${filename}" >&2
  echo "Build it first with ./build-example.sh ${example}" >&2
  exit 1
fi

echo "Step 1/2: Erasing flash..."
esptool --port "${port}" erase_flash

echo "Step 2/2: Writing firmware '${filename}' to ${flash_offset} at ${flash_baud} baud..."
esptool --port "${port}" --baud "${flash_baud}" write-flash "${flash_offset}" "${filename}"
