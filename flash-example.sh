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
  ESPTOOL_CHIP  Chip target passed to esptool (default: esp32c5)
  ESPTOOL_SUDO  Set to 1 to run esptool through sudo
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
esptool_chip="${ESPTOOL_CHIP:-esp32c5}"
filename="${build_root}/${example}/${example}.ino.merged.bin"

if command -v esptool >/dev/null 2>&1; then
  esptool_cmd="$(command -v esptool)"
elif command -v esptool.py >/dev/null 2>&1; then
  esptool_cmd="$(command -v esptool.py)"
else
  echo "Error: esptool is not installed or not available in PATH (expected 'esptool' or 'esptool.py')." >&2
  exit 1
fi

if [[ "${ESPTOOL_SUDO:-0}" == "1" ]]; then
  if ! command -v sudo >/dev/null 2>&1; then
    echo "Error: sudo is required when ESPTOOL_SUDO=1." >&2
    exit 1
  fi
  esptool_runner=(sudo "${esptool_cmd}")
else
  esptool_runner=("${esptool_cmd}")
fi

if [[ ! -f "${filename}" ]]; then
  echo "Error: firmware image not found: ${filename}" >&2
  echo "Build it first with ./build-example.sh ${example}" >&2
  exit 1
fi

echo "Step 1/2: Erasing flash..."
"${esptool_runner[@]}" --chip "${esptool_chip}" --port "${port}" erase_flash

echo "Step 2/2: Writing firmware '${filename}' to ${flash_offset} at ${flash_baud} baud..."
"${esptool_runner[@]}" --chip "${esptool_chip}" --port "${port}" --baud "${flash_baud}" write_flash "${flash_offset}" "${filename}"
