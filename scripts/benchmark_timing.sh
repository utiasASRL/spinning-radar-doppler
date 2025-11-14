#!/bin/bash
# -----------------------------------------------------------------------------
# Script: benchmark_timing.sh
#
# Description:
#   Benchmarks the timing of various parts of the extractor on a provided sequence.
#
# Behavior:
#   - Reads radar data from the provided input path.
#   - Times each part of the extraction process and prints the results to screen.
#
# Assumptions:
#   None.
# -----------------------------------------------------------------------------

set -euo pipefail

MAX_FRAMES=""
POSITIONAL=()

# --- Parse args ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --max_frames)
            if [[ $# -lt 2 ]]; then
                echo "Error: --max_frames requires an integer argument."
                exit 1
            fi
            MAX_FRAMES="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [--max_frames N] <sequence_path>"
            exit 0
            ;;
        *)
            POSITIONAL+=("$1")
            shift
            ;;
    esac
done

if [[ ${#POSITIONAL[@]} -ne 1 ]]; then
    echo "Usage: $0 [--max_frames N] <sequence_path>"
    exit 1
fi

SEQUENCE_PATH="${POSITIONAL[0]}"

# --- Resolve paths ---
INPUT_PATH="$(realpath "$SEQUENCE_PATH")"
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_PATH="$(realpath "$SCRIPT_PATH/..")"
CONFIG_PATH="$PROJECT_PATH/config/doppler_config.yaml"
BUILD_PATH="$PROJECT_PATH/build"
CURR_PATH="$(pwd)"

echo "Processing radar scans for sequence: $INPUT_PATH"

# --- Sanity checks ---
if [ ! -d "$INPUT_PATH" ]; then
    echo "Error: input path does not exist: $INPUT_PATH"
    exit 1
fi

if [ ! -x "$BUILD_PATH/apps/benchmark_timing" ]; then
    echo "Error: binary not found or not executable: $BUILD_PATH/apps/benchmark_timing"
    exit 1
fi

if [ ! -f "$CONFIG_PATH" ]; then
    echo "Error: config file not found: $CONFIG_PATH"
    exit 1
fi

# --- Run benchmark ---
cd "$BUILD_PATH"

CMD=( "./apps/benchmark_timing" "$INPUT_PATH" "$CONFIG_PATH" )
if [[ -n "$MAX_FRAMES" ]]; then
    CMD+=( "--max_frames" "$MAX_FRAMES" )
fi

echo "Running timing benchmark..."
echo "Command: ${CMD[*]}"
"${CMD[@]}"

# --- Done ---
cd "$CURR_PATH"
echo "✅ Timing benchmarks completed successfully"
