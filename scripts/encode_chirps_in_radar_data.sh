#!/bin/bash
# Script to encode chirp direction in the 10th byte of the raw radar data.
# Data is read in from a provided sequence path and an updated version is output
# either in-place (if --overwrite_input is set) or at the same level
# but under a folder called 'doppler_radar' otherwise. Additionally takes a
# --verbose flag to enable detailed logging inside the C++ app.
# Config file is read from ../config/doppler_config.yaml.
# ASSUMPTION: The radar is only moving forward!

set -euo pipefail # Exit on errors, unset variables are errors, catch errors in pipes

OVERWRITE_INPUT=false
VERBOSE=false
SEQUENCE_PATH=""

# --- Parse inputs ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --overwrite_input)
            OVERWRITE_INPUT=true
            ;;
        --verbose)
            VERBOSE=true
            ;;
        -h|--help)
            echo "Usage: $0 [--overwrite_input] [--verbose] <sequence_path>"
            echo ""
            echo "Options:"
            echo "  --overwrite_input   Overwrite the input sequence in-place."
            echo "  --verbose           Enable detailed logging inside the C++ app."
            exit 0
            ;;
        -*)
            echo "Unknown option: $1"
            echo "Usage: $0 [--overwrite_input] [--verbose] <sequence_path>"
            exit 1
            ;;
        *)
            # Positional argument (the sequence path)
            if [ -z "$SEQUENCE_PATH" ]; then
                SEQUENCE_PATH="$1"
            else
                echo "Error: multiple sequence paths provided: '$SEQUENCE_PATH' and '$1'"
                echo "Usage: $0 [--overwrite_input] [--verbose] <sequence_path>"
                exit 1
            fi
            ;;
    esac
    shift
done

# --- Validate required positional argument ---
if [ -z "$SEQUENCE_PATH" ]; then
    echo "Usage: $0 [--overwrite_input] [--verbose] <sequence_path>"
    exit 1
fi

INPUT_PATH="$(realpath "$SEQUENCE_PATH")"
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_PATH="$(realpath "$SCRIPT_PATH/..")"
CONFIG_PATH="$PROJECT_PATH/config/doppler_config.yaml"
BUILD_PATH="$PROJECT_PATH/build"
CURR_PATH="$(pwd)"

if [ "$OVERWRITE_INPUT" = true ]; then
    OUTPUT_PATH="$INPUT_PATH"
    echo "Overwriting input data in-place."
else
    OUTPUT_PATH="$(dirname "$INPUT_PATH")/doppler_radar"
fi

echo "Processing radar scans for sequence: $INPUT_PATH"
echo "Output directory: $OUTPUT_PATH"

# --- Sanity checks ---
if [ ! -d "$INPUT_PATH" ]; then
    echo "Error: input path does not exist: $INPUT_PATH"
    exit 1
fi

if [ ! -x "$BUILD_PATH/apps/encode_chirps_in_radar_data" ]; then
    echo "Error: binary not found or not executable: $BUILD_PATH/apps/encode_chirps_in_radar_data"
    exit 1
fi

if [ ! -f "$CONFIG_PATH" ]; then
    echo "Error: config file not found: $CONFIG_PATH"
    exit 1
fi

# --- Process scans ---
cd "$BUILD_PATH"
if [ "$VERBOSE" = true ]; then
    ./apps/encode_chirps_in_radar_data --verbose "$INPUT_PATH" "$OUTPUT_PATH" "$CONFIG_PATH"
else
    ./apps/encode_chirps_in_radar_data "$INPUT_PATH" "$OUTPUT_PATH" "$CONFIG_PATH"
fi

# --- Done ---
cd "$CURR_PATH"
echo "✅ Chirp encoding complete. Results saved to: $OUTPUT_PATH"