#!/bin/bash

# Script to encode chirp direction in the 10th byte of the raw radar data.
# Data is read in from a provided sequence path and an updated version is output
# either in-place (if --overwrite_input is set) or at the same level
# but under a folder called 'doppler_radar' otherwise.
# Config file is read from ../config/doppler_config.yaml.
# ASSUMPTION: The radar is only moving forward!

OVERWRITE_INPUT=false

# Parse optional flags
while [[ "$1" =~ ^- ]]; do
    case "$1" in
        --overwrite_input)
            OVERWRITE_INPUT=true
            shift
            ;;
        -h|--help)
            echo "Usage: ./encode_chirps_in_radar_data.sh [--overwrite_input] <sequence_path>"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: ./encode_chirps_in_radar_data.sh [--overwrite_input] <sequence_path>"
            exit 1
            ;;
    esac
done

# Require the remaining positional argument
if [ "$#" -ne 1 ]; then
    echo "Usage: ./encode_chirps_in_radar_data.sh [--overwrite_input] <sequence_path>"
    exit 1
fi

INPUT_PATH=$1
OUTPUT_PATH="$(dirname "$INPUT_PATH")/doppler_radar"

# Handle overwrite flag
if [ "$OVERWRITE_INPUT" = true ]; then
    OUTPUT_PATH="$INPUT_PATH"
    echo "Overwriting input data in-place."
fi

echo "Processing radar scans for sequence: $INPUT_PATH"
echo "Output directory: $OUTPUT_PATH"

CURR_PATH=$(pwd)
SCRIPT_PATH=$(dirname "$0")

# Save config file path
cd "$SCRIPT_PATH/../config"
CONFIG_PATH=$(pwd)/doppler_config.yaml
cd "$CURR_PATH"

# Process scans
cd "$SCRIPT_PATH/../build"
./apps/encode_chirps_in_radar_data "$INPUT_PATH" "$OUTPUT_PATH" "$CONFIG_PATH"

# Return home
cd "$CURR_PATH"