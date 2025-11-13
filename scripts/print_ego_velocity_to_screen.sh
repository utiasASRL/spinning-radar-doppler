#!/bin/bash
# -----------------------------------------------------------------------------
# Script: print_ego_velocity_to_screen.sh
#
# Description:
#   Simply estimates and prints the radar ego-velocity from extracted Doppler
#   measurements.
#
# Behavior:
#   - Reads radar data from the provided input path.
#   - Saves the generated video in the same directory as the input data.
#
# Assumptions:
#   Input radar data includes chirp-type metadata.
# -----------------------------------------------------------------------------

set -euo pipefail # Exit on errors, unset variables are errors, catch errors in pipes

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <sequence_path>"
    exit 1
fi

SEQUENCE_PATH="$1"

# --- Resolve paths ---
INPUT_PATH="$(realpath "$SEQUENCE_PATH")"
OUTPUT_PATH="$INPUT_PATH"  # Not actually generating CSV so doesn't matter
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

if [ ! -x "$BUILD_PATH/apps/generate_ego_velocity_csv" ]; then
    echo "Error: binary not found or not executable: $BUILD_PATH/apps/generate_ego_velocity_csv"
    exit 1
fi

if [ ! -f "$CONFIG_PATH" ]; then
    echo "Error: config file not found: $CONFIG_PATH"
    exit 1
fi

# --- Step 1: Generate Doppler velocities CSV ---
cd "$BUILD_PATH"
CSV_PATH="$OUTPUT_PATH/doppler_velocities.csv"
echo "Running velocity estimation..."
./apps/generate_ego_velocity_csv --no_save --verbose "$INPUT_PATH" "$CSV_PATH" "$CONFIG_PATH"

# --- Done ---
cd "$CURR_PATH"
echo "✅ Doppler velocity video generated successfully at: $OUTPUT_PATH"
