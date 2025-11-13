#!/bin/bash
# -----------------------------------------------------------------------------
# Script: create_doppler_video.sh
#
# Description:
#   Creates a video overlaying Doppler velocities on radar scans.
#
# Behavior:
#   - Reads radar data from the provided input path.
#   - Saves the generated video in the same directory as the input data.
#   - Supports optional flags:
#       • --verbose   → Enables detailed logging in the C++ Doppler generator.
#       • --no_ransac → Disables RANSAC during Doppler velocity estimation.
#
# Assumptions:
#   Input radar data includes chirp-type metadata.
# -----------------------------------------------------------------------------

set -euo pipefail # Exit on errors, unset variables are errors, catch errors in pipes

NO_RANSAC=false
VERBOSE=false
SEQUENCE_PATH=""

# --- Parse inputs ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --verbose)
            VERBOSE=true
            ;;
        --no_ransac)
            NO_RANSAC=true
            ;;
        -h|--help)
            echo "Usage: $0 [--verbose] [--no_ransac] <sequence_path>"
            echo ""
            echo "Options:"
            echo "  --verbose     Enable detailed logging inside the C++ app."
            echo "  --no_ransac   Disable RANSAC filtering when computing Doppler velocities."
            exit 0
            ;;
        -*)
            echo "Unknown option: $1"
            echo "Usage: $0 [--verbose] [--no_ransac] <sequence_path>"
            exit 1
            ;;
        *)
            # Positional argument (the sequence path)
            if [ -z "$SEQUENCE_PATH" ]; then
                SEQUENCE_PATH="$1"
            else
                echo "Error: multiple sequence paths provided: '$SEQUENCE_PATH' and '$1'"
                echo "Usage: $0 [--verbose] [--no_ransac] <sequence_path>"
                exit 1
            fi
            ;;
    esac
    shift
done

# --- Validate required positional argument ---
if [ -z "$SEQUENCE_PATH" ]; then
    echo "Usage: $0 [--verbose] [--no_ransac] <sequence_path>"
    exit 1
fi

INPUT_PATH="$(realpath "$SEQUENCE_PATH")"
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_PATH="$(realpath "$SCRIPT_PATH/..")"
CONFIG_PATH="$PROJECT_PATH/config/extractor_config.yaml"
BUILD_PATH="$PROJECT_PATH/build"
PYTHON_SCRIPT="$PROJECT_PATH/python/generate_video_from_csv.py"
OUTPUT_PATH="$INPUT_PATH"  # save video in same directory
CURR_PATH="$(pwd)"

echo "Processing radar scans for sequence: $INPUT_PATH"
echo "Output directory: $OUTPUT_PATH"

# --- Sanity checks ---
if [ ! -d "$INPUT_PATH" ]; then
    echo "Error: input path does not exist: $INPUT_PATH"
    exit 1
fi

if [ ! -x "$BUILD_PATH/apps/generate_doppler_velocity_csv" ]; then
    echo "Error: binary not found or not executable: $BUILD_PATH/apps/generate_doppler_velocity_csv"
    exit 1
fi

if [ ! -f "$CONFIG_PATH" ]; then
    echo "Error: config file not found: $CONFIG_PATH"
    exit 1
fi

if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Error: Python script not found: $PYTHON_SCRIPT"
    exit 1
fi

# --- Step 1: Generate Doppler velocities CSV ---
cd "$BUILD_PATH"
CSV_PATH="$OUTPUT_PATH/doppler_velocities.csv"

CMD=(./apps/generate_doppler_velocity_csv "$INPUT_PATH" "$CSV_PATH" "$CONFIG_PATH")
if [ "$VERBOSE" = true ]; then
    CMD+=("--verbose")
fi
if [ "$NO_RANSAC" = true ]; then
    CMD+=("--no_ransac")
fi

echo "Running Doppler velocity extraction..."
"${CMD[@]}"

# --- Step 2: Create video using Python script ---
cd "$PROJECT_PATH/python"
echo "Generating video..."
python3 "$PYTHON_SCRIPT" \
    --radar_data_path "$INPUT_PATH" \
    --csv_path "$CSV_PATH" \
    --output_path "$OUTPUT_PATH" \
    --config_path "$CONFIG_PATH"

# --- Done ---
cd "$CURR_PATH"
echo "✅ Doppler velocity video generated successfully at: $OUTPUT_PATH"
