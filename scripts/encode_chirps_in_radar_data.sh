#!/bin/bash

# Script to encode chirp direction in the 10th byte of the raw radar data.
# Data is read in from a provided sequence path and output at the same level
# but under a folder called 'doppler_radar'. Config file is read from
# ../config/doppler_config.yaml.
# ASSUMPTION: The radar is only moving forward!

if [ "$#" -ne 1 ]; then
    echo "Usage: ./encode_chirps_in_radar_data.sh <sequence_path>"
    exit 1
fi

INPUT_PATH=$1
OUTPUT_PATH="$(dirname "$INPUT_PATH")/doppler_radar"

echo "Processing radar scans for sequence: $INPUT_PATH"
echo "Output directory: $OUTPUT_PATH"

CURR_PATH=$(pwd)
SCRIPT_PATH=$(dirname $0)

# Save config file path
cd $SCRIPT_PATH/../config
CONFIG_PATH=$(pwd)/doppler_config.yaml
cd $CURR_PATH

# Process scans
cd $SCRIPT_PATH/../build
./apps/encode_chirps_in_radar_data $INPUT_PATH $OUTPUT_PATH $CONFIG_PATH

# Return home
cd $CURR_PATH