#!/bin/bash
# Script to create a virtual environment at the project root and install required Python packages.

set -euo pipefail # Exit on errors, unset variables are errors, catch errors in pipes

# Set up path variables for easy reference
CURR_PATH=$(pwd)
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_PATH="$(realpath "$SCRIPT_PATH/..")"

echo "Creating virtual environment in: $PROJECT_PATH/venv"

cd "$PROJECT_PATH"

# Create venv if it doesn't already exist
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo "Virtual environment created."
else
    echo "Virtual environment already exists. Skipping creation."
fi

# Activate and install dependencies
source venv/bin/activate
pip install --upgrade pip
if [ -f requirements.txt ]; then
    pip install -r requirements.txt
else
    echo "Warning: requirements.txt not found."
fi

cd "$CURR_PATH"
echo "Done."
