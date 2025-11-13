![Tests](https://github.com/utiasASRL/spinning-radar-doppler/actions/workflows/build_and_test.yml/badge.svg)

# spinning-radar-doppler
Code to extract and process Doppler measurements from a spinning radar sensor.

## Expected input format
The code currently expects that your radar data is contained as full radar scan images in a .png format.
Each image has a name corresponding to the unix timestamp at which it was collected at.

## Installation

### Dependencies

- Compiler with C++17 support
- CMake (>=3.16)
- Eigen (>=3.3.7)
- [lgmath (>=1.1.0)](https://github.com/utiasASRL/lgmath.git)
- Python3 (if Python visualizations are desired)

### Install c++ compiler, cmake, and yaml support

```bash
sudo apt -q -y install build-essential cmake libomp-dev libyaml-cpp-dev
```

### Install Eigen (>=3.3.7)

```bash
# using APT
sudo apt -q -y install libeigen3-dev

# OR from source
# Navigate to your desired workspace
git clone https://gitlab.com/libeigen/eigen.git . && git checkout 3.3.7
mkdir build && cd $_
cmake .. && make install # default install location is /usr/local/
```

- Note: if installed from source to a custom location then make sure `cmake` can find it.

### Install lgmath

Follow the instructions [here](https://github.com/utiasASRL/lgmath.git).

### Build and install spinning-radar-doppler

```bash
# Navigate to your desired workspace
git clone git@github.com:utiasASRL/spinning-radar-doppler.git
# build and install
bash scripts/build_package.sh
```

If Python visualization tools are desired, create a virtual environment by running

```bash
bash scripts/create_venv.sh
```

## Applications

The Doppler extractor is meant to be installed and used in other projects. However, some common applications are provided locally for development, processing, and visualization purposes.
They are all contained under the `apps` directory with pre-packaged uses contained in the `scripts` directory.
The `scripts` directory contains both direct application calls and combinations of application calls that together server some other purpose.
Each script contains a description of its purpose at the top.

### Encoding Chirps

If your Doppler-enabled radar data has an unknown chirp type, the `encode_chirps_in_radar_data` application is made to populate the chirp metadata.
Currently, this application assumes that the radar experiences some forward-only motion during the run.
The chirp type is populated into the unused 10th byte of the radar image.
To populate this metadata, run

```bash
DATA_PATH=your/data/here
bash scripts/encode_chirps_in_radar_data.sh --overwrite_input $DATA_PATH
```

Make sure to update the radar parameters in the `config/doppler_config.yaml` file to match your radar hardware.

### Visualizing Doppler Velocities

Once you have radar data with an encoded chirp type in the 10th byte of the image, the `generate_doppler_velocity_csv` application can be used to generate a CSV containing extracted Doppler velocities.
This CSV can then be used for other purposes, but a Python-based visualization script is included.
To run both parts together, the `scripts/generate_doppler_velocity_video.sh` script is provided. Detailsa about the script can be found at the top of the file.

Make sure to set/tune all relevant parameters in the `config/doppler_config.yaml` config file.
Additionally, ensure that the Python virtual environment is created by running `bash scripts/create_venv.sh`.