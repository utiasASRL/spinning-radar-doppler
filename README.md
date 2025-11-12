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

### Install c++ compiler, cmake, and yaml support

```bash
sudo apt -q -y install build-essential cmake libomp-dev libyaml-cpp-dev
```

### Install Eigen (>=3.3.7)

```bash
# using APT
sudo apt -q -y install libeigen3-dev

# OR from source
WORKSPACE=~/workspace  # choose your own workspace directory
mkdir -p ${WORKSPACE}/eigen && cd $_
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

## Applications

The Doppler extractor is meant to be installed and used in other projects. However, some common applications are provided locally for development, processing, and visualization purposes.
They are all contained under the `apps` directory with pre-packaged uses contained in the `scripts` directory.
The `scripts` directory contains both direct application calls and combinations of application calls that together server some other purpose.
Each script contains a description of its purpose at the top.