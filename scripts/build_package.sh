#!/bin/bash

SCRIPT_PATH=$(dirname "$(realpath "$0")")
BUILD_DIR=$SCRIPT_PATH/../build
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p $BUILD_DIR
fi

cd $BUILD_DIR

cmake ..
cmake -DCMAKE_BUILD_TYPE=Release .. -DSRD_BUILD_TESTS=ON -DSRD_BUILD_APPS=ON && make -j4