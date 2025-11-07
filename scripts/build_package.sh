#!/bin/bash

SCRIPT_PATH=$(dirname "$(realpath "$0")")
BUILD_DIR=$SCRIPT_PATH/../build
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p $BUILD_DIR
fi

cd $BUILD_DIR

cmake -DCMAKE_BUILD_TYPE=Release .. && make -j4