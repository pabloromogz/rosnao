#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
(cd `echo $SCRIPT_DIR`/src/rosnao_wrapper; qibuild configure; qibuild make)


#BUILD_DIR=`echo $SCRIPT_DIR`/rosnao_wrapper/build
# mkdir -p `echo $BUILD_DIR`
# rm -rf `echo $BUILD_DIR`/*
# cd `echo $BUILD_DIR`
# cmake ..
# cmake --build .