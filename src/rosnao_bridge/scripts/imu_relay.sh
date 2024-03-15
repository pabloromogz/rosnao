#!/bin/bash
SCRIPT_DIR=$(dirname `echo $(realpath "$0")`)
source `echo $SCRIPT_DIR`/get_bin_dir.sh

# Launches the imu proxy in Nao, and prints the imu data
(cd `echo $BIN_DIR`; chmod +x imu_proxy; ./imu_proxy)
exit 0