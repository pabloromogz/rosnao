
NAO_IP=192.168.23.156
SHM_ID=img
SHM_ID2=motion
RES=1 #1 for QVGA, 2 for VGA
CAM=0 #0 for bottom, 1 for top
TOPIC="cam0/image_raw"
FRAME="world"

SCRIPT_DIR=$(dirname `echo $(realpath "$0")`)
source `echo $SCRIPT_DIR`/devel/setup.bash

roslaunch rosnao_bridge motion_image_relay.launch nao_ip:=`echo $NAO_IP` shm_id:=`echo $SHM_ID` shm_id2:=`echo $SHM_ID2` res:=`echo $RES` cam:=`echo $CAM` topic:=`echo $TOPIC` frame:=`echo $FRAME`