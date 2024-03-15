NAO_IP=192.168.225.156
SHM_ID1=img1
SHM_ID2=img2
RES=1 #1 for QVGA, 2 for VGA, 3 for 4VGA(1280*960)
TOPIC1="cam0/image_raw"
TOPIC2="cam1/image_raw"
FRAME="world"

SCRIPT_DIR=$(dirname `echo $(realpath "$0")`)
source `echo $SCRIPT_DIR`/devel/setup.bash

roslaunch rosnao_bridge image_relay_stereo.launch nao_ip:=`echo $NAO_IP` shm_id1:=`echo $SHM_ID1` shm_id2:=`echo $SHM_ID2` res:=`echo $RES` topic1:=`echo $TOPIC1` topic2:=`echo $TOPIC2` frame:=`echo $FRAME`