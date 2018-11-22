#! /bin/sh
SCRIPT_DIR=$(cd `dirname $0`;pwd)

TARGET_DIR=$SCRIPT_DIR


die(){
    echo "ERROR:$@"	
    exit 255
}

cd $TARGET_DIR

echo "`pwd`"

# export LD_LIBRARY_PATH=./
./dmcam-cli -f TM-E2_HW30_SW158_20181120.bin








