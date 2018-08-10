#! /bin/sh
SCRIPT_DIR=$(cd `dirname $0`;pwd)

TARGET_DIR=$SCRIPT_DIR

die(){
    echo "ERROR:$@"	
    exit 255
}
# echo "`pwd`"

cd $TARGET_DIR

echo "`pwd`"


./dmcam-cli -f TM-E2_HW20_SW150_20180809.bin








