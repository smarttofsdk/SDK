#! /bin/sh
SCRIPT_DIR=$(cd `dirname $0`;pwd)

TARGET_DIR=$SCRIPT_DIR
# FINDID=111b:1238
# FINDVERSION="MCU Firmware Version : 112"
upgradebin=./TM-E2_HW20_SW143_20180724.bin

die(){
    echo "ERROR:$@"	
    exit 255
}
# echo "`pwd`"

cd $TARGET_DIR

echo "`pwd`"

 
 # ./dmcam-cli -f $upgradebin
 # ./dmcam-cli -e "flash mcu "$upgradebin""
 
ver=`./dmcam-cli-140 --print info |grep "MCU" |awk -F' ' 'NR==1{print $5}'`
 
echo "now mcu version is $ver"


if [ $ver -le 131 ]; then
    echo "use cli131"
    # ./dmcam-cli-131 -f $upgradebin
	./dmcam-cli-131 -f "TM-E2_HW20_SW143_20180724.bin"
else
    echo "use cli140"
    # ./dmcam-cli-140 -f $upgradebin
	 ./dmcam-cli-140 -f "TM-E2_HW20_SW143_20180724.bin"
fi









