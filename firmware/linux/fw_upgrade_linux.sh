#! /bin/sh
SCRIPT_DIR=$(cd `dirname $0`;pwd)

TARGET_DIR=$SCRIPT_DIR

package_DIR=$SCRIPT_DIR/package_upgrade
nopackHW20_DIR=$SCRIPT_DIR/nopack_HW20
nopackHW30_DIR=$SCRIPT_DIR/nopack_HW30


die(){
    echo "ERROR:$@"	
    exit 255
}
# echo "`pwd`"

cd $TARGET_DIR

echo "`pwd`"

# ./dmcam-cli_static143 --print info
product=`./dmcam-cli_static143 --print info |grep "Product" |awk -F' ' 'NR==1{print $3}'`
echo "get product is $product"
if [ "$product"x = "TM-E2-1.0"x ]; then
	# ./dmcam-cli_static143 --print info
	echo "upgrade E2"
	./dmcam-cli -f TC-General-1.0_HW30*.bin
else
	if [ "$product"x = "TM-E3-1.0"x ]; then
		echo "upgrade E3"
		./dmcam-cli -f TC-General-2.0*.bin
	else
		if [ "$product"x = "TM-E4-1.0"x ]; then
			echo "upgrade E4"
			./dmcam-cli -f TC-General-1.0*.bin
		else
			echo "there's no suitable file for upgrade,please contact FAE"
		fi
	fi
fi








