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

 
 
ver=`./dmcam-cli_static143 --print info |grep "MCU" |awk -F' ' 'NR==1{print $5}'`
hardver=`./dmcam-cli_static143 --print info |grep "Its Hardware" |awk -F' ' 'NR==1{print $5}'`
tfcver=`./dmcam-cli_static143 --print info |grep "TFC Firmware" |awk -F' ' 'NR==1{print $5}'`

# ver=146
# hardver=30

echo "now mcu version is $ver,hardver is $hardver,tfcver is $tfcver"

if [ $tfcver -gt 0 -a $tfcver -le 145 ]; then
	echo "your version is not correct for support,please contact the FAE for help"
else
	if [ $ver -le 131 ]; then
		
		echo "your version is too old,please first to the uploaddir to upgrade to 143,the do the next upgrade"
	else
		if [ $ver -le 143 -a $hardver -eq 20 ]; then
			echo "use in package_upgrade"
			cd $package_DIR
			./upgrade.sh
		else
			if [ $hardver -eq 20 -a $ver -gt 143 ]; then
				echo "use in nopack_HW20"
				cd $nopackHW20_DIR
				./upgrade.sh
			else
				if [ $hardver -eq 30 ]; then
					echo "use in nopack_HW30"
					cd $nopackHW30_DIR
					./upgrade.sh
				else
					echo "not support upgrade,please contact the FAE"
				fi
			fi
		fi
	fi
fi








