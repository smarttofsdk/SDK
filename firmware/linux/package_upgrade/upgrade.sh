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

 
echo "be careful there are 3 steps to upgrade,do not shutdown the window during upgrading"
echo "...............step 1 of all 3................"
./dmcam-cli_static143 -e version
 
export LD_LIBRARY_PATH=./
./bin_data_upgrade 1

echo "...............step 2 of all 3................"
./dmcam-cli_static143 -f TM-E2_HW20_SW156_20180921.bin
echo "do not close"
#wait 
sleep 10

export LD_LIBRARY_PATH=./
./bin_data_upgrade 6
echo "do not close"
sleep 10

echo "...............step 3 of all 3................"
export LD_LIBRARY_PATH=./
./bin_data_upgrade 2
echo "now upgrade finished"




# ver=`./dmcam-cli_static143 --print info |grep "MCU" |awk -F' ' 'NR==1{print $5}'`
 
# echo "now mcu version is $ver"

# ./dmcam-cli_static143 -e version

# ./bin_data_upgrade 1

# ./dmcam-cli_static143 -f TM-E2_HW20_SW150_20180809.bin

# #wait 
# sleep 10

# ./bin_data_upgrade 6

# sleep 10

# ./bin_data_upgrade 2



# if [ $ver -le 131 ]; then
    # echo "use cli131"
    # # ./dmcam-cli-131 -f $upgradebin
	# ./dmcam-cli-131 -f "TM-E2_HW20_SW143_20180724.bin"
# else
    # echo "use cli140"
    # # ./dmcam-cli-140 -f $upgradebin
	 # ./dmcam-cli-140 -f "TM-E2_HW20_SW143_20180724.bin"
# fi

# ./dmcam-cli_143static -e version

# ./pk_old_calibration_data

# ./dmcam-cli_143static -f TM-E2_HW20_SW147_20180801.bin

# # wait
# sleep 60 

# ./rest_module

# sleep 10

# ./download_calibration_data








