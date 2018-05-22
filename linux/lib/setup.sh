#! /bin/sh
SCRIPT_DIR=$(cd `dirname $0`;pwd)

TARGET_DIR=$SCRIPT_DIR
DEST_DIR=/etc/udev/rules.d
LIB_DIR64=$TARGET_DIR/linux64
LIB_DIR32=$TARGET_DIR/linux32

die(){
    echo "ERROR:$@"	
    exit 255
}
echo "`pwd`"

cd $TARGET_DIR
#build rules
touch 99-persistent-usb.rules
echo "SUBSYSTEMS==\"usb\",ATTRS{idVendor}==\"111b\",ATTRS{idProduct}==\"1238\",GROUP=\"plugdev\",MODE=\"0666\"" >99-persistent-usb.rules


sudo cp 99-persistent-usb.rules $DEST_DIR/99-persistent-usb.rules
sudo chmod a+r /etc/udev/rules.d/99-persistent-usb.rules
sudo service udev restart
rm -rf 99-persistent-usb.rules

lsusb

#cp libdmcam.so
sysbit=`getconf LONG_BIT`
if [ $sysbit -eq 64 ] ; then
	echo "your system is 64bit"
	cd $LIB_DIR64
	pwd
	sudo cp libdmcam.so /usr/lib/libdmcam.so
else
    echo "your system is 32bit"
	cd $LIB_DIR32
	sudo cp libdmcam.so /usr/lib/libdmcam.so
fi	





