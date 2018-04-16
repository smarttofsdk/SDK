#!/bin/bash
# ubuntu 16.04/14.04 -> you can chose to install kinetic or indigo,kinetic is recommended.
#other operating system ->install kinetic by default
# created by liuguiyu
 

function InstallIndigo ()
{
 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 
 sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
 
 sudo apt-get update
 
 sudo apt-get install ros-indigo-desktop-full -y
 
 sudo rosdep init
 rosdep update
 
 echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 
 sudo apt-get install python-rosinstall
}
 
function InstallKinetic ()
{
 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 
 sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
 
 sudo apt-get update
 
 sudo apt-get install ros-kinetic-desktop-full -y
 
 sudo rosdep init
 rosdep update
 
 echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 
 sudo apt-get install python-rosinstall
}
 
 
# check os version
if cat /etc/lsb-release | grep -i "\b 14.04 \b" >/dev/null 2>&1
 then
 OsVersion="14.04"
 echo "u Os Version is $OsVersion"
elif cat /etc/lsb-release | grep -i "\b 14.10\b" >/dev/null 2>&1
 then
 OsVersion="14.10"
 echo "u Os Version is $OsVersion"
elif cat /etc/lsb-release | grep -i "\b 16.04\b" >/dev/null 2>&1
 then
 OsVersion="16.04"
 echo "u Os Version is $OsVersion"
else
 OsVersion=OsVersion=$(cat /etc/issue)
 echo "u Os Version is $OsVersion"
 echo "i suggest u to choose your os to Ubuntu 14.04 or 16.04"
 return 1
fi
 
 
 
# choose ros version
if [ "$OsVersion" == "16.04" ]
 then
 echo -n "Your os version is $OsVersion,please input kinetic or indigo u want to install:"
 read RosVersion
 RosVersion=$(echo "$RosVersion" | tr A-Z a-z);
 echo "Your choose is $RosVersion"
else
 RosVersion="kinetic"
 echo "Your choose is $RosVersion"
fi
 
# check ros version & install
if [ "$RosVersion" == "indigo" ]
 then
 echo "start to install $RosVersion"
 InstallIndigo
elif [ "$RosVersion" == "kinetic" ]
 then
 echo "start to install $RosVersion"
 InstallKinetic
else
 echo "Your choose is $RosVersion is not availavle"
 return 1
fi
 

