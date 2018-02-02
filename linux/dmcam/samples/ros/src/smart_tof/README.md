# smart_tof
ros package for smart tof camera(ver 0.1.1, this is a release version)

by **Xue Wuyang** and **Liu Guiyu**.

## Requirement:
1. ubuntu(16.04 recommended)
2. ros(kinetic recommended)

## Attention:
1. If you don't want to type in password every time you use tof, try [this method](http://blog.csdn.net/lina_acm/article/details/52080448). Then find NEEDSUDO definition in TofHandle.h and define it 0.
2. If you want to see point cloud in rviz, you must type "smart_tof" into "Global Options/Fixed Frame" in rviz and then add point cloud topic into displays.

## Environment configuration
1. install ros 
(If you have installed ros on your computer, skip this step.)
Enter the package directory in your terminal shell.then run:  
> sudo chmod 755 install_ros.sh  
> ./install_ros.sh
2. build catkin_ws(If you have built catkin_ws directory in your home directory, skip this step.):  
> sudo chmod 755 build_catkin_ws.sh  
> ./build_catkin_ws.sh

## Usage:
1. clone this repository into your catkin_ws/src folder
2. > cd catkin_ws
3. > catkin_make -DCATKIN_WHITELIST_PACKAGES="smart_tof"  
4. initial parameters can be modify in parameters/initial_param.yaml
5. > roscore
6. > roslaunch smart_tof start.launch

## Provided by this package:
1. image of gray, distance and amplification
2. camera information
3. point cloud

## Test environment:
1. ubuntu 12.04, 14.04, 16.04
2. ros indigo, kinetic
