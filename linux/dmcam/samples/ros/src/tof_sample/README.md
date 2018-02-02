tof_sample

This is a simple sample for using smart tof in ros with package smart_tof. You can find out how to get image and point cloud from package smart_tof's topic and change some parameters of tof using service provided by package smart_tof.

by Xue Wuyang

Requirement:
1. package smart_tof and what package smart_tof requires

Usage:
put the tof_sample in the same catkin work space as smart_tof,
then there are two ways to run this sample:

a. run directory(follow a1 to a3)

a.1. catkin_make -DCATKIN_WHITELIST_PACKAGES="tof_sample"

a.2. run the smart_tof package(like: roslaunch smart_tof start.launch)

a.3. rosrun tof_sample tof_sample

or

b. run with launch file(follow b1 to b2)

b.1. catkin_make -DCATKIN_WHITELIST_PACKAGES="tof_sample"

b.2. roslaunch tof_sample start.launch

then you will see gray image and point cloud displayed, and the intg time will change between 100 and 200 every 5 seconds.


