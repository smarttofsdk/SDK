/*****************************************************************//**
 *       @file  tof_smaple.cpp
 *      @brief  DM's camera device API
 *
 *  Detail Decsription starts here
 *
 *   @internal
 *     Project  $Project$
 *     Created  3/13/2017 
 *    Revision  $Id$
 *     Company  Data Miracle, Shanghai
 *   Copyright  (C) 2017 Data Miracle Intelligent Technologies
 *    
 *    THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *    KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *    PARTICULAR PURPOSE.
 *
 * *******************************************************************/
/*
 *a sample to use tof in ros with smart_tof
 *
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <sys/types.h>
#include <getopt.h>
#include <stdbool.h>
#include <unistd.h>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/distortion_models.h"
#include "sensor_msgs/CameraInfo.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include "smart_tof/change_intg.h"
#include "smart_tof/change_power.h"

pcl::visualization::CloudViewer pclviewer("Cloud Viewer");

void img_gray_callback(const sensor_msgs::ImageConstPtr& msg)
{
    //for dist img you may replace "mono8" with sensor_msgs::image_encodings::TYPE_32FC1 
    cv::Mat img = cv_bridge::toCvCopy(msg, "mono8")->image;
    cv::imshow("image gray got", img);
    cv::waitKey(1);
}

void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    pclviewer.showCloud(pc);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tof_smaple");
    ros::NodeHandle nh;
    //setting running rate as 30Hz
    ros::Rate loop_rate(30);
    //service client for intg time
    ros::ServiceClient client = nh.serviceClient<smart_tof::change_intg>("/smarttof/change_intg");
    smart_tof::change_intg intg_srv;

    //image subscriber
    image_transport::ImageTransport it(nh);
    //image_transport::Subscriber imggray_sub = it.subscribe("/smarttof/image_gray", 1, img_gray_callback);

    //point cloud subcriber
    ros::Subscriber pcl_sub = nh.subscribe("smarttof/pointcloud", 1, pcl_callback);

    int cnt = 0;
    while(ros::ok())
    {
        //call service to change intg time every 5 seconds
        if(++cnt == 150)
        {
            intg_srv.request.a = 700;
            client.call(intg_srv);ROS_INFO("set intg 700!");
        }
        else if(cnt == 300)
        {
            intg_srv.request.a = 1000;
            client.call(intg_srv);ROS_INFO("set intg 1000!");
            //cnt = 0;
        }
        //necessary for running callback
        ros::spinOnce();
        //sleep to fit the running rate
        loop_rate.sleep();
    }

    return 1;
}
