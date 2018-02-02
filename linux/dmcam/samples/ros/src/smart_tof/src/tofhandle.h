/*****************************************************************//**
 *       @file  tofhandle.h
 *      @brief  DM's camera device API
 *
 *  Detail Decsription starts here
 *
 *   @internal
 *     Project  $Project$
 *     Created  6/15/2017 
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
#ifndef TOFHANDLE_H
#define TOFHANDLE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <sys/types.h>
#include <getopt.h>
#include <stdbool.h>
#include <unistd.h>

#include "pthread.h"
#include "dmcam.h"
#include "function.h"

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


#define FRAME_SIZE (320*240*2*4)
//switch to chmod tof device
#define NEEDSUDO 1

typedef pcl::PointCloud<pcl::PointXYZ> pclPointCloudXYZ;

static bool on_cap_err(dmcam_dev_t *_dev, int _err, void *_err_arg);


class TofHandle
{
public:
    TofHandle(ros::NodeHandle *_n);
    ~TofHandle(void);
    //publish all topics, image_transport::ImageTransport *_it
    void pub_all(void);
    //get tof state(can it be used?)
    bool get_tof_state(void);
    //get test mode state
    bool get_test_mode(void);
    //call back for server
    bool change_power(smart_tof::change_power::Request& req, smart_tof::change_power::Response& res);
    bool change_intg(smart_tof::change_intg::Request& req, smart_tof::change_intg::Response& res);

private:
    //node handler from main
    ros::NodeHandle *nh;
    //image transport handle
    image_transport::ImageTransport *it;
    //the opened device
    dmcam_dev_t *dev;
    //0 means not good, and will do nothing later on
    bool tof_state;
    //switch of showing picture and some test information
    bool test_mode;
    //frame info
    dmcam_frame_t fbuf_info;
    //buffer for storing raw data from tof
    uint8_t *fbuf;
    uint8_t *dist_pseudo_rgb;
    //buffer for storing gray\dist\amb image temporarily
    float *exchange;
    //array to store point cloud
    float *pcl_exchange;
    //parameters of tof
    uint32_t dev_mode;
    uint32_t mod_freq;
    dmcam_param_roi_t roi;
    uint32_t format;
    uint8_t power_percent;
    uint32_t fps;
    uint16_t intg_us;
    //parameters of camera
    CAMERA_INTRINSIC_PARAMETERS cam_int_param;
    //publisher
    image_transport::Publisher image_gray_pub;
    image_transport::Publisher image_dist_pub;
    image_transport::Publisher image_amb_pub;
    ros::Publisher cam_info_pub;
    ros::Publisher pcl_pub;
    //message
    sensor_msgs::CameraInfo cam_inf_msg;
    std_msgs::Header img_head_msg;
    sensor_msgs::PointCloud2  cloud_msg;
    //server
    ros::ServiceServer change_power_ser;
    ros::ServiceServer change_intg_ser;

    //get one frame from tof
    void get_one_frame(void);
    //initialize tof
    void tof_init(void);
    //initialize parameters
    void param_init(dmcam_dev_t *dev_0);
    //initialize topics
    void topic_init(void);
    //init server
    void server_init(void);
    //sudo device
    void sudodev(dmcam_dev_t *_dev);
    //publish image(including point cloud inside)
    void pub_image(void);
    //publish point cloud
    void pub_pcl(const cv::Mat& _dist);
    //publish camera info
    void pub_caminfo(void);
};

#endif // TOFHANDLE_H
