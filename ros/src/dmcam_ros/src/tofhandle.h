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
#include <map>

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

#include "dmcam_ros/change_intg.h"
#include "dmcam_ros/change_power.h"
#include "dmcam_ros/change_frame_rate.h"
#include "dmcam_ros/change_mod_freq.h"
#include "dmcam_ros/change_sync_delay.h"
#include "dmcam_ros/change_filter.h"
#include "dmcam_ros/disable_filter.h"

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
    void publicAll(void);
    //get tof state(can m_imageTransport be used?)
    bool getTofState(void);
    //get test mode state
    bool getTestMode(void);
    //call back for server
    bool changeParameters(dmcam_param_item_t params);
    bool changePower(dmcam_ros::change_power::Request& req, dmcam_ros::change_power::Response& res);
    bool changeIntgTime(dmcam_ros::change_intg::Request& req, dmcam_ros::change_intg::Response& res);
    bool changeFrameRate(dmcam_ros::change_frame_rate::Request& req, dmcam_ros::change_frame_rate::Response& res);
    bool changeModFreq(dmcam_ros::change_mod_freq::Request& req, dmcam_ros::change_mod_freq::Response& res);
    bool changeSyncDelay(dmcam_ros::change_sync_delay::Request& req, dmcam_ros::change_sync_delay::Response& res);	
    bool changeFilter(dmcam_ros::change_filter::Request& req, dmcam_ros::change_filter::Response& res);
    bool disableFilter(dmcam_ros::disable_filter::Request& req, dmcam_ros::disable_filter::Response& res);

private:
    //node handler from main
    ros::NodeHandle *m_nodeHandle;
    //image transport handle
    image_transport::ImageTransport *m_imageTransport;
    //the opened device
    dmcam_dev_t *m_dev;
    //0 means not good, and will do nothing later on
    bool m_cameraState;
    //switch of showing picture and some test information
    bool m_testMode;
    //frame info
    dmcam_frame_t m_frameInfo;
    //buffer for storing raw data from tof
    uint8_t *m_frameBuffer;
    uint8_t *dist_pseudo_rgb;
    //buffer for storing gray\dist\amb image temporarily
    float *m_rbgBuffer;
    //array to store point cloud
    float *m_pclBuffer;
    //parameters of tof
    uint32_t m_devMode;
    uint32_t m_modFreq;
    uint32_t m_modFreq1;
    uint32_t m_syncDelay;
    bool m_enbaleFlip;
    int m_flipCode;
    dmcam_param_roi_t m_roi;
    uint32_t m_format;
    uint8_t power_percent;
    uint32_t m_fps;
    uint16_t m_intgTime;//intg_us in 1.72 SDK
    uint16_t m_intgTimeHDR;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat map1, map2;
    CAMERA_INTRINSIC_PARAMETERS intrinsicmat;
    //parameters of camera
    dmcam_len_param_t cam_int_param;
    //publisher
    image_transport::Publisher m_imageGrayPub;
    image_transport::Publisher m_imageDistPub;
    ros::Publisher m_cameraInfoPublic;
    ros::Publisher m_pointCloudPublic;
    //message
    sensor_msgs::CameraInfo m_cameraInfoMessage;
    std_msgs::Header m_imgHeadMsg;
    sensor_msgs::PointCloud2  m_pointCloudMsg;

    //server
    ros::ServiceServer m_changePowerServer;
    ros::ServiceServer m_changeIntgServer;
    ros::ServiceServer m_changeFrameServer;
    ros::ServiceServer m_changeModfreqServer;
    ros::ServiceServer m_changeSyncDelayServer;
    //server fliter
    ros::ServiceServer m_changeFilterServer;
    ros::ServiceServer m_disableFilterServer;

    //get one frame from tof
    uint32_t epcGetPartType();
    std::string checkDeviceType();
    void getFrameInfo(std::string deviceType);
    void getOneFrame(void);
    //initialize tof
    void tofInitialize(void);
    //initialize parameters
    void paramInitialize(dmcam_dev_t *dev_0);
    //initialize topics
    void topicInitialize(void);
    //init server
    void serverInitialize(void);
    //sudo device
    void sudodev(dmcam_dev_t *_dev);
    //publish image(including point cloud inside)
    void publicImage(void);
    //publish point cloud
    void publicPointCloud(float* pcl_buff);
    //publish camera info
    void publicCaminfo(void);
    //undistort the image
    void undistort( cv::InputArray _src, cv::OutputArray _dst, cv::InputArray _cameraMatrix,
                    cv::InputArray _distCoeffs, cv::InputArray _newCameraMatrix );

    int m_width;
    int m_height;
    int m_curfsize;
    int m_frameSize;
    std::string m_deviceType;

private:
    std::map<std::string,dmcam_filter_id_e> m_filterType;
};

#endif // TOFHANDLE_H
