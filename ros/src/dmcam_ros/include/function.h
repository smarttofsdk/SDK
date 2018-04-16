/*****************************************************************//**
 *       @file  function.h
 *      @brief  DM's camera device API
 *
 *  Detail Decsription starts here
 *
 *   @internal
 *     Project  $Project$
 *     Created  6/20/2017 
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
#ifndef __FUNCTION_H__
#define __FUNCTION_H__

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// camera parameters
struct CAMERA_INTRINSIC_PARAMETERS
{
	float cx;			// center point x
	float cy;           // center point y
	float fx;           // focal length x
	float fy;           // focal length y
	float scale;        // ratio between image value and real distance (unit is meter)
};


/**
*  distance data to pointcloud data, the distance data is stored in cv::Mat
*
* @param pc [out] pointcloud data in float, each point includes x, y, z, unit is meter
* @param pc_sz [in]  length of pointcloud 
* @param img_dist [in] input distance data in cv::Mat
* @param camera [in] camera intrinsic parameters
*/
int dmg_depth_to_pointcloud_from_cvmat(float *pc, int pc_sz, const cv::Mat& img_dist, CAMERA_INTRINSIC_PARAMETERS& camera);

/**
*  distance data to pointcloud data, the distance data is stored in array
*
* @param pc [out] pointcloud data in float, each point includes x, y, z, unit is meter
* @param pc_sz [in]  length of pointcloud
* @param img_dist [in] input distance data in array
* @param img_h [in] height of the distance image
* @param img_w [in] width of the distance image
* @param img_d [in] data depth of the distance image: 1=uint8, 2=uint_16, 4=float32
* @param camera [in] camera intrinsic parameters
*/
int dmg_depth_to_pointcloud(float *pc, int pc_sz, const char* img_dist, int img_h, int img_w, int img_d, CAMERA_INTRINSIC_PARAMETERS& camera);

/**
*  distance data to pointcloud data, the distance data is stored in file
*
* @param pc [out] pointcloud data in float, each point includes x, y, z, unit is meter
* @param pc_sz [in]  length of pointcloud
* @param img_dist_filename [in] input distance data in file
* @param camera [in] camera intrinsic parameters
*/
int dmg_depth_to_pointcloud_from_file(float *pc, int pc_sz, const char* img_dist_filename, CAMERA_INTRINSIC_PARAMETERS& camera);

/**
*  pointcloud data to distance data, the pointcloud data is stored in pcl::PointCloud
*
* @param img_dist [out] distance data in uint_16
* @param img_h [in] height of the distance image
* @param img_w [in] width of the distance image
* @param pc [in] input pointcloud data in pcl::PointCloud, unit is meter
* @param camera [in] camera intrinsic parameters
*/
int dmg_pointcloud_to_depth_from_pcl(short *img_dist, int img_h, int img_w, const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, CAMERA_INTRINSIC_PARAMETERS& camera);

/**
*  pointcloud data to distance data, the pointcloud data is stored in array
*
* @param img_dist [out] distance data in uint_16
* @param img_h [in] height of the distance image
* @param img_w [in] width of the distance image
* @param pc [in] input pointcloud data in array, unit is meter
* @param pc_sz [in]  length of pointcloud
* @param camera [in] camera intrinsic parameters
*/
int dmg_pointcloud_to_depth(short *img_dist, int img_h, int img_w, const float* pc, int pc_sz, CAMERA_INTRINSIC_PARAMETERS& camera);

/**
*  pointcloud data to distance data, the pointcloud data is stored in file
*
* @param img_dist [out] distance data in uint_16
* @param img_h [in] height of the distance image
* @param img_w [in] width of the distance image
* @param pc [in] input pointcloud data in file, unit is meter
* @param camera [in] camera intrinsic parameters
*/
int dmg_pointcloud_to_depth_from_file(short *img_dist, int img_h, int img_w, const char* pointcloud_filename, CAMERA_INTRINSIC_PARAMETERS& camera);


#endif