/*****************************************************************//**
 *       @file  function.cpp
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
#include "function.h"

int dmg_depth_to_pointcloud_from_cvmat(float *pc, int pc_sz, const cv::Mat& img_dist, CAMERA_INTRINSIC_PARAMETERS& camera) {

	if (img_dist.data == NULL) {
		printf("Img Read Failed");
		return -1;
	}
	
	if (!pc || pc_sz <= 0) return -1;

	if (img_dist.type() != CV_8UC1 && img_dist.type() != CV_16UC1 && img_dist.type() != CV_32FC1) {
		printf("Invalid Type");
		return -1;
	}

	// convert to CV_32FC1
	cv::Mat img_f;
	img_dist.convertTo(img_f, CV_32FC1);
	img_f = img_f / camera.scale;
		
	int r = img_f.rows;
	int c = img_f.cols;
	int idx = 0;
	for (int m = 0; m < r; m++)
		for (int n = 0; n < c; n++)
		{
			float d = img_f.ptr<float>(m)[n];
			if (d <= 0 || d >= 6.25) {
				pc[3 * idx + 0] = 0;
				pc[3 * idx + 1] = 0;
				pc[3 * idx + 2] = 0;
			}
			else {
				// 计算这个点的空间坐标
				pc[3 * idx + 2] = float(d);
				pc[3 * idx + 0] = (n - camera.cx) * pc[3 * idx + 2] / camera.fx;
				pc[3 * idx + 1] = (m - camera.cy) * pc[3 * idx + 2] / camera.fy;
			}
			idx++;
		}

	return 0;
}

int dmg_depth_to_pointcloud(float *pc, int pc_sz, const char* img_dist, int img_h, int img_w, int img_d, CAMERA_INTRINSIC_PARAMETERS& camera) {
	
	int ret = -1;
	if (img_d != 1 && img_d != 2 && img_d != 4) return ret;
	if (!img_dist || (img_w * img_h) <= 0) return ret;
	if (!pc || pc_sz <= 0) return ret;

	char* img_dis = (char *)malloc(img_w * img_h * img_d);
	if (!img_dis) return ret;
	memcpy(img_dis, img_dist, img_w * img_h * img_d);
	cv::Mat input_dis;

	int img_type;
	switch (img_d)
	{
	case 1: {
		img_type = CV_8UC1;
		break;
	}
	case 2: {
		img_type = CV_16UC1;
		break;
	}
	case 4: {
		img_type = CV_32FC1;
		break; 
	}
	default:
		goto FINAL;
	}

	input_dis = cv::Mat(img_h, img_w, img_type, img_dis);

	ret = dmg_depth_to_pointcloud_from_cvmat(pc, pc_sz, input_dis, camera);

FINAL:
	free(img_dis);
	return ret;
}

int dmg_depth_to_pointcloud_from_file(float *pc, int pc_sz, const char* img_dist_filename, CAMERA_INTRINSIC_PARAMETERS& camera) {

	const cv::Mat input_dis_src = cv::imread(img_dist_filename, -1);
	if (input_dis_src.data == NULL) {
		printf("Img Read Failed");
		return -1;
	}

	if (!pc || pc_sz <= 0) return -1;

	return dmg_depth_to_pointcloud_from_cvmat(pc, pc_sz, input_dis_src, camera);
}

int dmg_pointcloud_to_depth_from_pcl(short *img_dist, int img_h, int img_w, const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, CAMERA_INTRINSIC_PARAMETERS& camera) {
	
	if (pc == NULL) {
		printf("Cloud Read Failed");
		return -1;
	}
	if (!img_dist || (img_h * img_w) <= 0) return -1;

	memset(img_dist, 0xff, img_h * img_w * sizeof(short));

	for (int i = 0; i < pc->points.size(); i++) {
		if (pc->points[i].z <= 0) continue;
		int u = int(camera.cx + pc->points[i].x * camera.fx / pc->points[i].z + 0.5);
		int v = int(camera.cy + pc->points[i].y * camera.fy / pc->points[i].z + 0.5);
		int uv = v * img_w + u;
		img_dist[uv] = short(pc->points[i].z * camera.scale);
	}

	return 0;
}

int dmg_pointcloud_to_depth(short *img_dist, int img_h, int img_w, const float* pc, int pc_sz, CAMERA_INTRINSIC_PARAMETERS& camera) {

	if (!pc) return -1;
	if (!img_dist || (img_h * img_w) <= 0) return -1;

	memset(img_dist, 0xff, img_h * img_w * sizeof(short));

	for (int i = 0; i < pc_sz; i++) {
		if (pc[3 * i + 2] <= 0) continue;
		int u = int(camera.cx + pc[3 * i + 0] * camera.fx / pc[3 * i + 2] + 0.5);
		int v = int(camera.cy + pc[3 * i + 1] * camera.fy / pc[3 * i + 2] + 0.5);
		int uv = v * img_w + u;
		img_dist[uv] = short(pc[3 * i + 2] * camera.scale);
	}

	return 0;

}

int dmg_pointcloud_to_depth_from_file(short *img_dist, int img_h, int img_w, const char* pointcloud_filename, CAMERA_INTRINSIC_PARAMETERS& camera) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_filename, *cloud) == -1)
	{
		printf("Cloud Read Failed");
		return -1;
	}

	if (!img_dist || (img_h * img_w) <= 0) return -1;

	return dmg_pointcloud_to_depth_from_pcl(img_dist, img_h, img_w, cloud, camera);
}