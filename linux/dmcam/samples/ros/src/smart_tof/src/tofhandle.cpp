/*****************************************************************//**
 *       @file  tofhandle.c
 *      @brief  DM's camera device API
 *
 *  Detail Decsription starts here
 *
 *   @internal
 *     Project  $Project$
 *     Created  6/14/2017 
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
#include "tofhandle.h"

TofHandle::TofHandle(ros::NodeHandle *_n)
{
    nh = _n;
    tof_state = 0;
    it = new image_transport::ImageTransport((*_n));

    tof_init();
    if (tof_state) {
        topic_init();
        server_init();
        exchange = new float[roi.cur_fsize ];
	dist_pseudo_rgb = new uint8_t[roi.cur_fsize*3];
        pcl_exchange = new float[3 * roi.cur_fsize];
    }
}

TofHandle::~TofHandle(void)
{
    if (tof_state) {
        free(fbuf);
        delete(exchange);
        delete(pcl_exchange);
        delete(it);
        delete(dist_pseudo_rgb);
        dmcam_cap_stop(dev);
        dmcam_dev_close(dev);
        dmcam_uninit();
        ROS_INFO("[SMART TOF]smart tof is shutted down!");
    }
}

//initial tof
void TofHandle::tof_init(void)
{
    dmcam_init(NULL);
    dmcam_log_cfg(LOG_LEVEL_INFO, LOG_LEVEL_TRACE, LOG_LEVEL_NONE);

    //try to open smart tof
    dmcam_dev_t dev_list[4];

    int dev_cnt = dmcam_dev_list(dev_list, 4);
    ROS_INFO("[SMART TOF]%d dmcam device found!", dev_cnt);
    if (dev_cnt == 0) {
        ROS_INFO("[SMART TOF]find device failed. Please kill the process and check your device.");
        return;
    }
#if NEEDSUDO
    sudodev(&dev_list[0]);
#endif
    dev = dmcam_dev_open(NULL);

    if (!dev) {
        ROS_INFO("[SMART TOF]open device failed!");
        return;
    } else {
        ROS_INFO("[SMART TOF]open device succeeded!");
    }
    //init the device opened
    param_init(dev);

    dmcam_cap_set_frame_buffer(dev, NULL, FRAME_SIZE);
    dmcam_cap_set_callback_on_error(dev, on_cap_err);

    fbuf = (uint8_t *)malloc(FRAME_SIZE);

    dmcam_cap_start(dev);

    tof_state = 1;
}

//initialize server
void TofHandle::server_init(void)
{
    change_power_ser = nh->advertiseService("/smarttof/change_power", &TofHandle::change_power, this);
    change_intg_ser = nh->advertiseService("/smarttof/change_intg", &TofHandle::change_intg, this);
}

//initial parameters
void TofHandle::param_init(dmcam_dev_t *dev_0)
{
    bool flag = 0;

    nh->getParam("testmode", test_mode);
    int tmp;
    int tmp1, tmp2, tmp3, tmp4, tmp5;
    int tmp6, tmp7, tmp8, tmp9, tmp10, tmp11;
    nh->getParam("dev_mode", tmp);
    nh->getParam("mod_freq", tmp1);
    nh->getParam("format", tmp2);
    nh->getParam("power_percent", tmp3);
    nh->getParam("fps", tmp4);
    nh->getParam("intg_us", tmp5);
    nh->getParam("srow", tmp6);
    nh->getParam("erow", tmp7);
    nh->getParam("scol", tmp8);
    nh->getParam("ecol", tmp9);
    nh->getParam("cur_fsize", tmp10);
    nh->getParam("max_fsize", tmp11);
    dev_mode = tmp;
    mod_freq = tmp1;
    format = tmp2;
    power_percent = tmp3;
    fps = tmp4;
    intg_us = tmp5;
    roi.srow = tmp6;
    roi.erow = tmp7;
    roi.scol = tmp8;
    roi.ecol = tmp9;
    roi.cur_fsize = tmp10;
    roi.max_fsize = tmp11;

    printf("----------setting params----------\n");
    dmcam_param_item_t param_items[7];
    param_items[0].param_id = PARAM_DEV_MODE;
    param_items[0].param_val.dev_mode = dev_mode;
    param_items[0].param_val_len = sizeof(dev_mode);

    param_items[5].param_id = PARAM_MOD_FREQ;
    param_items[5].param_val.mod_freq = mod_freq;
    param_items[5].param_val_len = sizeof(mod_freq);

    param_items[4].param_id = PARAM_ROI;
    param_items[4].param_val.roi.ecol = roi.ecol;
    param_items[4].param_val.roi.erow = roi.erow;
    param_items[4].param_val.roi.scol = roi.scol;
    param_items[4].param_val.roi.srow = roi.srow;
    param_items[4].param_val_len = sizeof(roi);

    param_items[3].param_id = PARAM_FRAME_FORMAT;
    param_items[3].param_val.frame_format.format = format;
    param_items[3].param_val_len = sizeof(format);

    param_items[1].param_id = PARAM_ILLUM_POWER;
    param_items[1].param_val.illum_power.percent = power_percent;
    param_items[1].param_val_len = sizeof(power_percent);

    param_items[6].param_id = PARAM_FRAME_RATE;
    param_items[6].param_val.frame_rate.fps = fps;
    param_items[6].param_val_len = sizeof(fps);

    param_items[2].param_id = PARAM_INTG_TIME;
    param_items[2].param_val.intg.intg_us = intg_us;
    param_items[2].param_val_len = sizeof(intg_us);

    if (test_mode) {
        printf("param mode is %d\n", param_items[0].param_val.dev_mode);
        printf("param freq is %d\n", param_items[5].param_val.mod_freq);
        printf("param fps is %d\n", param_items[6].param_val.frame_rate.fps);
        printf("param roi is (%d,%d;%d,%d)\n",
               param_items[4].param_val.roi.srow, param_items[4].param_val.roi.scol,
               param_items[4].param_val.roi.erow, param_items[4].param_val.roi.ecol);
        printf("param power is %d\n", param_items[1].param_val.illum_power.percent);
        printf("param format is %d\n", param_items[3].param_val.frame_format.format);
        printf("param intg time is %d\n", param_items[2].param_val.intg.intg_us);
    }

    for (int i = 0; i < 7; ++i) {
        flag = dmcam_param_batch_set(dev_0, &(param_items[i]), 1);
    }

    sleep(1);
    ROS_INFO("[SMART TOF]initialize parameters succeeds!");

    //camera intrinsic
    double ttmp;
    nh->getParam("cx", ttmp);
    cam_int_param.cx = ttmp;
    nh->getParam("cy", ttmp);
    cam_int_param.cy = ttmp;
    nh->getParam("fx", ttmp);
    cam_int_param.fx = ttmp;
    nh->getParam("fy", ttmp);
    cam_int_param.fy = ttmp;
    nh->getParam("scale", ttmp);
    cam_int_param.scale = ttmp;

    //camera info init
    cam_inf_msg.height = 240;
    cam_inf_msg.width = 320;
    cam_inf_msg.roi.width = 320; //roi.ecol - roi.scol; //
    cam_inf_msg.roi.height = 240; //;roi.erow - roi.srow;
    cam_inf_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    cam_inf_msg.header.frame_id = "smart_tof";
    cam_inf_msg.K[2] = cam_int_param.cx;
    cam_inf_msg.K[5] = cam_int_param.cy;
    cam_inf_msg.K[0] = cam_int_param.fx;
    cam_inf_msg.K[4] = cam_int_param.fy;
    cam_inf_msg.K[8] = 1; //cam_int_param.scale
    cam_inf_msg.R[0] = 1;
    cam_inf_msg.R[4] = 1;
    cam_inf_msg.R[8] = 1;
    cam_inf_msg.P[2] = cam_int_param.cx;
    cam_inf_msg.P[6] = cam_int_param.cy;
    cam_inf_msg.P[0] = cam_int_param.fx;
    cam_inf_msg.P[5] = cam_int_param.fy;
    cam_inf_msg.P[10] = 1; //cam_int_param.scale
    img_head_msg.frame_id = "smart_tof";

    if (test_mode) {
        //test part
        printf("-------get param after set-------\n");
        flag = 0;
        dmcam_param_item_t get_param1;
        get_param1.param_id = PARAM_ROI;
        flag = dmcam_param_batch_get(dev_0, &get_param1, 1);
        printf("get flag is %d,parameter len we get %d\n", flag, get_param1.param_val_len); //return 4
        printf("get roi is (%d,%d;%d,%d)\n",
               get_param1.param_val.roi.srow, get_param1.param_val.roi.scol,
               get_param1.param_val.roi.erow, get_param1.param_val.roi.ecol);
        printf("cur_fsize is %d\n", get_param1.param_val.roi.cur_fsize);

        get_param1.param_id = PARAM_MOD_FREQ;
        flag = dmcam_param_batch_get(dev_0, &get_param1, 1);
        printf("[%d]get flag is %d, parameter len we get %d, ", PARAM_MOD_FREQ, flag, get_param1.param_val_len);
        printf("get freq is %d\n", get_param1.param_val.mod_freq);

        get_param1.param_id = PARAM_ILLUM_POWER;
        flag = dmcam_param_batch_get(dev_0, &get_param1, 1);
        printf("[%d]get flag is %d, parameter len we get %d, ", PARAM_ILLUM_POWER, flag, get_param1.param_val_len);
        printf("get power is %d\n", get_param1.param_val.illum_power.percent);

        get_param1.param_id = PARAM_FRAME_RATE;
        flag = dmcam_param_batch_get(dev_0, &get_param1, 1);
        printf("[%d]get flag is %d, parameter len we get %d, ", PARAM_FRAME_RATE, flag, get_param1.param_val_len);
        printf("get fps is %d\n", get_param1.param_val.frame_rate.fps);

        get_param1.param_id = PARAM_INTG_TIME;
        flag = dmcam_param_batch_get(dev_0, &get_param1, 1);
        printf("[%d]get flag is %d, parameter len we get %d, ", PARAM_INTG_TIME, flag, get_param1.param_val_len);
        printf("get intg time is %d\n", get_param1.param_val.intg.intg_us);

        sleep(1);
        //test part
    }
}

//initial topics
void TofHandle::topic_init(void)
{
    image_gray_pub = it->advertise("/smarttof/image_gray", 1);
    image_dist_pub = it->advertise("/smarttof/image_dist", 1);
    image_amb_pub = it->advertise("/smarttof/image_amb", 1);
    cam_info_pub = nh->advertise<sensor_msgs::CameraInfo>("/smarttof/camera_info", 1);
    pcl_pub = nh->advertise<sensor_msgs::PointCloud2>("smarttof/pointcloud", 1);
}

//sudo device
void TofHandle::sudodev(dmcam_dev_t *_dev)
{
    if (_dev == NULL)
        return;

    char path[21];
    char *_pa = "/dev/bus/usb/000/000";
    strcpy(path, _pa);
    int tmp = _dev->usb_dev_addr;
    for (int i = 0; tmp != 0; ++i) {
        path[19 - i] = tmp % 10 - 0 + '0';
        tmp = tmp / 10;
    }
    path[15] = _dev->usb_bus_num - 0 + '0';
    char *orhead = "sudo -S chmod 666 ";
    char order[39];
    strcpy(order, orhead);
    strcpy(&order[18], path);
    printf("%s\n", order);
    system(order);
}

//publish all topics
void TofHandle::pub_all(void)
{
    get_one_frame();
    pub_image();
    pub_caminfo();
}

//get tof state(can it be used?)
bool TofHandle::get_tof_state(void)
{
    return tof_state;
}

//get test mode state
bool TofHandle::get_test_mode(void)
{
    return test_mode;
}

//get one frame
void TofHandle::get_one_frame(void)
{
    if (1 != dmcam_cap_get_frames(dev, 1, fbuf, roi.cur_fsize * 4, &fbuf_info)) {
       printf("Get frame failed:%d\n", roi.cur_fsize * 4);
    } else {
     //   printf("Get frame ok:%d\n", roi.cur_fsize * 4);
    }
}

//pub image
void TofHandle::pub_image(void)
{
    //prepare header
    img_head_msg.stamp = ros::Time::now();
    int w = fbuf_info.frame_info.width;
    int h = fbuf_info.frame_info.height;
    //pub gray image
    /*    dmcam_raw2gray(exchange, roi.cur_fsize/4, fbuf, roi.cur_fsize);
        cv::Mat img_gray((roi.erow-roi.srow), (roi.ecol-roi.scol), CV_32FC1, exchange);
        img_gray *=255*255;
        img_gray.convertTo(img_gray,CV_8UC1);
    if(test_mode){
        cv::imshow("img gray", img_gray);
    }
        image_gray_pub.publish(
                    sensor_msgs::ImagePtr(
                        cv_bridge::CvImage(img_head_msg, "mono8", img_gray).toImageMsg()
                        )
                    );
    */
    //pub dist image
    dmcam_filter_args_u famp;
    famp.min_amp = 30; // TODO: use parameters 
    dmcam_filter_enable(dev, DMCAM_FILTER_ID_AMP, &famp, sizeof(famp));
    int pix_cnt = dmcam_frame_get_distance(dev, exchange, roi.cur_fsize/2, 
		    fbuf, fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);
    if (pix_cnt != h * w) {
	    printf(" unmatch pixcnt: %d, HxW: %dx%d\n", pix_cnt, h, w);
    }

    //dmcam_raw2dist(exchange, roi.cur_fsize / 2, fbuf, roi.cur_fsize * 4);

    cv::Mat img_dist(h, w, CV_32FC1, exchange);

    //pub point cloud here
    pub_pcl(img_dist);
    if (test_mode) {
        cv::imshow("img dist", img_dist);
    }

    {
	 cv_bridge::CvImage img_bridge;
	 sensor_msgs::Image img_msg; // >> message to be sent

	 /* convert dist to pseudo img */
	 dmcam_cmap_float(dist_pseudo_rgb, roi.cur_fsize * 3, 
			 exchange, pix_cnt, DMCAM_CMAP_OUTFMT_RGB, 0.0, 5.0);

	 cv::Mat img_dist_rgb(h, w, CV_8UC3, dist_pseudo_rgb);
	 img_bridge = cv_bridge::CvImage(img_head_msg, 
			 sensor_msgs::image_encodings::RGB8, img_dist_rgb);
	 img_bridge.toImageMsg(img_msg);
	 image_dist_pub.publish(img_msg);
    }
	
    //pub amb image
    /*2dmcam_raw2amb(exchange, roi.cur_fsize / 4, fbuf, roi.cur_fsize);
    cv::Mat img_amb((roi.erow - roi.srow), (roi.ecol - roi.scol), CV_32FC1, exchange);
    image_amb_pub.publish(
        sensor_msgs::ImagePtr(
            cv_bridge::CvImage(img_head_msg, sensor_msgs::image_encodings::TYPE_32FC1, img_amb).toImageMsg()
            )
        );*/
    if (test_mode) {
        cv::waitKey(1);
        cv::moveWindow("img gray", 1, 1);
        cv::moveWindow("img dist", 1, 350);
    }
}

//publish point cloud
void TofHandle::pub_pcl(const cv::Mat& _dist)
{
    dmg_depth_to_pointcloud_from_cvmat(pcl_exchange, roi.cur_fsize / 2, _dist, cam_int_param);
    pclPointCloudXYZ::Ptr pCloud(new pclPointCloudXYZ);
    for (int m = 0; m < roi.cur_fsize / 2; m++) {
        pCloud->points.push_back(pcl::PointXYZ(pcl_exchange[3 * m], pcl_exchange[3 * m + 1], pcl_exchange[3 * m + 2]));
    }
    pCloud->height = 1;
    pCloud->width = pCloud->points.size();
    pCloud->is_dense = false;
    pcl::toROSMsg(*pCloud, cloud_msg);
    cloud_msg.header.frame_id = "smart_tof";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.is_dense = false;
    pcl_pub.publish(cloud_msg);
    pCloud->points.clear();
}

//publish camera info
void TofHandle::pub_caminfo(void)
{
    cam_inf_msg.header.stamp = ros::Time::now();
    cam_info_pub.publish(cam_inf_msg);
}

//capture error handle function
static bool on_cap_err(dmcam_dev_t *_dev, int _err, void *_err_arg)
{
    ROS_INFO("[SMART TOF]cap error found : %s, arg=%p!", dmcam_error_name(_err), _err_arg);
    switch (_err) {
        case DMCAM_ERR_CAP_FRAME_DISCARD:
            ROS_INFO("[SMART TOF]data process too slow: total missing %d frames\n", *((uint32_t *)&_err_arg));
            break;
        case DMCAM_ERR_CAP_STALL:
            ROS_INFO("[SMART TOF]usb pipe stall!\n");
            break;
        default:
            break;
    }
    dmcam_cap_stop(_dev);
    dmcam_cap_start(_dev);
    return true;
}

bool TofHandle::change_power(smart_tof::change_power::Request& req, smart_tof::change_power::Response& res)
{
    bool flag = 0;
    power_percent = req.b;
    res.power_percent_new = power_percent;
    dmcam_param_item_t reset_power;
    reset_power.param_id = PARAM_ILLUM_POWER;
    reset_power.param_val.illum_power.percent = power_percent;
    reset_power.param_val_len = sizeof(power_percent);
    dmcam_param_batch_set(dev, &reset_power, 1);
    flag = dmcam_param_batch_get(dev, &reset_power, 1);
    printf("new get power is %d\n", reset_power.param_val.illum_power.percent);

    return true;
}

bool TofHandle::change_intg(smart_tof::change_intg::Request& req, smart_tof::change_intg::Response& res)
{
    bool flag = 0;
    intg_us = req.a;
    res.intg_us_new = intg_us;
    dmcam_param_item_t reset_intg;
    reset_intg.param_id = PARAM_INTG_TIME;
    reset_intg.param_val.intg.intg_us = intg_us;
    reset_intg.param_val_len = sizeof(intg_us);
    dmcam_param_batch_set(dev, &reset_intg, 1);
    flag = dmcam_param_batch_get(dev, &reset_intg, 1);
    printf("new get intg time is %d\n", reset_intg.param_val.intg.intg_us);
    return true;
}

