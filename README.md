# SmartToF SDK使用说明

------

**SmartToF SDK用户手册参考以下([网址链接](https://smarttofdoc.readthedocs.io/en/latest/))**

## 1、SmartToF SDK简介

SmartToF TC系列模组是数迹公司采用TOF技术开发的3D视觉模组，采用业界领先的传感器芯片，具有测量精度高、抗干扰能力强、外观小巧等优点。模组可用于精确的人流统计、物流仓储、手势识别、机器人避障和车载控制等新兴技术领域。SmartToF SDK是配套SmartToF系列模组进行开发的软件工具包，支持windows、linux、Android等主流平台，SDK的总体架构图如下：

！[框图](https://github.com/smarttofsdk/doctest/tree/master/source/Introduction/image/Overview.png)

SDK中架构中的主要部分说明和特点如下图所示：

！[架构图](https://github.com/smarttofsdk/doctest/tree/master/source/Introduction/image/Components.png)

------

## 2、主要API介绍和样例说明

### 2.1 最简化的采集数据例程

```c
/*初始化*/
dmcam_init(NULL);
...
/*打开设备*/
dev = dmcam_dev_open(NULL);//打开第一个设备
/*采集设置*/
dmcam_cap_cfg_t cap_cfg = {
    .cache_frames_cnt = FRAME_BUF_FCNT, /* FRAME_BUF_FCNT frames can be cached in frame buffer*/
    .on_error = NULL,      /* No error callback */
    .on_frame_ready = NULL, /* No frame ready callback*/
    .en_save_replay = false, /* false save raw data stream to replay file */
    .en_save_dist_u16 = false, /* disable save dist stream into replay file */
    .en_save_gray_u16 = false, /* disable save gray stream into replay file*/
    .fname_replay = NULL, /* replay filename */
};
dmcam_cap_config_set(dev,&cap_cfg);
...
/*开始采集*/
dmcam_cap_start(dev);//开始采集
/*获得采集数据*/
fr_cnt = dmcam_cap_get_frames(dev,20,fbuf,FRAME_SIZE*20,&fbuf_info);//采集20帧数据
/*获得深度数据*/
dmcam_frame_get_dist_u16(dev,dist,dist_len,fbuf,fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);//解析出一帧深度数据
/*获得灰度数据*/
dmcam_frame_get_gray_u16(dev,gray,gray_len,fbuf,fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);//解析出一帧灰度数据
/*获取点云数据*/
dmcam_frame_get_pcl(dev,pcl,pcl_len,dist,dist_len,img_w,img_h,NULL);//将转换的深度数据转换成点云数据
/*停止采集*/
dmcam_cap_stop(dev);
...
dmcam_dev_close(dev);
dmcam_uninit();
```

------

### 2.2相关样例说明

SmartToF SDK提供的主要样例如下：

- C样例：参见（[C样例链接](https://github.com/smarttofsdk/SDK/tree/master/windows/samples/c)）
  - sample_capture_frames.c:展示采集数据
  - sample_set_param.c:展示进行参数的设置
  - sample_filter.c:展示校准数据
- C++样例：参见([C++样例链接](https://github.com/smarttofsdk/SDK/tree/master/windows/samples/c%2B%2B))
  - sample_capture_frames.cpp:展示采集数据
  - sample_set_param.cpp:展示进行参数的设置
  - sample_filter.cpp:展示校准数据
- python样例：参见（[python样例链接](https://github.com/smarttofsdk/SDK/tree/master/windows/samples/python)）
  - sample_gui_pyQtGraph.py:采集数据显示
  - sample_gui_pygame.py:采集数据显示
  - sample_param.py:展示参数设置和读取
  - sample_basic:展示采集数据
- java样例：参见([java样例链接](https://github.com/smarttofsdk/SDK/tree/master/windows/samples/java/com/smarttof/dmcam/sample))
  - sampleBasic.java:java采集样例
  - sampleBasicUi.java:java采集显示样例
- OpenNI样例：参见([OpenNI样例链接](https://github.com/smarttofsdk/SDK/tree/master/windows/samples/openni2))
- ROS样例：参见([ros样例链接](https://github.com/smarttofsdk/SDK/tree/master/ros))

