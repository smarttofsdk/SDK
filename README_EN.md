
# SmartToF SDK User Guide

------
**The Detailed SmartToF SDK User Guide see ([the link](https://smarttofdoc.readthedocs.io/en/latest/))**

## 1、Brief Introduction on SmartToF SDK 
SmartToF TC series module is a 3D vision module developed by Digital company with TOF technology. It adopts industry-leading sensor chip and has the advantages of high measurement accuracy, strong anti-interference ability and compact structure. The module can be applied to people counting、gesture recognition、logistics storage、robot obstacle avoidance、vehicle controlling system and other frontier creative technologies. SmartToF SDK is a development kit based on SmartToF TC series module,which is currently supporting Windows,Linux,Android and other mainstream platforms. The overall architecture of the SDK is shown below:

![Block diagram](https://github.com/smarttofsdk/doctest/raw/master/source/Introduction/image/Overview.png)

Main instructions and features of the architecture in the SDK are shown in the following figure:

![Architecture](https://github.com/smarttofsdk/doctest/raw/master/source/Introduction/image/Components.png)

------

## 2. Introduction on samples and simplified data acquisition

### 2.1 Simplified data acquisition sample
~~~C
/*Initialization*/
dmcam_init(NULL);
...
/*Open device*/
dev = dmcam_dev_open(NULL);//Open the first device
/*Setting acquisition buffer*/
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
/*Start acquisition*/
dmcam_cap_start(dev);//Start acquisition
/*Obtaining data*/
fr_cnt = dmcam_cap_get_frames(dev,20,fbuf,FRAME_SIZE*20,&fbuf_info);//Acquire 20 frames
/*Acquire depth data*/
dmcam_frame_get_dist_u16(dev,dist,dist_len,fbuf,fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);//Transform one frame to depth data
/*Acquire gray data*/
dmcam_frame_get_gray_u16(dev,gray,gray_len,fbuf,fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);;//Transform one frame to gray data
/*Acquire pointcloud data*/
dmcam_frame_get_pcl(dev,pcl,pcl_len,dist,dist_len,img_w,img_h,NULL);//Transform the depth data to pointcloud data
/*Stop acquisition*/
dmcam_cap_stop(dev);
...
dmcam_dev_close(dev);
dmcam_uninit();
~~~
------

### 2.2 Introduction on samples

The main examples provided with the SmartToF SDK are as follows:

- C/C++ sample:
  - Introduction see ([the link](https://smarttofdoc.readthedocs.io/en/latest/Tutorial/C_C++/index.html))
  - Detailed introduction see ([the link](https://smarttofdoc.readthedocs.io/en/latest/Tutorial/Python/index.html))
- Python sample:
  - Introduction see ([the link](https://smarttofdoc.readthedocs.io/en/latest/Tutorial/Python/index.html))
  - Detailed introduction see([the link](https://smarttofdoc.readthedocs.io/en/latest/Reference/Python/index.html))
- C# sample:
  - Introduction see([the link](https://smarttofdoc.readthedocs.io/en/latest/Tutorial/Csharp/index.html))
  - Detailed introduction see([the link](https://smarttofdoc.readthedocs.io/en/latest/Reference/Csharp/index.html))
- Java sample:
  - Introduction see([the link](https://smarttofdoc.readthedocs.io/en/latest/Tutorial/Java/index.html))
  - Detailed introduction see([the link](https://smarttofdoc.readthedocs.io/en/latest/Reference/Java/index.html))
- ROS sample:
  - Introduction see([the link](https://smarttofdoc.readthedocs.io/en/latest/Tutorial/ROS/index.html))
  - Detailed introduction see ([the link](https://smarttofdoc.readthedocs.io/en/latest/Reference/ROS/index.html))
- Android sample:
  - Introduction see([the link](https://smarttofdoc.readthedocs.io/en/latest/Tutorial/Android/Androidapk.html))
  - Detailed introduction see([the link](https://smarttofdoc.readthedocs.io/en/latest/Reference/Android/index.html))
