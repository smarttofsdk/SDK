
# SmartToF SDK User Guide
***
## 1、Brief Introduction on SmartToF SDK
SmartToF SDK is the general development kit for TOF 3D cameras，which has the following functions:
- Support depth, gray data acquisition and display
- Support automatic calibration of data acquired
- Support pointcloud view

So far, SmartToF SDK support the following TOF 3D camera models：
- TCM-E1 model
- TCM-E2 model
- TC series

SmartToF SDK mainly includes the following modules:
- DMCAM core C library and language extensions
  - lib
  - python
  - java
- SmartToF developer tools
  - SmartTofViewer
  - SmartTofCli
  - SmartTofAnalyzer

SmartToF SDK supports multiple OS including windows and linux. And it provides various samples so that users' software development，including：
- C/C++ samples
- python samples
- ROS samples
- android sample

The supported platforms of SmartToF SDK are shown below：

|                    | Windows | Linux   | Android |
| :----------------- | :------ | :------ | :------ |
| Core API C library | &radic; | &radic; | &radic; |
| Python             | &radic; | &radic; |         |
| Java               | &radic; | &radic; | &radic; |
| Ros                |         | &radic; |         |
| C#                 | &radic; | &radic; |         |
| Matlab             | &radic; |         |         |
| usbdriver          | &radic; | &radic; | &radic; |
| SmartTofAnalyzer   | &radic; | &radic; |         |
| SmartTofCli        | &radic; | &radic; |         |
| SmartToFViewer     | &radic; | &radic; | &radic; |
***
## 2、The Use of SmartToF SDK
The use of SmartToF SDK requires system configuration for the working OS，including
- Windows configurations. Please see ([reference link](https://github.com/smarttofsdk/SDK/wiki/SmartToF-SDK-configuration-in-Windows_EN))
- Linux configurations. Please see（[reference link](https://github.com/smarttofsdk/SDK/wiki/SmartToF-SDK-configuration-in-Linux_EN)）

***
## 3、Introduction on Main APIs and Samples
All definitions of structures and declarations of functions in SmartToF SDK are located in dmcam.h under lib\include. For example for Windows, the definitions and declarations are under SDK/windows/dmcam/lib/include. There are detailed explanation of the functions and their parameters in it.
### 3.1 Simplified data acquisition sample
~~~C
/*Initialization*/
dmcam_init(NULL);
...
/*Open device*/
dev = dmcam_dev_open(NULL);//Open the first device
/*Setting acquisition buffer*/
dmcam_cap_set_frame_buffer(dev,NULL,FRAME_SIZE*FRAME_BUF_FCNT);
...
/*Start acquisition*/
dmcam_cap_start(dev);//Start acquisition
/*Obtaining data*/
fr_cnt = dmcam_cap_get_frames(dev,20,fbuf,FRAME_SIZE*20,&fbuf_info);//Acquire 20 frames
/*Acquire depth data*/
dmcam_frame_get_distance(dev,dist,dist_len,fbuf,fbuf_info.frame_info.frame_size,&fbuf_info.frame_info);//Transform one frame to depth data
/*Acquire gray data*/
dmcam_frame_get_gray(dev,gray,gray_len,fbuf,fbuf_info.frame_info.frame_size,&fbuf_info.frame_info);//Transform one frame to gray data
/*Acquire pointcloud data*/
dmcam_frame_get_pcl(dev,pcl,pcl_len,dist,dist_len,img_w,img_h,NULL);//Transform the depth data to pointcloud data
/*Stop acquisition*/
dmcam_cap_stop(dev);
...
dmcam_dev_close(dev);
dmcam_uninit();
~~~
### 3.2 Introduction on main APIs
#### 3.2.1 Initialize module
~~~C
dmcam_init(const char *log_fname);	//Initialization
~~~
| Parameter | Description                              |
| :-------- | :--------------------------------------- |
| log_fname | File name of log. If it is NULL, the default file name is dmcam_YYYYMMDD.log |

#### 3.2.2 List all modules
~~~C 
dmcam_dev_list(dmcam_dev_t *dev_list,int dev_list_num); 
~~~
| Parameter    | Description                            |
| :----------- | :------------------------------------- |
| dev_list     | List for connected devices             |
| dev_list_num | Size of the list for connected devices |

#### 3.2.3 Open device
~~~C
dmcam_dev_open(dmcam_dev_t *dev); 
~~~
| Parameter | Description               |
| :-------- | :------------------------ |
| dev       | Designated device to open |

#### 3.2.4 Set parameters
~~~C
dmcam_param_batch_set(dmcam_dev_t *dev, const dmcam_param_item_t *param_items, int item_cnt);
~~~
| Parameter   | Description                |
| :---------- | :------------------------- |
| dev         | Designated device          |
| param_items | Parameter of device        |
| item_cnt    | Count of device parameters |

#### 3.2.5 Set buffer
~~~C
dmcam_cap_set_frame_buffer(dmcam_dev_t *dev, uint8_t *frame_buf, uint32_t frame_buf_size);
~~~
| Parameter      | Description          |
| :------------- | :------------------- |
| dev            | Designated device    |
| frame_buf      | Array of buffer      |
| frame_buf_size | Size of buffer array |

#### 3.2.6 Set callback function on error
~~~C
dmcam_cap_set_callback_on_error(dmcam_dev_t *dev, dmcam_cap_err_f cb);
~~~
| Parameter | Description                |
| :-------- | :------------------------- |
| dev       | Designated device          |
| cb        | Callback function on error |

#### 3.2.7 Acquire frames
~~~C
dmcam_cap_get_frames(dmcam_dev_t *dev, uint32_t frame_num, uint8_t *frame_data, uint32_t frame_dlen, dmcam_frame_t *first_frame_info);
~~~
| Parameter        | Description                    |
| :--------------- | :----------------------------- |
| dev              | Designated device              |
| frame_num        | Number of frames to acquire    |
| frame_data       | Acquired frame data            |
| frame_dlen       | Size of frame data             |
| first_frame_info | Information of the first frame |

#### 3.2.8 Transform to depth data
~~~C
dmcam_frame_get_distance(dmcam_dev_t *dev, float *dst, int dst_len,uint8_t *src, int src_len, const dmcam_frame_info_t *finfo);
~~~
| Parameter | Description                              |
| :-------- | :--------------------------------------- |
| dev       | Designated device                        |
| dst       | Destination for the transformed depth data |
| dst_len   | Buffer size of depth data                |
| src       | Original data obtained                   |
| src_len   | Size of original data                    |
| finfo     | Information of original frame            |

#### 3.2.9 Transform to gray data
~~~C
dmcam_frame_get_gray(dmcam_dev_t *dev, float *dst, int dst_len,
uint8_t *src, int src_len, const dmcam_frame_info_t *finfo);
~~~
| Parameter | Description                              |
| :-------- | :--------------------------------------- |
| dev       | Designated device                        |
| dst       | Destination for the transformed gray data |
| dst_len   | Buffer size of gray data                 |
| src       | Original data obtained                   |
| src_len   | Size of original data                    |
| finfo     | Information of original frame            |

#### 3.2.10 Acquire pointcloud data
~~~C
dmcam_frame_get_pcl(dmcam_dev_t * dev, float *pcl, int pcl_len,
const float *dist, int dist_len, int img_w, int img_h, const dmcam_camera_para_t *p_cam_param);
~~~
| Parameter   | Description                              |
| :---------- | :--------------------------------------- |
| dev         | Designated device                        |
| pcl         | Pointcloud data output. Each point's coordinate consists of three elements |
| dist        | Depth data of input image                |
| dist_len    | Size of depth data of input image        |
| img_w       | Width of depth image                     |
| img_h       | Height of depth image                    |
| p_cam_param | Internal parameter of camera             |
***
### 3.3 Introduction on relevant samples
The main samples provided by SmartToF SDK are following：
- C samples：See（[C samples link](https://github.com/smarttofsdk/SDK/tree/master/windows/samples/c)）
  - sample_capture_frames.c: Demonstrate data acquired
  - sample_set_param.c: Demonstrate setting parameters
  - sample_filter.c: Demonstrate data calibration
- C++ samples:See([C++ samples link](https://github.com/smarttofsdk/SDK/tree/master/windows/samples/c%2B%2B))
  - sample_capture_frames.cpp: Demonstrate data acquired
  - sample_set_param.cpp: Demonstrate setting parameters
  - sample_filter.cpp: Demonstrate data calibration 
- python samples：See（[python samples link](https://github.com/smarttofsdk/SDK/tree/master/windows/samples/python)）
  - sample_gui_pyQtGraph.py: Demonstrate data acquired
  - sample_gui_pygame.py: Demonstrate data acquired
  - sample_param.py: Demonstrate parameters setting and reading
  - sample_basic: Demonstrate data acquired
- java samples: See([java samples link](https://github.com/smarttofsdk/SDK/tree/master/windows/samples/java/com/smarttof/dmcam/sample))
  - sampleBasic.java: java acquisition sample
  - sampleBasicUi.java: java acquisition and demonstration sample
- OpenNI sample:see([OpenNI sample link](https://github.com/smarttofsdk/SDK/tree/master/windows/samples/openni2))
- ROS sample: See([ros sample link](https://github.com/smarttofsdk/SDK/tree/master/ros))
  - tof_sample: Demonstrate how to acquire image and pointcloud

