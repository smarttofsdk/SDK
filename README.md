
# SmartToF SDK使用说明
***
## 1、SmartToF SDK简介
SmartToF SDK是TOF 3D相机的通用开发套件，具有下面所列功能:
- 支持深度、灰度数据采集与显示
- 支持采集数据自动校准
- 支持点云显示

目前SmartToF SDK支持以下型号的TOF 3D相机模组：
- TCM-E1模组
- TCM-E2模组

SmartToF SDK主要包括以下模块:
- DMCAM核心C库及语言扩展
  - lib
  - python
  - java
- SmartToF 开发者工具
  - SmartTofViewer
  - SmartTofCli
  - SmartTofAnalyzer

SmartToF SDK支持windows和linux等多种操作系统，同时提供多种参考样例方便用户二次开发，包括：
- C/C++样例
- python样例
- ROS样例
- android样例

具体样例如下表

| 系统      | ROS  | Opencv  | OpenNI  | PCL     | VTK     | matlab | apk     |
| :------ | :--- | :------ | :------ | :------ | :------ | :----- | :------ |
| windows |      | &radic; | &radic; | &radic; | &radic; |        |         |
| linux   |      | &radic; | &radic; | &radic; | &radic; |        |         |
| android |      |         |         |         |         |        | &radic; |
***
## 2、SmartToF SDK的使用
SmartToF SDK的使用需要进行所在系统的环境配置，包括
- windows下的安装配置，详细步骤见([参考链接](https://github.com/smarttofsdk/SDK/wiki/SmartToF-SDKwindow%E4%B8%8B%E7%9A%84%E5%AE%89%E8%A3%85%E9%85%8D%E7%BD%AE))
- linux下的安装配置,详细步骤见（[参考链接](https://github.com/smarttofsdk/SDK/wiki/SmartToF-SDK-Linux%E4%B8%8B%E7%9A%84%E5%AE%89%E8%A3%85%E9%85%8D%E7%BD%AE)）

***
## 3、主要API介绍和样例说明
SmartToF SDK中所有相关结构体定义和函数声明都位于lib\include文件夹下的dmcam.h中，例如windows下在SDK/windows/dmcam/lib/include下，里面对函数的主要功能和参数都有详细说明。
### 3.1 最简化的采集数据例程
~~~C
/*初始化*/
dmcam_init(NULL);
...
/*打开设备*/
dev = dmcam_dev_open(&dev_list[0]);//打开第一个设备
/*设置采集缓存*/
dmcam_cap_set_frame_buffer(dev,NULL,FRAME_SIZE*FRAME_BUF_FCNT);
...
/*开始采集*/
dmcam_cap_start(dev);//开始采集
/*获得采集数据*/
fr_cnt = dmcam_cap_get_frames(dev,20,fbuf,FRAME_SIZE*20,&fbuf_info);//采集20帧数据
/*获得深度数据*/
dmcam_frame_get_distance(dev,dist,dist_len,fbuf,fbuf_info.frame_info.frame_size,&fbuf_info.frame_info);//解析出一帧深度数据
/*获得灰度数据*/
dmcam_frame_get_gray(dev,gray,gray_len,fbuf,fbuf_info.frame_info.frame_size,&fbuf_info.frame_info);//解析出一帧灰度数据
/*停止采集*/
dmcam_cap_stop(dev);
...
dmcam_dev_close(dev);
dmcam_uninit();
~~~
### 3.2主要API介绍
#### 3.2.1模组初始化
~~~C
dmcam_init(const char *log_fname);	//初始化
~~~
| 参数        | 描述                                  |
| :-------- | :---------------------------------- |
| log_fname | 日志文件名，如果为NULL,默认为dmcam_YYYYMMDD.log |

#### 3.2.2列出所有模组
~~~C 
dmcam_dev_list(dmcam_dev_t *dev_list,int dev_list_num); 
~~~
| 参数           | 描述    |
| :----------- | :---- |
| dev_list     | 连接设备表 |
| dev_list_num | 设备表大小 |

#### 3.2.3打开设备
~~~C
dmcam_dev_open(dmcam_dev_t *dev); 
~~~
| 参数   | 描述     |
| :--- | :----- |
| dev  | 指定打开设备 |

#### 3.2.4设置参数
~~~C
dmcam_param_batch_set(dmcam_dev_t *dev, const dmcam_param_item_t *param_items, int item_cnt);
~~~
| 参数          | 描述      |
| :---------- | :------ |
| dev         | 指定设备    |
| param_items | 设置的参数值  |
| item_cnt    | 设置的参数个数 |

#### 3.2.5设置采集帧缓存
~~~C
dmcam_cap_set_frame_buffer(dmcam_dev_t *dev, uint8_t *frame_buf, uint32_t frame_buf_size);
~~~
| 参数             | 描述     |
| :------------- | :----- |
| dev            | 指定设备   |
| frame_buf      | 缓存数组   |
| frame_buf_size | 缓存数组大小 |

#### 3.2.6设置错误回调
~~~C
dmcam_cap_set_callback_on_error(dmcam_dev_t *dev, dmcam_cap_err_f cb);
~~~
| 参数   | 描述     |
| :--- | :----- |
| dev  | 指定设备   |
| cb   | 错误回调函数 |

#### 3.2.7采集帧数据
~~~C
dmcam_cap_get_frames(dmcam_dev_t *dev, uint32_t frame_num, uint8_t *frame_data, uint32_t frame_dlen, dmcam_frame_t *first_frame_info);
~~~
| 参数               | 描述      |
| :--------------- | :------ |
| dev              | 指定设备    |
| frame_num        | 要采集的帧数  |
| frame_data       | 采集的帧数据  |
| frame_dlen       | 帧数据缓存大小 |
| first_frame_info | 第一帧数据   |

#### 3.2.8解析出深度数据
~~~C
dmcam_frame_get_distance(dmcam_dev_t *dev, float *dst, int dst_len,uint8_t *src, int src_len, const dmcam_frame_info_t *finfo);
~~~
| 参数      | 描述       |
| :------ | :------- |
| dev     | 指定设备     |
| dst     | 转换出的深度数据 |
| dst_len | 深度数据缓存大小 |
| src     | 采集的原始数据  |
| src_len | 原始数据大小   |
| finfo   | 原始帧信息    |

#### 3.2.9解析出灰度数据
~~~C
dmcam_frame_get_gray(dmcam_dev_t *dev, float *dst, int dst_len,
uint8_t *src, int src_len, const dmcam_frame_info_t *finfo);
~~~
| 参数      | 描述       |
| :------ | :------- |
| dev     | 指定设备     |
| dst     | 转换出的灰度数据 |
| dst_len | 灰度数据缓存大小 |
| src     | 采集的原始数据  |
| src_len | 原始数据大小   |
| finfo   | 原始帧信息    |

#### 3.2.10获取点云数据
~~~C
dmcam_frame_get_pcl(dmcam_dev_t * dev, float *pcl, int pcl_len,
const float *dist, int dist_len, int img_w, int img_h, const dmcam_camera_para_t *p_cam_param);
~~~
| 参数          | 描述                   |
| :---------- | :------------------- |
| dev         | 指定设备                 |
| pcl         | 输出的点云数据，每三个元素构成一点的坐标 |
| dist        | 输入图像的深度数据            |
| dist_len    | 输入图像深度数据的长度          |
| img_w       | 深度图像的宽度              |
| img_h       | 深度图像的高度              |
| p_cam_param | 相机内参                 |
***
### 3.3相关样例说明
SmartToF SDK提供的主要样例如下：
- C样例：参见（[C样例链接](https://github.com/smarttofsdk/SDK/tree/master/windows/dmcam/samples/c)）
  - sample_capture_frames.c:展示采集数据
  - sample_set_param.c:展示进行参数的设置
  - sample_filter.c:展示校准数据
- python样例：参见（[python样例链接](https://github.com/smarttofsdk/SDK/tree/master/windows/dmcam/samples/python)）
  - test_gui_pyQtGraph.py:采集数据显示
  - test_gui_pygame.py:采集数据显示
  - test_param.py:展示参数设置和读取
  - test_raw:展示采集数据
- java样例：参见([java样例链接](https://github.com/smarttofsdk/SDK/tree/master/windows/dmcam/samples/java))
  - testBasic.java:java采集样例
  - testBasicUi.java:java采集显示样例
- ROS样例：参见([ros样例链接](https://github.com/smarttofsdk/SDK/tree/master/linux/dmcam/samples/ros/src/tof_sample))
  - tof_sample:展示如何获取图像和点云。

