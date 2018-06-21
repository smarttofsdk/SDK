# SmartToF C 样例
***
## 在windows下运行
- 在windows下生成vs工程

1. 在windows/samples/c下新建文件夹vsbuild
2. 使用命令行工具或者msys2下mingw工具，进入vsbuild使用cmake生成vs工程，具体命令如下
~~~BASH
cd vsbuild
cmake .. -G "Visual Studio 12 2013"  //根据自身所装的vs版本
~~~
3. 生成工程后打开dmcam_c_sample.sln,编译生成C样例的可执行文件。

生成的vs工程对应关系

|版本|数字|
|:---|:---|
|Visual Studio 2005|8|
|Visual Studio 2008|9|
|Visual Studio 2010|10|
|Visual Studio 2012|11|
|Visual Studio 2013|12|
|Visual Studio 2015|14|
|Visual Studio 2017|15|

- windows下直接生成可执行文件
1. 在windows/samples/c下新建文件夹build
2. 使用命令行工具或者msys2,进入build使用cmake生成可执行文件，具体命令参考如下
~~~BASH
cd build
cmake .. -G "MSYS Makefiles"
make -j
~~~
3.编译生成可执行文件后可双击运行

## 在linux下运行
1. 将对应版本的libdmcam.so拷贝到/usr/lib目录下
2. 打开终端，安装cmake
~~~BASH
sudo apt-get install cmake
~~~
3. 在linux/sample/c下新建文件夹build，在终端运行下面命令
~~~BASH
cd build
cmake .. -G "Unix Makefiles"
make -j
~~~
运行生成的可执行文件。

## C样例说明
1 sample_capture_frames展示模组的基本使用，从连接到采集数据及最后原始采集数据的处理。样例基本概括展示了使用模组采集数据所用到的相关API，具体的API说明参考([SmartToF_SDK重要接口说明](https://github.com/smarttofsdk/SDK/wiki/SmartToF-SDK-%E9%87%8D%E8%A6%81API%E8%AF%B4%E6%98%8E))。

2 sample_filter展示对原始数据的一些滤波处理所用到的两个主要API-- 
~~~C
dmcam_filter_enable(dmcam_dev_t *dev,  dmcam_filter_id_e fid, dmcam_filter_args_u *args, uint32_t arg_len)

dmcam_filter_disable(dmcam_dev_t *dev,  dmcam_filter_id_e fid)
~~~
dmcam_filter_id_e 和dmcam_filter_args_u的定义分别如下
~~~C
typedef enum {
    DMCAM_FILTER_ID_LEN_CALIB,  /**>lens calibration*/
    DMCAM_FILTER_ID_PIXEL_CALIB, /**>pixel calibration*/
    DMCAM_FILETER_ID_KALMAN,    /**>Kalman filter for distance data*/
    DMCAM_FILETER_ID_GAUSS,     /**>Gauss filter for distance data*/
    DMCAM_FILTER_ID_AMP, /**>Amplitude filter control*/
    DMCAM_FILTER_ID_AUTO_INTG, /**>auto integration filter enable : use sat_ratio to adjust */
    DMCAM_FILTER_ID_SYNC_DELAY,//Delay module capture start in random ms,capture sync use
    DMCAM_FILTER_ID_TEMP_MONITOR,//Monitor Module temperature
    DMCAM_FILTER_ID_HDR,
    DMCAM_FILTER_CNT,
}dmcam_filter_id_e;

typedef union {
    uint8_t case_idx; /**>User Scenario index */
    uint32_t lens_id; /**>length index*/
    uint32_t min_amp; /**>Min amplitude threshold*/
    uint16_t sat_ratio; /**>saturation ratio threshold*/
}dmcam_filter_args_u;
~~~
对照相应的参数实行滤波控制。

3 sample_set_param主要展示对模组的参数设置以及参数获取，包括模组的模式、帧率、积分时间等，相关主要API为dmcam_param_batch_set和dmcam_param_batch_get，具体的API说明参考([SmartToF_SDK重要接口说明](https://github.com/smarttofsdk/SDK/wiki/SmartToF-SDK-%E9%87%8D%E8%A6%81API%E8%AF%B4%E6%98%8E))。
