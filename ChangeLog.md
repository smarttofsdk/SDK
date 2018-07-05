# 版本号：1.40

## 发布时间：2018/07/05

## 注意
在linux下使用SDK V1.31及以后版本必须要更新模组固件，模组升级文件位于SDK的firmware目录。

### 固件部分
- 同SDK V1.32版本，没有更新

### SDK软件部分
- dmcam lib
  - 增加点云优化处理
  - 修复HDR显示问题
- tools
  - SmarttofViewer
    - 增加深度图滤波复选框
    - 大幅优化点云显示效果




# 版本号：1.32

## 发布时间：2018/06/21

## 注意

SDK v1.31及其以上版本在linux下不兼容以前的模组固件，所以在linux下使用SDK v1.31必须要更新模组固件；执行fw_upgrade.bat脚本进行模组升级。

## 主要修改：

### 固件部分

- 修改CB板温度更新机制
- 版本号132

### SDK软件部分

- dmcam lib
  - 修复HDR灰度图问题
  - 增加禁用DRNU像素校准支持
  - 软件增强USB传输抗干扰处理
  - 优化温度补偿系数
- Android
  - 同步更新SmartTofViewer apk
- Samples
  - 增加C++样例
- tools 
  - SmartTofViewer
    - 界面增加“像素校准”复选框
    - 修复底层stall导致viewer异常错误问题
  - SmartTofCli
    - 增加固件与模组硬件是否匹配检测

# 版本号：1.31

## 发布时间：2018/05/25

## 注意

SDK v1.31版本在linux下不兼容以前的模组固件，所以在linux下使用SDK v1.31必须要更新模组固件，对于HDR、温度保护等新特性 windows和linux下都需要更新模组固件，固件位置在SmarttofCli中

## 主要修改：

### 固件部分

- 温度保护最低降至1帧，温度降下来后支持自动升帧率
- 增强切换帧率稳定性

### SDK软件部分

- dmcam lib
  - 修复低帧率timeout问题
  - 修复HDR和镜头校准同时使用问题
  - 大幅优化数据转换接口（raw2dist,raw2gray）性能
  - 修复frame_get_xyzd接口输出颜色可能不正常问题
  - 默认镜头参数使用镜头TRC-2150D2(122度)的参数
  - 修复平面物体点云图像弯曲问题，优化点云显示效果
- tools 
  - PCLViewer
    - 优化显示及调整效果

# 版本号：1.30

## 发布时间：2018/05/21

## 主要修改：

### 固件部分
  - 增加HDR支持
  - 修复USB异常断开，灯板未关闭问题
  - 增加温度保护机制
  - 增加帧数据校验
  - 修复数据帧增加padding后，低帧率采集问题
  - 修复电源指示灯采集中熄灭问题

### SDK软件部分
- dmcam lib
  - 增加HDR支持
  - 调整数据帧头，优化传输速度
  - 增加帧数据校验支持
  - 修复android 打开设备问题
  - 修复参数长度溢出问题
- Android
  - 发布Android平台支持
    - 提供Android平台所需要的库
    - 提供Android平台smarttofviewer工具和源码
- Samples
  - 发布windows下Openni2的驱动库
  - 增加使用Openni2样例
- tools 
  - SmartTofViewer
    - 界面增加HDR选项和HDR曝光时间调整条
  - SmartTofCli
    - 修复采集时获取数据错误等bug
- doc
  - 样例
  - README_OpenNI2.md
  - 应用指南
  - 《SmartTof FAQ 手册》
  - 《SmartTof SDK 简介》
  - 《SmartTof_Cli 说明文档》
  - 《SmartTof怎样消除多模组同时使用时的串扰》
- 其他
  - 在linux下增加使用模组设备的权限设置的脚本




# 版本号：1.20

## 发布时间：2018/04/13

## 主要修改：

- dmcam lib
  - 优化温度校准补偿
  - 增加温度保护机制
  - 增加多模组干扰软件滤波支持
- ROS
  - 增加ROS平台支持
    - 提供深度、灰度、点云、相机参数4个topic
    - 提供修改参数service
    - 提供ROS扩展的样例
- Android
  - 增加Android平台支持
    - 提供Android平台smarttofviewer工具及源码
    - 提供Android平台所需要的库
- samples
  - 完善样例readme
- tools
  - SmartToFCli
    - 增加保存灰度、深度、点云数据命令支持
    - 增加固件更新脚本
  - SmartToFViewer 
    - 界面增加多模组滤波干扰复选框
- doc
  - 样例readme
    - README_C_sample.md
    - README_Python_sample.md
    - README_sample.md
  - 应用指南
    - 《SMARTTof_SDK_ROS用户手册.pdf》
    - 《TCM-Ex模组固件升级说明.pdf》
    - 《SmartTof怎样获得不定距离的最佳图像.pdf》
    - 《SmartTof怎样获得最佳图像.pdf》

# 版本号：1.15

## 发布时间：2018/03/09
## 主要修改：
- dmcam lib
  - 增加静态编译库(.a)和visual studio库(.lib)
  - 支持自动曝光时间模式
- python
  - 提升python扩展稳定性
  - 增加windows 64bit python支持
    - win下目前支持 python 2.7/3.4/3.5/3.6 的32bit、64bit版本
    - Linux下目前支持 python 2.7/3.4/3.5 的64bit版本
- java
  - 增加linux java扩展
    - win下支持java 32bit, 64bit
    - Linux下支持java 64bit
- samples
  - 为c/c++ sample增加CMakelist.txt，可直接在发布目录下编译
- tools
  - SmartToFCli
    - 默认进入interactive模式，-h显示帮助
  - SmartToFViewer 
    - 优化灰度图显示
    - 支持自动曝光时间模式

# 版本号：1.12
## 发布时间：2018/02/28
## 主要修改：
 * 增加温度补偿
 * 支持帧率设置
 * 优化灰度图显示
