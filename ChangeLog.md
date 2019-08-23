# 版本号：1.72

## 发布时间：2019/8/23

### 固件部分 固件版本：172

- [**Bugfix**] 修复运动模式图像异常问题

### SDK软件部分
- dmcam lib
  - [**New**] 增加“深度滤波强度” 参数的支持
    - 索尼传感器部分
      - [**Enhance**] 优化频率切换逻辑
      - [**Bugfix**] 修复原始深度图qi_shift计算问题
      - [**Bugfix**] 修复有校准数据的双频向无校准数据的单频切换的问题
  - [**Enhance**] 优化dev_open速度：PARAM_INFO_CAPABILITY获取并入get_info
  - [**Bugfix**] 修复多线程进行采集和停止时候，usb cancle导致的无法采集问题
- tools
  - smarttofviewer
    - [**New**] 调整binning设置到basic setting； fmt 设置到adv setting
    - [**New**] UI支持FILTER_STRENGTH , offset设置放入高级参数
    - [**Enhance**] 优化viewer打开速度和scan设备的卡顿
    - [**Bugfix**] 修复设备切换无效问题
    - [**Bugfix**] 修复首次设备打开选择不正确的问题
    - [**Bugfix**] 修复binning控件属性问题
- ros
  - [**Bugfix**] 修复编译时的路径问题
- samples
  - [**Enhance**] 更Android相关库和样例
  - [**Enhance**] 更新OpenNI样例



# 版本号：1.70

## 发布时间：2019/8/5

### 固件部分 固件版本：170

- [**New**] Binning设置支持 
- [**New**] 增加EPC635支持
- [**New**] 固件名称修改
- [**New**] 调整binning设置，可通过ROI参数中binning参数设置，统一ROI参数设置
- [**Bugfix**] 修复ROI 列问题
- [**Bugfix**] 修复笔记本采集超时问题

### SDK软件部分
- dmcam lib
  - [**New**] 增加binning设置功能
  - [**New**] 修改EPC部分roi的row设置，ROI参数设置含义调整
  - [**New**] 增加EPC635的支持
    - 索尼传感器部分
      - [**New**] imx双频计算模式支持
      - [**New**] 降低bpf滤波强度
      - [**New**] 
  - [**Enhance**] 优化dm_stream的TRACE打印
  - [**Enhance**] 提升roi相关的打印信息
  - [**Bugfix**] 丢掉废弃的API:dmcam_cmap_float
  - [**Bugfix**] 修复EPC的binning录像问题
  - [**Bugfix**] 修复初始参数为HDR模式录像回放的问题
- tools
  - smarttofviewer
    - [**New**] 更新封面图片
    - [**New**] 增加binning设置功能
    - [**New**] 增加dmcam_dist_u16 录像
    - [**Bugfix**] 修复binning模式回放问题
    - [**Bugfix**] 修复校准拟合不正确的问题，增加校准值校验和修复功能
    - [**Bugfix**] 修复pclviewer和smarttofviewer视角不一致的问题
  - smarttof_cli
    - [**New**] 支持-u方式指定device_uri,支持ONI录像设备的使用
    - [**Enhance**] 修改ROI设置更新
    - [**Bugfix**] 修复capture命令在file replay时候的部分问题
- ros
  - [**New**] ros增加查找dmcam库的cmake
  - [**Enhance**] 丢弃使用相关废弃接口，同步到最新的库
- samples
  - [**Enhance**] 更Android相关库和样例
  - [**Enhance**] 更新OpenNI样例



# 版本号：1.68

## 发布时间：2019/3/30

### 固件部分 固件版本：168

- [**Bugfix**] 新MCU升级兼容性
- [**Bugfix**] 修复时间戳误差大和滞后问题
- [**Bugfix**] E3固件帧率问题修复
- [**Bugfix**] 运动模式灰度帧大小计算问题

### SDK软件部分
- dmcam lib
  - [**New**] 增加dmcam_frame_get_pcl_xyzi接口获得包括灰度纹理的点云
  - [**New**] 增加dmcam_cap_seek_frame接口以便在播放录像文件时查找帧
  - [**New**] 增加dmcam_frame_get_dist_raw接口获得原始的不带校准数据的深度数据
    - 索尼传感器部分
      - [**New**] 为索尼传感器增加压缩支持
      - [**New**] 增加索尼传感器的滤波器优化
      - [**New**] 索尼部分校准
  - [**Enhance**] 从固件164版本后录像支持环境光校准的灰度帧
  - [**Enhance**] 录像支持HDR模式和普通模式的切换
  - [**Enhance**] 对网络设备dmcam_dev_get_uri返回带token标志的URI
  - [**Enhance**] 提高dmcam_frame_get_pcl_xyzd和dmcam_frame_get_pcl_xyzi的计算性能
  - [**Bugfix**] 修复镜头参数文件打开后未关闭的问题
  - [**Bugfix**] 修复录像文件没有时间戳的问题
  - [**Bugfix**] 针对固件164版本设备有时不能正常关闭的问题
- tools
  - smarttofviewer
    - [**New**] 界面增加开启点云显示选项
    - [**New**] 界面增加”上下反转选项“，去掉“抗干扰检测”
    - [**New**] 增加深度图-原始视图，显示raw格式
    - [**New**] 增加“重复播放“选项，回放时候默认选中
    - [**Enhance**] 优化帧率界面设置方式
    - [**Bugfix**] 修复HDR开启的录像播放问题
- ros
  - [**Enhance**] 丢弃使用相关废弃接口，同步到最新的库
- samples
  - [**New**] 更新python下样例使用最新的接口
  - [**Enhance**] 更Android相关库和样例
  - [**Enhance**] 更新OpenNI样例
- doc
  - [**Enhance**] 更新版SmartToF SDK User Guide ,在readthedoc上托管


# 版本号：1.62

## 发布时间：2019/1/30

### 固件部分 固件版本：164

- [**New**] 增加行ROI支持
- [**New**] 增加环境光补偿支持
- [**NEW**] 调整缓冲RAM布局
- [**Bugfix**] 修复时间戳毫秒部分错误
- [**Bugfix**] 支持温度高自动降帧

### SDK软件部分
- dmcam lib
  - [**New**] 采集设置API接口变更，增加dmcam_cap_config_set()接口，可统一设置帧缓冲和是否开启录像
  - [**New**] 增加环境光补偿
  - [**Enhance**] 大幅优化计算处理速度
  - [**Enhance**] 调整get_pcl接口输出和深度图一样的画幅（一一映射），如深度值无效则为（0,0,0）或（0,0,0,0）
  - [**Enhance**] 更新系统校准计算
  - [**Bugfix**] 修复调用接口复位时usb锁住问题
  - [**Bugfix**] 修复get_frames在处理平台速度较慢的时候请求多帧只返回1帧的问题。
  - [**Bugfix**] 修复TC-E3录制和回放帧数不对的问题
- tools
  - smarttofviewer
    - [**New**] 录像增加支持OpenNI兼容模式
    - [**Enhance**] 优化linux显示布局
    - [**Bugfix**] 修复中文路径问题，修复切换设备和录像之间状态不正常的问题
    - [**Bugfix**] 修复固件文件名带路径出错问题
    - [**Bugfix**] 修复距离偏移启动时不更新的问题
  - smarttof_cli
    - [**New**] 增加HDR积分时间设置，采集存储按行扫描存储
    - [**New**] info命令增加打印URI信息
    - [**New**] 增加filter、采集等命令
    - [**New**] 复位增加提示信息
    - [**Bugfix**] 修复多参数异常及像素坐标点问题
- ros
  - [**Bugfix**] 修复ros运行时深度图和点云图显示时卡顿问题
- Openni
  - [**New**] 发布Openni支持模组驱动的源码
- samples
  - [**New**] 增加C#样例get/set param 代码片段
  - [**New**] 更新python下样例使用最新的接口
  - [**Enhance**] 更Android相关库和样例
  - [**Enhance**] 更新OpenNI样例
- doc
  - [**New**] 改版SmartToF SDK User Guide ,在readthedoc上托管
  - [**New**] 增加运动模式场景说明

# 版本号：1.60

## 发布时间：2018/9/23

### 固件部分

- [**New**] 帧数据信息增加时间戳信息，精确到0.1ms
- [**New**] 支持系统校准参数存储、读取
- [**Enhance**]  运动模式优化
- [**Bugfix**] 修复上电红外灯闪烁一次问题

### SDK软件部分
- dmcam lib
  - [**New**] 采集设置API接口变更，增加dmcam_cap_config_set()接口，可统一设置帧缓冲和是否开启录像
  - [**New**] 增加dmcam_dev_open_by_uri， 可通过URI 打开指定的USB或ETHERNET或录像文件设备
  - [**New**] 支持录像和播放录像文件功能
  - [**New**] 增加应用端系统校准参数存取
  - [**New**] 增加帧获取时间戳，时间戳位于dmcam_frame_info_t
  - [**New**] 修改获取深度和灰度数据接口(dmcam_frame_get_dist/gray_u16/f32，增加更多的颜色转换接口(dmcam_cmap_dist_u16/f32_to_RGB, dmcam_cmap_gray_u16/f32_to_IR)
  - [**Enhance**] 优化运动模式接口
  - [**Enhance**] gray下支持运动模式
- tools
  - smarttofviewer
    - [**New**] 增加录像和播放录像功能
    - [**New**] 增加支持多设备选择 
  - smarttof_cli
    - [**New**] 增加rx replay命令，可以进行dmcam录像的获取
    - [**Enhance**] 优化help文字
    - [**Bugfix**] 修复固件文件名带路径出错问题
- samples
  - [**Enhance**] 更新c/c++采集接口设置样例。增加sample_save_replay展示录像的例子
  - [**Enhance**] 更新java采集接口样例
  - [**Enhance**] 更新C#采集接口样例
  - [**Enhance**] 更Android相关库和样例
  - [**Enhance**] 更新OpenNI样例
- doc
  - [**New**] 更新SmartToF SDK User Guide ,上传网上托管



# 版本号：1.56

## 发布时间 ： 2018/09/21

### 固件部分
- [**Bugfix**] 修复温度及帧率切换上层missing frame问题
- [**Bugfix**]修复设置低帧率自动升帧问题
- [**New**]增加运动模式支持
- [**New**]加快温度更新频率

### SDK软件部分

- dmcam lib
  - [**New**] 支持运动模式0和1，大幅消除降低物体运动时的重影问题。
  - [**New**] 支持UDP cap
  - [**Enhance**] 日志打印优化
  - [**Enhance**] 大幅优化HDR效果
  - [**Bugfix**] 修复正常模式HDR问题
  - [**Bugfix**] 修复filter_enable接口无效ID返回true的问题
  - [**Bugfix**] 修复dmcam_get_frames可能超时的问题
- C#扩展
  - [**New**] 提供C#环境所需要的库
- tools
  - smarttofviewer
    - [**New**] 增加运动模式0、1控制支持
    - [**New**] 灰度模式显示平均幅值
  - smartofcli
    - [**New**] CLI工具改为静态编译
- samples
  - c#
    - [**New**] 增加c#使用dmcam库接口样例

# 版本号：1.54

## 发布时间 ： 2018/08/28

### 固件部分

- [**Bugfix**] 修复图像个别像素点错位问题，由版本150引入的出现条纹问题。
- [**Bugfix**] 恢复帧数据校验

### SDK软件部分

- dmcam lib
  - [**New**] 支持TCM-E3型号的120fps采集模式
  - [**Enhance**] 优化深度滤波性能
  - [**Enhance**] API接口线程安全模式支持
  - [**Enhance**] 提升校准计算性能
  - [**Bugfix**] 修复温度采集出错时引起的20厘米左右的误差
  - [**Bugfix**] 修复dmcam_cap_get_frame接口调用一次清空当前frame buffer问题
  - [**Bugfix**] 修复首次加载校准数据offset问题
  - [**Bugfix**] 修复旧校准数据升级后图像问题
- cli
  - [**Enhance**] rx命令优化
  - [**Enhance**] regrd/regwr 读入地址支持16 进制输入
  - [**Enhance**] 增强打印说明
- tools
  - smarttofviewer
    - [**New**] 增加帧率、温度、幅值、精度偏差等显示
    - [**Enhance**] 提高点云显示效果和平面平整度

# 版本号：1.50

## 发布时间 ： 2018/08/10
## 注意
对模组校准数据格式进行了修改，使用新版SDK需要模组固件全部更新到1.50

### 固件部分
- 修复增加36MHZ引入的读取帧率参数错误
- bootloader校验问题修复，增加固件合法性检测
- 修复图像断层问题
- 修复图像丢部分行像素问题

### SDK软件部分
- dmcam lib
  - 优化校准数据加载及存储格式
  - 增加dm_gausfilter模块，dm_conv模块
  - 支持2组不同频率校准数据切换，默认使用第一组校准数据
  - 优化代码层次结构，优化内存，优化代码执行效率
- ros
  - 修复点云显示不完整
- tools
  - smarttofviewer
    - 增加固件版本低于143的弹框警告
    - 优化UI布局，增加waring显示
    - 增加频率切换支持

# 版本号：1.43
## 发布时间：2018/07/26
## 注意
对模组固件进行了修复，所有模组必须全部更新到SDK V1.43中的固件。

### 固件部分
- 修复0℃以下环境IB板温度采集问题
- 修改系统温度更新机制
- 修复ADC采集超时图像卡顿问题
- 修复固件中GPIO初始化存在冲突问题
- 增强抗干扰性（PC其他USB设备插拔的干扰）

### SDK软件部分
- dmcam lib
  - 修复drnu可能的内存访问错误
  - 修复默认不启用pix calib的问题
  - 修复校准数据不带校准频率导致的simple calib参数错误
  - 更新java使用simple calib
- tools 
  - Smarttof cli
    - 修复采集数据丢失问题
  - Smarttofviewer
    - 修复smarttofviewer不能正常退出的问题


# 版本号：1.42

## 发布时间：2018/07/13

## 注意
在linux下使用SDK V1.31及以后版本必须要更新模组固件，模组升级文件位于SDK中的firmware目录。

### 固件部分
- 同SDK V1.40版本

### SDK软件部分
- dmcam lib
  - 增加优化的filter：gauss、flynoise、fillhole
  - 进一步优化点云效果
  - 修复连续cap_start导致的问题
  - 改进实测距离偏小问题
  - 改进快速算法，优化内存占用
  - 修复内存泄漏
# 版本号：1.40

## 发布时间：2018/07/05

## 注意
在linux下使用SDK V1.31及以后版本必须要更新模组固件，模组升级文件位于SDK的firmware目录。

### 固件部分
- 同SDK V1.32版本，没有更新

### SDK软件部分
- dmcam lib
  - 增加点云优化处理，默认使能fliter_id为DMCAM_FILTER_ID_MEDIAN,用户可以自行关闭
  - 修复HDR显示问题
  - 修复灰度图时内存泄漏问题
- tools
  - SmarttofViewer
    - 增加深度图滤波复选框
    - 大幅优化点云显示效果
- firmware
  - 增加firmware文件夹，存放固件升级相关文件




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
