# SmartToF Python 样例
***
针对Windows、Linux平台的python开发，提供适用于python的包文件，并提供样例展示如何在python下使用模组。

## python样例运行所需的库
- 安装支持smarttof模组的python扩展库
  SDK中分别为windows和linux平台提供了支持smarttof模组的python扩展库，并且对应不同的python版本，方便用户选择适合的开发环境。下图是python扩展库的版本支持列表。
  -  windows
  
  |支持版本|扩展库名|
  |:---|:---|
  |python2.7 32位|dmcam-xxx-cp27-cp27m-win32.whl|
  |python3.4 32位|dmcam-xxx-cp34-cp34m-win32.whl|
  |python3.5 32位|dmcam-xxx-cp35-cp35m-win32.whl|
  |python3.6 32位|dmcam-xxx-cp36-cp36m-win32.whl|
  |python2.7 64位|dmcam-xxx-cp27-cp27m-win_amd64.whl|
  |python3.4 64位|dmcam-xxx-cp34-cp34m-win_amd64.whl|
  |python3.5 64位|dmcam-xxx-cp35-cp35m-win_amd64.whl|
  |python3.6 64位|dmcam-xxx-cp36-cp36m-win_amd64.whl|
  - Linux
  
  |支持版本|扩展库名|
  |:---|:---|
  |python2.7 64位|dmcam-xxx-cp27-cp27mu-linux_x86_64.whl|
  |python3.4 64位|dmcam-xxx-cp34-cp34m-linux_x86_64.whl|
  |python3.5 64位|dmcam-xxx-cp35-cp35m-linux_x86_64.whl|
说明：上面扩展库名中的xxx是对应的发布版本号

- 安装支持样例运行的其他库

  ​需要安装numpy、matplotlib、pygame、PyQt5、pyqtgraph,可以通过以下命令安装
  ~~~BASH
  pip install -r requirement.txt
  ~~~

说明：在python2.7或者python3.4环境下安装PyQt5可能回导致失败，可以换成安装PyQt4。

## python样例的使用
- 注意和C库中的名称区别
在使用python开发时，先安装对导入dmcam包后，python中使用的API函数跟dmcam.h中的函数名有所区别。C库中的API形式如dmcam_xxx_xxx,例如dmcam_dev_open,而在python中调用dmcam库时，形式为dmcam.xxx_xxx,打开设备为dmcam.dev_open。
- python下基本参数设置和采集
  - python下最简采集流程
  ~~~PYTHON
  #初始化
  dmcam.init()
  ...
  #打开设备
  dev = dmcam.dev_open()
  #设置采集缓存
  dmcam.cap_set_frame_buffer(dev,None,FRAME_SIZE*FRAME_BUF_FCNT)
  ...
  #开始采集
  dmcam.cap_start(dev)
  #获得采集数据
  ret = dmcam.cap_get_frames(dev,1,f,finfo)
  w = finfo.frame_info.width
  h = finfo.frame_info.height
  #获得深度数据
  dist_cnt,dist = dmcam.frame_get_distance(dev,w*h,f,finfo.frame_info)
  #获取灰度数据
  gray_cnt,gray = dmcam.frame_get_gray(dev,w*h,f,finfo.frame_info)
  #停止采集
  dmcam.cap_stop(dev)
  ...
  dmcam.uninit()
  ~~~

  - python 下参数设置
  下面是对模组参数设置的示例代码，展示python下对模组的参数设置，这里是设置模式、帧率等
  ~~~PYTHON
  wparams ={
    dmcam.PARAM_DEV_MODE: dmcam.param_val_u(),
    dmcam.PARAM_FRAME_RATE: dmcam.param_val_u(),
    dmcam.PARAM_ILLUM_POWER: dmcam.param_val_u(),
  }
  wparams[dmcam.PARAM_DEV_MODE].dev_mode = 1
  wparams[dmcam.PARAM_FRAME_RATE].frame_rate.fps = 30
  wparams[dmcam.PARAM_ILLUM_POWER].illum_power.persent = 30
  ret = dmcam.param_batch_set(device,wparams)
  ~~~
  - python下滤波功能使用
  下面是在python下开启模组的幅值滤波、自动积分、多模组消除串扰等功能。
  ~~~PYTHON
  #开启幅值滤波
  amp_min_val = dmcam.filter_args_u()
  amp_min_val.min_amp = 30	#设置幅值滤波阈值
  dmcam.filter_enable(dev,dmcam.DMCAM_FILTER_ID_AMP,amp_min_val,sys.getsizeof(amp_min_val))	#开启幅值滤波
  
  # 开启自动积分时间
  intg_auto_arg = dmcam.filter_args_u()
  intg_auto_arg.sta_ratio = 5	#设置曝光点参数，一般为5
  dmcam.filter_enable(dev,dmcam.DMCAM_FILTER_ID_AUTO_INTG,intg_auto_arg,sys.getsizeof(intg_auto_arg))	#开启自动曝光
  
  # 开启消除串扰
  delay = dmcam.filter_args_u()
  delay.sync_delay = 0; #random delay
  if INTERFERENC_CHECK_EN:	#需检测是否使能
  dmcam.filter_enable(dev,dmcam.DMCAM_FILTER_ID_SYNC_DELAY,delay,sys.getsizeof(delay))
  
  ~~~

