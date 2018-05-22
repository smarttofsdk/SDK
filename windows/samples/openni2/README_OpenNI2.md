# 支持SmartTof模组的OpenNI2驱动
***
这是为了提供Smarttof模组对OpenNI2的支持而发布的驱动，支持的平台是windows，支持的OpenNI的版本是OpenNI2 V2.2。目前的版本主要支持通过OpenNI2传输深度数据流，并且可以通过OpenNI2的的参数设置来对模组进行如帧率等设置。需要注意的是有些OpenNI2发布包里所提供的例子需要彩色视频流，这些例子在我们提供的SmartTof 驱动里不能正常运行。OpenNI2发布包里提供的NiViewer2工具除了会打开深度数据流，也会尝试去打开彩色和IR数据流，但是会显示打开失败。

# 在windows下使用OpenNI2

在windows平台上，需要将支持OpenNI2的smarttof的相关动态库拷贝到安装OpenNI2的指定目录下，这里以OpenNI2 V2.2版本为例，需要将提供的smarttof模组驱动库拷贝到安装目录下的\Redist\OpenNI2\Drivers下。如果直接使用OpenNI2提供的NiViewer工具，则将smarttof的驱动库文件拷贝到NiViewer目录下的Tools\OpenNI2\Drivers下，这时连上模组后可以直接通过NiViewer使用模组进行采集和显示。
需要拷贝的支持OpenNI的文件包括以下几个：
- smarttof.dll
- libdmcam.dll
- libdmcam.h

# 使用Openni的smarttof样例
SDK中提供了使用openni进行模组数据采集和参数基本设置的样例工程，样例基本概括展示了如何使用Openni的进行模组的使用，在样例工程中，如果使用采集样例将sample_set_param.cpp从生成中排除，使用参数设置样例则将main.cpp从生成中排除。
- 采集样例
在采集样例中，展示了通过Openni采集了模组的200帧数据，设备的URI定为SMARTTOFURI,或者直接为ANY_DEVICE。
- 参数设置样例
参数设置样例主要展示一些模组特有的参数设置和获取以及一些滤波功能的开启和关闭，这里的相关设置主要通过Openni的setProperty和getProperty设置和获取。下面具体展示进行参数设置和获取的具体代码。
~~~C
//获取参数
dmcam_param_item_t rpm_intg;
rpm_intg.param_id = PARAM_INTG_TIME;	//读取参数为积分时间
rpm_intg.param_val_len = sizeof(rpm_intg.param_val.intg.intg_us);
depth.getProperty(PROPERTY_ID_PARAM_GET, &rpm_intg);//获取积分时间
printf("INTG is:%d us\n", rpm_intg.param_val.intg.intg_us);
~~~
~~~C
//设置参数
dmcam_param_item_t wparam;
wparam.param_id = PARAM_INTG_TIME;  //设置参数为积分时间
wparam.param_val.intg.intg_us = 1000;//设置积分时间的大小
wparam.param_val_len = sizeof(wparam.param_val.intg.intg_us);
depth.setProperty(PROPERTY_ID_PARAM_SET, (void *)&wparam, sizeof(wparam));
~~~
上面是演示模组积分时间的获取和设置，其他如帧率等其他参数设置可以参考上面的代码。除了模组特有的相关参数设置，下面展示相关滤波功能使用：
~~~C
//开启最小幅值滤波功能
dmcam_filter_args_u amp_min_val;
amp_min_val.min_amp = 30;  //设置最小幅值滤波值
depth.setProperty(PROPERTY_ID_FILTER_AMP_ENABLE,&amp_min_val);//开启设置最小幅值滤波
printf("frame rate:%d fps\n", rparam.param_val.frame_rate.fps);
~~~

