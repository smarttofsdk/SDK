# Smarttof 样例概述
***
对于针对Windows、Linux平台的开发人员，提供了基于C/C++、JAVA、python等示例，这些示例基于smarttof的dmcam库进行图像采集和转换处理。
## C examples

### sample_capture_frames
这个C样例展示对模组的一个基本使用，包扩查询连接模组，打开模组，模组采集和采集数据处理。
样例首先查询连接的模组数，然后连接模组，最后开始采集，并且展示了采集前的相关设置。采集进行时也将采集到的部分原始数据进行深度数据、灰度数据和点云数据的转换。采集满指定的帧数后退出程序。

### sample_filter
这个C样例主要展示对模组校准的相关设置，包括鱼眼校准的使能，出厂校准的相关使能和用户校准的开启关闭，重点是熟悉对dmcam_filter_enable和dmcam_filter_disable这几个主要API的掌握。

### sample_set_param
这个C样例主要展示如何获取模组的参数以及进行设置。样例中展示了如对模组频率等单个参数的设置和获取，同时也展示了对模组批量参数的设置和获取。
***

## Java example
这个java样例配合SDK中相关的java库，展示一个简单的采集显示样例。

***

## python example

### sample_basic.py
这个python样例展示了在python环境下对模组dmcam库的一个基本使用，包括模组的连接、打开和采集，便于用户了解在python下进行模组开发的一个基本流程。

### sample_param.py
这个python样例展示了在python环境下对模组的参数进行获取和设置，样例中先对模组的模式、帧率和积分时间进行了批量设置，后对模组的频率、帧率、版本信息等进行了批量读取。

### sample_gui_pygame.py
这个python样例展示在python环境下对模组进行数据采集，并将采集的数据转换成深度、灰度等数据，配合pygame库分别显示在几个子窗口中。

### sample_gui_pyQtGraph.py
这个python样例展示如何获取图像数据，应用pyQtGraph等相关GUI显示出来，要运行这个样例要安装相关的库，具体需要安装的库参考python样例目录下的requirement.txt。




