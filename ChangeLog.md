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
