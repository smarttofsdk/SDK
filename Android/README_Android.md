# Android 发布包介绍

------

SDK的Android文件夹下提供了用于在Android下模组app开发的库和参考样例，并提供了apk安装文件。

# 文件夹说明

- lib文件夹：Android下开发app连接smarttof所必须的库
- tools:
  - bin: 演示android手机连接模组的apk安装文件
  - src: android下开发app的样例

# 使用说明

- 使用提供的apk
  1. 直接将bin文件夹里的apk安装到手机上
  2. 使用otg线将手机和模组相连
  3. 打开app运行，点击采集按钮
- 通过src文件夹下的SmartTOFViewer-android样例进行编译生成apk
  1. 配置开发Android的eclipse环境
  2. 将src文件夹下的样例工程导入eclipse
  3. 编译生成apk，安装到手机运行