set PATH=D:\msys32\usr\bin;%PATH%
set JAVA_HOME=C:\Java64\jdk1.8.0_151

cp -a ../libdmcam/bin/android/* ./libs/

call F:\Development_IDE\JAVA_NDK_ECLIPSE\android-sdk-windows\android-sdk-windows\tools\android.bat update project --path . --name SmartTOFViewer 
call F:\Development_IDE\android_ndk\eclipse\plugins\org.apache.ant_1.9.2.v201404171502\bin\ant clean debug
