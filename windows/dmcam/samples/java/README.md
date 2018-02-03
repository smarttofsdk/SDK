This Makefile is used to convert C/C++ interface to java interface
1.Makesure java is installed, add JAVA_HOME to system path like follows:
http://jingyan.baidu.com/album/c85b7a6414f2ee003bac95d5.html?picindex=1
2.Edit "DMCAM_TOP_DIR" if needed(according to your dmcam lib path)
3.Edit "JAVA_HOME" in Makefile if needed(according to your java install folder)
4.Run make xxx,usage:
make X64=1  #64 bit use 
make        #32 bit use
make clean  #clean 
Note:Firt start will load caliration data, you should wait until the red led stop blinking, then restart the devcie and test it.