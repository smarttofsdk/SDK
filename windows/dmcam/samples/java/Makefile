X64?=0

DMCAM_TOP_DIR:=$(shell pwd)/../../
DMCAM_BIN_DIR:=$(DMCAM_TOP_DIR)/lib
ifeq ($(X64),1)
	ARCH:=x64
	CROSS_COMPILE:=x86_64-w64-mingw32-
	JAVA_HOME:=c:/Program\ Files/Java/jdk1.8.0_151
else
	ARCH:=x86
	CROSS_COMPILE:=i686-w64-mingw32-
	JAVA_HOME:=c:/Program\ Files\ \(x86\)/Java/jdk1.8.0_151
endif

#DMCAM_ANDROID_DIR:=$(DMCAM_TOP_DIR)/build/android/dmcam_jar

JAVA:=$(JAVA_HOME)/bin/java.exe
JAVAC:=$(JAVA_HOME)/bin/javac.exe
JAR:=$(JAVA_HOME)/bin/jar.exe

MOD_NAME:=dmcam
MOD_DYNAMIC_TARGET:=$(MOD_NAME)_java.dll
MOD_TARGET_JAR:=$(MOD_NAME)_notest.jar
MOD_TARGET_TEST_JAR:=$(MOD_NAME)_test.jar
MOD_PACK:=com.smarttof.dmcam

MOD_SRC_OUT:=$(subst .,/,$(MOD_PACK))
SWIG_SRC:=$(MOD_NAME).i

CC=$(CROSS_COMPILE)gcc
CFLAGS:=-Wall -O -mthreads -I$(JAVA_HOME)\\include \
	   -I$(JAVA_HOME)\\include\\win32 -I $(DMCAM_BIN_DIR)/include
LDFLAGS:= -L$(DMCAM_BIN_DIR)/$(ARCH) -ldmcam -mthreads -Wl,--add-stdcall-alias

#-------------------------------------------------
JAVA_SRCS := $(wildcard $(MOD_SRC_OUT)/*.java)
JAVA_SRCS_TEST:= $(wildcard $(MOD_SRC_OUT)/test/*.java)
JAVA_OBJS := $(JAVA_SRCS:.java=.class) 
JAVA_TEST_OBJS := $(JAVA_SRCS_TEST:.java=.class)
JAVA_OBJS_JAR_RAW := $(JAVA_OBJS) $(wildcard $(MOD_SRC_OUT)/test/*.class)
JAVA_OBJS_JAR := $(subst $$,\$$,$(JAVA_OBJS_RAW))

.PHONY: all jar jar-test

all: $(MOD_SRC_OUT) $(MOD_DYNAMIC_TARGET) 
	@$(MAKE) -f $(MAKEFILE_NAME) jar jar-test

$(MOD_SRC_OUT):
	@mkdir -p $@

$(MOD_NAME)_wrap.c: $(SWIG_SRC)
	swig -Wall -java -package $(MOD_PACK) -outdir $(MOD_SRC_OUT) $<
ifeq (,$(wildcard $(DMCAM_TOP_BIN)/*))
	$(error dmcam bin dir not found!)
endif

$(MOD_DYNAMIC_TARGET):$(MOD_NAME)_wrap.o
	$(CC) -shared -o $@ $^ $(LDFLAGS)

$(JAVA_SRCS): $(MOD_DYNAMIC_TARGET)

%.class : %.java
	$(JAVAC) $<

#current file name
MAKEFILE_NAME := $(word $(words $(MAKEFILE_LIST)), $(MAKEFILE_LIST))

jar: $(MOD_TARGET_JAR)

jar-test: $(MOD_TARGET_TEST_JAR)

$(MOD_TARGET_JAR): $(JAVA_OBJS)
	$(JAR) cvf $@ $^

$(MOD_TARGET_TEST_JAR): $(JAVA_OBJS) $(JAVA_TEST_OBJS)
	$(JAR) cvf $@ $^ $(MOD_SRC_OUT)/test/*.class

#--- install --
install:
	@echo "Install binary to bin/java ..."
	@mkdir -p $(DMCAM_BIN_DIR)/java/$(ARCH)
	@cp -a $(MOD_TARGET_TEST_JAR) $(DMCAM_BIN_DIR)/java/$(MOD_NAME).jar 
	@cp -a $(MOD_DYNAMIC_TARGET) $(DMCAM_BIN_DIR)/java/$(ARCH)/

ifeq (,$(wildcard $(DMCAM_ANDROID_DIR)/*))
	$(error dmcam android dir not found!)
endif

test: 
	#cp -a $(DMCAM_BIN_DIR)/$(ARCH)/*.dll .
	$(JAVA) -cp $(MOD_TARGET_TEST_JAR) com.smarttof.dmcam.test.testBasicUi
#---
show:
	@echo "SOURCES: $(JAVA_SRCS)"
	#@echo "OBJS: $(JAVA_OBJS)"
	@echo "CC: $(CC)"

clean:
	rm -f *.o $(MOD_DYNAMIC_TARGET) *.c $(MOD_SRC_OUT)/*.java $(MOD_SRC_OUT)/*.class *.jar

