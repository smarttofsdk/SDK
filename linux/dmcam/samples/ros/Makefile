SRC:=src

.PHONY: all test init_ws build

all: $(SRC)/CMakeLists.txt build

$(SRC)/CMakeLists.txt:
	@cd $(SRC); catkin_init_workspace; cd -

build:
	@catkin_make

test: 
	sudo cp ../../bin/x64/libdmcam.so /usr/lib/
	roscore &
	rosrun image_view image_view image:=/smarttof/image_dist &
	. devel/setup.sh;  roslaunch smart_tof start.launch
