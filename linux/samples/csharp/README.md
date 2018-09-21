# Prerequisit # 
* Windows: visual studio c# 201X
* Linux: mono

# Build C# samples #

The samples can be build with CMAKE.

## windows ##

* Build using cmake, the <BUILD_TYPE> can be "Release", "Debug", "RelWithDebInfo"

```bash
mkdir build
cmake ..
cmake --build . --config <BUILD_TYPE>
```

 ## Linux ##

* Build using cmake. the <BUILD_TYPE> can be "Release", "Debug", "RelWithDebInfo"
```bash
mkdir build
cmake .. -DCMAKE_BUILD_TYPE=<BUILD_TYPE>
cmake --build . 
```



# Running C# Samples

* sample_basic : one connected device,  sample some frames and print data info.
* sample_ui:  basic ui example to capture and show distance images

You can simply run:

```bash
./sample_basic.exe
```

If some so is not found under linux, you may specify the LD_LIBRARY_PATH and run samples

```
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd) ./sample_basic
```

