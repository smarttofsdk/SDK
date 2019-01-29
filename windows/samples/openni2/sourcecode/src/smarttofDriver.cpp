#include "smarttofDriver.h"
#include "smarttofDevice.h"


#include "Driver\OniDriverAPI.h"
#include "dmcam.h"

#define FRAME_SIZE 320*240*2*4
#define FRAME_BUF_FCNT 10

using namespace oni::driver;
using namespace smarttof;


#define XN_NEW(type, ...)               new type(__VA_ARGS__)
#define XN_DELETE(p)                    delete (p)
ONI_EXPORT_DRIVER(smarttof::smarttofDriver);

smarttofDriver::smarttofDriver(OniDriverServices* pDriverServices)
	:DriverBase(pDriverServices)
{}

smarttofDriver::~smarttofDriver()
{}

OniStatus smarttofDriver::initialize(DeviceConnectedCallback connectedCallback,
	DeviceDisconnectedCallback disconnectedCallback,
	DeviceStateChangedCallback deviceStateChangedCallback,
	void* pCookie)
{
	DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, pCookie);

	dmcam_init(NULL);
	//dmcam_dev_t *dev;
	int dev_cnt;
	dmcam_dev_t dev_list[4];
	
	printf("2 into smarttof.dll\n");

	dev_cnt = dmcam_dev_list(dev_list, 4);
	if (dev_cnt == 0)
	{
		return ONI_STATUS_NO_DEVICE;
	}

	//get sensor info and report them to OpenNI2
	OniDeviceInfo deviceInfo;

	if (dev_cnt != 0)
	{
#ifdef HAVE_STRCPY_S
		strcpy_s(deviceInfo.uri, ONI_MAX_STR, "SMARTTOFURI");
		strcpy_s(deviceInfo.vendor, ONI_MAX_STR, "SHUJI");
		strcpy_s(deviceInfo.name, ONI_MAX_STR, "TOF SENSOR");
#else
		strncpy(deviceInfo.uri, "SMARTTOFURI", ONI_MAX_STR);
		strncpy(deviceInfo.vendor, "SHUJI", ONI_MAX_STR);
		strncpy(deviceInfo.name, "TOF SENSOR", ONI_MAX_STR);
#endif
		deviceInfo.usbVendorId = 0x111b;		//!!! 指定我们的usbid
		deviceInfo.usbProductId = 0x1238;	//!!! 指定我们的usbvid

		deviceConnected(&deviceInfo);
		deviceStateChanged(&deviceInfo, 0);

		return ONI_STATUS_OK;
	}

}

DeviceBase* smarttofDriver::deviceOpen(const char* uri, const char* /*mode*/)
{
	printf("smarttof dll use cmake................................\n");
	if (strcmp(uri, "SMARTTOFURI") == 0)
	{
		//initialize
		//dmcam_init(NULL);
		dmcam_dev_t *pdev;
		printf("start to open dev\n");
		pdev = dmcam_dev_open(NULL);
		if (!pdev)
		{
			return NULL;
		}
		printf("log dmcam_dev_open ok\n");
	
		smarttofDevice *pDevice = new smarttofDevice(pdev,getServices());
		pDevice->dev = pdev;
		//device_handle = pdev;
		//set_device_handle(pdev);
		/* set frame buffer to use internal buffer */
		dmcam_cap_set_frame_buffer(pdev, NULL, FRAME_SIZE * FRAME_BUF_FCNT);

		/* disable error callback for capturing */
		dmcam_cap_set_callback_on_error(pdev, NULL);

		/* disable frame ready callback */
		dmcam_cap_set_callback_on_frame_ready(pdev, NULL);
		return pDevice;
	}
	return NULL;
}
/*
void smarttofDriver::set_device_handle(dmcam_dev_t *dev)
{
	smarttofDriver::dev = dev;
}
dmcam_dev_t* smarttofDriver::get_device_handle(void)
{
	return smarttofDriver::dev;
}*/
void smarttofDriver::deviceClose(oni::driver::DeviceBase* pDevice)
{
	printf("close dev\n");
	if (device_handle){
		dmcam_dev_close(device_handle);
		printf("close dev ok\n");
	}
	delete pDevice;
}

void smarttofDriver::shutdown()
{

}
dmcam_dev_t* smarttofDriver::get_device_handle(void)
{
	return dev;
}
//ONI_EXPORT_DRIVER(smarttof::smarttofDriver)
