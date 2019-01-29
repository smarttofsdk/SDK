#ifndef _SMARTTOFDRIVER_H_
#define _SMARTTOFDRIVER_H_

#include "Driver\OniDriverAPI.h"
#include "dmcam.h"
namespace smarttof
{
	class smarttofDriver : public oni::driver::DriverBase
	{
	public:
		smarttofDriver(OniDriverServices* pDriverServices);
		virtual OniStatus initialize(oni::driver::DeviceConnectedCallback connectedCallback,
			oni::driver::DeviceDisconnectedCallback disconnectedCallback,
			oni::driver::DeviceStateChangedCallback deviceStateChangedCallback,
			void* pCookie);
		virtual ~smarttofDriver();

		virtual oni::driver::DeviceBase* deviceOpen(const char* uri, const char* mode);
		virtual void deviceClose(oni::driver::DeviceBase* pDevice);

		virtual void shutdown();
		
		dmcam_dev_t *device_handle;
	    dmcam_dev_t* get_device_handle(void);
		//static void set_device_handle(dmcam_dev_t *dev);
	private:
		dmcam_dev_t *dev;
		oni::driver::DeviceBase* m_devices;
	};
}

#endif