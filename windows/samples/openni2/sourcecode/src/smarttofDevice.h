#ifndef _SMARTTOFDEVICE_H_
#define _SMARTTOFDEVICE_H_

#include "smarttofDepthStream.h"
#include "Driver\OniDriverAPI.h"
#include "SmartTofStreamImpl.h"
#include "dmcam.h"

namespace smarttof
{
	class smarttofDevice : public oni::driver::DeviceBase
	{
	public:
		//smarttofDevice(dmcam_dev_t* dev);
		smarttofDevice(dmcam_dev_t* mdev, oni::driver::DriverServices& driverServices);
		virtual ~smarttofDevice();

		virtual OniStatus getSensorInfoList(OniSensorInfo** pSensors, int* numSources);	//must

		virtual oni::driver::StreamBase *createStream(OniSensorType sensorType);	//must
		virtual void destroyStream(oni::driver::StreamBase* pStream);				//must

		virtual OniStatus setProperty(int propertyId, const void* data, int dataSize);
		virtual OniStatus getProperty(int propertyId, void* data, int* pDataSize);
		virtual OniBool isPropertySupported(int propertyId);

		//dmcam_dev_t* smarttofDevice::getCamera();
		 dmcam_dev_t* dev;
		 SmartTofStreamImpl *m_pStreamImpl;
		 SmartTofStreamImpl *m_pDepthStream;
	private:
		//smarttofStream* m_stream;
		int m_numSensors;
		OniSensorInfo m_sensors[1];
		OniDeviceInfo* m_pInfo;
		oni::driver::DriverServices& m_driverServices;	

	};
}
#endif