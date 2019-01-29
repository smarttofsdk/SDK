#include "smarttofDevice.h"


#include "dmcam.h"

using namespace oni::driver;
using namespace smarttof;

//smarttofDevice::smarttofDevice(dmcam_dev_t* dev)
//{
//    m_numSensors = 1;
//    dev = dmcam_dev_open(NULL);
//}

smarttofDevice::smarttofDevice(dmcam_dev_t *mdev, oni::driver::DriverServices& driverServices) :dev(mdev), m_driverServices(driverServices)
{
	m_numSensors = 1;
	m_sensors[0].sensorType = ONI_SENSOR_DEPTH;
	m_sensors[0].numSupportedVideoModes = 1;
	m_sensors[0].pSupportedVideoModes = new OniVideoMode[1];
	m_sensors[0].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
	m_sensors[0].pSupportedVideoModes[0].fps = DEFAULT_FPS;
	m_sensors[0].pSupportedVideoModes[0].resolutionX = DEFAULT_RESOLUTION_X;
	m_sensors[0].pSupportedVideoModes[0].resolutionY = DEFAULT_RESOLUTION_Y;

}

smarttofDevice::~smarttofDevice()
{

}

OniStatus smarttofDevice::getSensorInfoList(OniSensorInfo** pSensors, int* numSensors)
{
	*numSensors = m_numSensors;
	*pSensors = m_sensors;
	return ONI_STATUS_OK;
}

StreamBase* smarttofDevice::createStream(OniSensorType sensorType)
{

	if (sensorType == ONI_SENSOR_DEPTH)
	{
		printf("Create depth stream\n");
		m_pDepthStream = new SmartTofStreamImpl(dev, sensorType);
		smarttofStream *pImage = new smarttofStream(m_pDepthStream);
		m_driverServices.errorLoggerAppend("smarttofDevice: create a stream of type %d", sensorType);
		return pImage;
	}
//	m_driverServices.errorLoggerAppend("smarttofDevice:can't create a stream of type %d", sensorType);
	return NULL;
}

void smarttofDevice::destroyStream(oni::driver::StreamBase* pStream)
{
	//no_op
}

OniStatus smarttofDevice::setProperty(int propertyId, const void* data, int dataSize)
{
	printf("set ptoperty\n");
	return ONI_STATUS_OK;
}

OniStatus smarttofDevice::getProperty(int propertyId, void* data, int* pDataSize)
{
	switch (propertyId)
	{
	case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:
		return ONI_STATUS_OK;
	default:
//		m_driverServices.errorLoggerAppend("Unknown property: %d\n", propertyId);
		return ONI_STATUS_ERROR;
		break;
	}
}

OniBool smarttofDevice::isPropertySupported(int propertyId)
{
	return (propertyId == ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION);
}



