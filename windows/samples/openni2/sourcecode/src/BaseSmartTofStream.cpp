#include "BaseSmartTofStream.h"
#include "SmartTofStreamImpl.h"

using namespace oni::driver;
using namespace smarttof;

BaseSmartTofStream::BaseSmartTofStream(SmartTofStreamImpl* pStreamImpl):
m_pStreamImpl(pStreamImpl)
{
	m_running = false;
	m_cropping.enabled = FALSE;
	pStreamImpl->addStream(this);
	//StreamServices *m_streamServices = new StreamServices();
	//setServices(m_streamServices);
}

BaseSmartTofStream::~BaseSmartTofStream()
{

}
OniStatus BaseSmartTofStream::start()
{
	if (m_pStreamImpl != NULL){
		OniStatus status = m_pStreamImpl->start();
		if (status == ONI_STATUS_OK)
			m_running = TRUE;
		return status;
	}else{
		printf("Stream is NULL\n");
	}
}

void BaseSmartTofStream::stop()
{
	m_running = FALSE;
	m_pStreamImpl->stop();
}

void BaseSmartTofStream::destroy()
{
	stop();
	m_pStreamImpl->removeStream(this);
}

/*Not implement now*/
OniStatus BaseSmartTofStream::getProperty(int propertyId, void* data, int* pDataSize)
{
	OniStatus status = ONI_STATUS_NOT_SUPPORTED;
	switch (propertyId)
	{
	case ONI_STREAM_PROPERTY_CROPPING:
		if (*pDataSize != sizeof(OniCropping))
		{
			printf("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniCropping));
			status = ONI_STATUS_ERROR;
		}
		else
		{
			status = GetCropping((OniCropping*)data);
		}
		break;
	case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:
	{
		float* val = (float*)data;
		//XnDouble tmp;
		double tmp = 0;
		if (m_videoMode.resolutionX == 640)
			//tmp = NUI_CAMERA_COLOR_NOMINAL_HORIZONTAL_FOV * xnl::Math::DTR;
			tmp = 2.0;
		else
		tmp = 1.0;
		//	tmp = NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV * xnl::Math::DTR;
		*val = (float)tmp;
		status = ONI_STATUS_OK;
		break;
	}
	case ONI_STREAM_PROPERTY_VERTICAL_FOV:
	{
		float* val = (float*)data;
		//XnDouble tmp;
		double tmp=0;
		if (m_videoMode.resolutionY == 480)
			tmp = 1.0;
			//tmp = NUI_CAMERA_COLOR_NOMINAL_VERTICAL_FOV * xnl::Math::DTR;
		else
			tmp = 2.0;
			//tmp = NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV * xnl::Math::DTR;
		*val = (float)tmp;
		status = ONI_STATUS_OK;
		break;
	}
	case ONI_STREAM_PROPERTY_VIDEO_MODE:
	{
		if (*pDataSize != sizeof(OniVideoMode))
		{
			printf("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniVideoMode));
			status = ONI_STATUS_ERROR;
		}
		else
		{
			status = GetVideoMode((OniVideoMode*)data);
		}

		break;
	}
	default:
		status = ONI_STATUS_NOT_SUPPORTED;
		break;
	}

	return status;
}


OniStatus BaseSmartTofStream::setProperty(int propertyId, const void* data, int dataSize)
{
	OniStatus status = ONI_STATUS_NOT_SUPPORTED;
	if (propertyId == ONI_STREAM_PROPERTY_CROPPING)
	{
		if (dataSize != sizeof(OniCropping))
		{
			printf("Unexpected size: %d != %d\n", dataSize, sizeof(OniCropping));
			status = ONI_STATUS_ERROR;
		}
		status = SetCropping((OniCropping*)data);
	}
	else if (propertyId == ONI_STREAM_PROPERTY_VIDEO_MODE)
	{
		if (dataSize != sizeof(OniVideoMode))
		{
			printf("Unexpected size: %d != %d\n", dataSize, sizeof(OniVideoMode));
			status = ONI_STATUS_ERROR;
		}
		status = SetVideoMode((OniVideoMode*)data);
	}
	return status;
}

OniBool BaseSmartTofStream::isPropertySupported(int propertyId)
{
	OniBool status = FALSE;
	switch (propertyId)
	{
	case ONI_STREAM_PROPERTY_CROPPING:
	case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:
	case ONI_STREAM_PROPERTY_VERTICAL_FOV:
	case ONI_STREAM_PROPERTY_VIDEO_MODE:
		status = TRUE;
		break;
	default:
		status = FALSE;
		break;
	}
	return status;
}
OniStatus BaseSmartTofStream::SetVideoMode(OniVideoMode* videoMode)
{
	if (!m_pStreamImpl->isRunning())
	{
		m_videoMode = *videoMode;
		m_pStreamImpl->setVideoMode(videoMode);
		return ONI_STATUS_OK;
	}

	return ONI_STATUS_OUT_OF_FLOW;
}

OniStatus BaseSmartTofStream::GetVideoMode(OniVideoMode* pVideoMode)
{
	*pVideoMode = m_videoMode;
	return ONI_STATUS_OK;
}

OniStatus BaseSmartTofStream::SetCropping(OniCropping* cropping)
{
	m_cropping = *cropping;
	return ONI_STATUS_OK;
}

OniStatus BaseSmartTofStream::GetCropping(OniCropping* cropping)
{
	*cropping = m_cropping;
	return ONI_STATUS_OK;
}
