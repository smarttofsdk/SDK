#include "smarttofDepthStream.h"
#include "SmartTofStreamImpl.h"
#include "dmcam.h"
#include "common.h"
static const int DEPTH_WIDTH = 320;
static const int DEPTH_HEIGHT = 240;

using namespace oni::driver;
using namespace smarttof;

smarttofStream::smarttofStream(SmartTofStreamImpl* pStreamImpl) :BaseSmartTofStream(pStreamImpl)
{
	m_videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
	m_videoMode.fps = DEFAULT_FPS;
	m_videoMode.resolutionX = SMARTTOF_RESOLUTION_X_320;
	m_videoMode.resolutionY = SMARTTOF_RESOLUTION_Y_240;
	m_frameId = 1;

}

smarttofStream:: ~smarttofStream()
{

}
/*
OniStatus smarttofStream::start()
{
	unsigned char *fbuf = new unsigned char[10 * 320 * 240 * 8]();

	if (fbuf == NULL)
		return ONI_STATUS_ERROR;
	
//	dmcam_dev_t *dev = smarttofDriver.dev;
	//dmcam_cap_start(dev);

	m_threadHandle = CreateThread(NULL, 0, threadFunc, NULL, 0, NULL);
	return ONI_STATUS_OK;
}

void smarttofStream::stop()
{
	m_running = false;
	CloseHandle(m_threadHandle);
}

void smarttofStream::Mainloop()
{
m_running = true;
while (m_running)
{
OniFrame *pFrame = getServices().acquireFrame();
BuildFrame(pFrame);
raiseNewFrame(pFrame);
getServices().releaseFrame(pFrame);
printf("get one frame\n");
}
}

DWORD WINAPI smarttofStream::threadFunc(LPVOID pThreadParam)
{
smarttofStream* pStream = (smarttofStream *)pThreadParam;
pStream->m_running = true;
pStream->Mainloop();
return 0;
}
int smarttofStream::BuildFrame(OniFrame* pFrame)
{
pFrame->frameIndex = m_frameId;

pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
pFrame->videoMode.resolutionX = DEFAULT_RESOLUTION_X;
pFrame->videoMode.resolutionY = DEFAULT_RESOLUTION_Y;
pFrame->videoMode.fps = 30;

pFrame->width = DEFAULT_RESOLUTION_X;
pFrame->height = DEFAULT_RESOLUTION_Y;

pFrame->cropOriginX = pFrame->cropOriginY = 0;
pFrame->croppingEnabled = FALSE;

pFrame->sensorType = ONI_SENSOR_DEPTH;
pFrame->stride = DEPTH_WIDTH * sizeof(OniDepthPixel);
pFrame->timestamp = m_frameId * 33000;
++m_frameId;
return 1;
}
*/

void smarttofStream::frameReceived(BaseSmartTofStream *stream, float *fdata,dmcam_frame_info_t& imageFrame)
{
	BaseSmartTofStream *pStream = stream;
	OniFrame* pFrame = getServices().acquireFrame();
	
	if (pFrame == NULL){
		printf("aquire frame failed\n");
		return;
	}
	printf("fsize:%d\n", pFrame->dataSize);
	unsigned short dist[320 * 240];
	unsigned short *pdata = (unsigned short *)pFrame->data;
	for (int i = 0; i < 320 * 240; i++)
	{
		dist[i] = (uint16_t)(fdata[i] * 100);
		pdata[i] = dist[i];
	}
	//memcpy(pFrame->data, fdata, pFrame->dataSize);
	pFrame->sensorType = ONI_SENSOR_DEPTH;
	pFrame->width = DEFAULT_RESOLUTION_X;
	pFrame->height = DEFAULT_RESOLUTION_Y;
	pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
	pFrame->videoMode.resolutionX = DEFAULT_RESOLUTION_X;
	pFrame->videoMode.resolutionY = DEFAULT_RESOLUTION_Y;
	pFrame->videoMode.fps = 30;
	pFrame->cropOriginX = pFrame->cropOriginY = 0;
	pFrame->croppingEnabled = FALSE;
	pFrame->frameIndex = m_frameId;
	pFrame->sensorType = ONI_SENSOR_DEPTH;
	pFrame->stride = DEFAULT_RESOLUTION_X*sizeof(OniDepthPixel);
	pFrame->timestamp = m_frameId * 33000;
	raiseNewFrame(pFrame);
	getServices().releaseFrame(pFrame);
	m_frameId++;
}

OniStatus smarttofStream::getProperty(int propertyId, void* data, int* pDataSize)
{
	//to do...
	if (propertyId == ONI_STREAM_PROPERTY_VIDEO_MODE)
	{
		if (*pDataSize != sizeof(OniVideoMode))
		{
			printf("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniVideoMode));
			return ONI_STATUS_ERROR;
		}
		return GetVideoMode((OniVideoMode*)data);
	}
	else if (propertyId == PROPERTY_ID_PARAM_GET){
		m_pStreamImpl->paramBatchGet((dmcam_param_item_t *)data, 1);
	}
	return ONI_STATUS_NOT_IMPLEMENTED;
}

OniStatus smarttofStream::setProperty(int propertyId, const void* data, int dataSize)
{
	//to do...
	switch (propertyId)
	{
	case PROPERTY_ID_PARAM_SET:
		m_pStreamImpl->paramBatchSet((dmcam_param_item_t *)data, dataSize/sizeof(dmcam_param_item_t));
		break;
	case PROPERTY_ID_FILTER_LEN_CALIB_ENABLE:
		m_pStreamImpl->filterenable(DMCAM_FILTER_ID_LEN_CALIB, (dmcam_filter_args_u *)data);
		break;
	case PROPERTY_ID_FILTER_LEN_CALIB_DISABLE:
		m_pStreamImpl->filterdisable(DMCAM_FILTER_ID_LEN_CALIB);
		break;
	case PROPERTY_ID_FILTER_PIXEL_CALIB_ENABLE:
		m_pStreamImpl->filterenable(DMCAM_FILTER_ID_PIXEL_CALIB, (dmcam_filter_args_u *)data);
		break;
	case PROPERTY_ID_FILTER_PIXEL_CALIB_DISABLE:
		m_pStreamImpl->filterdisable(DMCAM_FILTER_ID_PIXEL_CALIB);
		break;
	//case PROPERTY_ID_FILTER_KALMAN_ENABLE:
	//	m_pStreamImpl->filterenable(DMCAM_FILTER_ID_KALMAN, (dmcam_filter_args_u *)data);
	//	break;
	//case PROPERTY_ID_FILTER_KALMAN_DISABLE:
	//	m_pStreamImpl->filterdisable(DMCAM_FILTER_ID_KALMAN);
	//	break;
	//case PROPERTY_ID_FILTER_GAUSS_ENABLE:
	//	m_pStreamImpl->filterenable(DMCAM_FILTER_ID_GAUSS, (dmcam_filter_args_u *)data);
	//	break;
	//case PROPERTY_ID_FILTER_GAUSS_DISABLE:
	//	m_pStreamImpl->filterdisable(DMCAM_FILTER_ID_GAUSS);
	//	break;
	case PROPERTY_ID_FILTER_AMP_ENABLE:
		m_pStreamImpl->filterenable(DMCAM_FILTER_ID_AMP, (dmcam_filter_args_u *)data);
		break;
	case PROPERTY_ID_FILTER_AMP_DISABLE:
		m_pStreamImpl->filterdisable(DMCAM_FILTER_ID_AMP);
		break;
	case PROPERTY_ID_FILTER_AUTO_INTG_ENABLE:
		m_pStreamImpl->filterenable(DMCAM_FILTER_ID_AUTO_INTG, (dmcam_filter_args_u *)data);
		break;
	case PROPERTY_ID_FILTER_AUTO_INTG_DISABLE:
		m_pStreamImpl->filterdisable(DMCAM_FILTER_ID_AUTO_INTG);
		break;
	case PROPERTY_ID_FILTER_SYNC_DELAY_ENABLE:
		m_pStreamImpl->filterenable(DMCAM_FILTER_ID_SYNC_DELAY, (dmcam_filter_args_u *)data);
		break;
	case PROPERTY_ID_FILTER_SYNC_DELAY_DISABLE:
		m_pStreamImpl->filterdisable(DMCAM_FILTER_ID_SYNC_DELAY);
		break;
	case PROPERTY_ID_FILTER_TEMP_MONITOR_ENABLE:
		m_pStreamImpl->filterenable(DMCAM_FILTER_ID_TEMP_MONITOR, (dmcam_filter_args_u *)data);
		break;
	case PROPERTY_ID_FILTER_TEMP_MONITOR_DISABLE:
		m_pStreamImpl->filterdisable(DMCAM_FILTER_ID_TEMP_MONITOR);
		break;
	case PROPERTY_ID_FILTER_HDR_ENABLE:
		m_pStreamImpl->filterenable(DMCAM_FILTER_ID_HDR, (dmcam_filter_args_u *)data);
		break;
	case PROPERTY_ID_FILTER_HDR_DISABLE:
		m_pStreamImpl->filterdisable(DMCAM_FILTER_ID_HDR);
		break;
	}
	return ONI_STATUS_OK;
}

OniStatus smarttofStream::setVideoMode(OniVideoMode* pVideoMode)
{
    memcpy(&m_videoMode,pVideoMode,sizeof(m_videoMode));
    return ONI_STATUS_OK;
}

OniStatus smarttofStream::getVideoMode(OniVideoMode *pVideoMode)
{
	pVideoMode->pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
	pVideoMode->fps = DEFAULT_FPS;
	pVideoMode->resolutionY = DEFAULT_RESOLUTION_Y;
	pVideoMode->resolutionX = DEFAULT_RESOLUTION_X;
	return ONI_STATUS_OK;
}
OniBool smarttofStream::isPropertySupported(int propertyId)
{
	return 0;
}

void smarttofStream::notifyAllProperties()
{

}

void smarttofStream::populateFrameImageMetadata(OniFrame* pFrame, int dataUnitSize)
{

}
void smarttofStream::copyDepthPixelsStraight(const float* source, int numPoints, OniFrame* pFrame)
{

}
void smarttofStream::copyDepthPixelsWithImageRegistration(const float* source, int numPoints, OniFrame* pFrame)
{

}