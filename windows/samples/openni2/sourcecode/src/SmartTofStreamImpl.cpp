#include "SmartTofStreamImpl.h"
#include "smarttofDevice.h"
//#include "smarttofdriver.h"
//#include "Driver\OniDriverAPI.h"
#include "BaseSmartTofStream.h"
using namespace oni::driver;
using namespace smarttof;

#define DEFAULT_FPS 30


SmartTofStreamImpl::SmartTofStreamImpl(dmcam_dev_t *pSensor, OniSensorType sensorType) :
m_pSmartTofSensor(pSensor), m_sensorType(sensorType), m_running(FALSE)
{
	setDefaultVideoMode();
}
SmartTofStreamImpl::~SmartTofStreamImpl()
{
	if (m_running)
	{
		//DWORD exitCode;
		m_running = FALSE;
		//ExitThread(GetExitCodeThread(m_threadHandle, &exitCode));
		//CloseHandle(&m_threadHandle);
	}
	if (m_hNextFrameEvent != INVALID_HANDLE_VALUE){
		//CloseHandle(m_hNextFrameEvent);
	}
}

void SmartTofStreamImpl::addStream(BaseSmartTofStream* stream)
{
	printf("Add new stream\n");
	m_pBaseStream = stream;
}

void SmartTofStreamImpl::removeStream(BaseSmartTofStream* stream)
{

}
unsigned int SmartTofStreamImpl::getStreamCount()
{
	return 1;
}
void SmartTofStreamImpl::setVideoMode(OniVideoMode* videoMode)
{
	m_videoMode.fps = videoMode->fps;
	m_videoMode.pixelFormat = videoMode->pixelFormat;
	m_videoMode.resolutionX = videoMode->resolutionX;
	m_videoMode.resolutionY = videoMode->resolutionY;
}

OniStatus SmartTofStreamImpl::start()
{
	if (m_running != TRUE)
	{
		printf("start capture\n");
		m_hStreamHandle = CreateThread(NULL, 0, threadFunc, this, 0, NULL);
		dmcam_cap_start(m_pSmartTofSensor);
		return ONI_STATUS_OK;
	}
	else
	{
		return ONI_STATUS_OK;
	}
}

void SmartTofStreamImpl::stop()
{
	if (m_running == true)
	{
		m_running = false;
		//stop capture here
		printf("stop capture\n");
		dmcam_cap_stop(m_pSmartTofSensor);
	}
}

void SmartTofStreamImpl::setSensorType(OniSensorType sensorType)
{
	if (m_sensorType != sensorType)
	{
		m_sensorType = sensorType;
		setDefaultVideoMode();
	}
}

static const unsigned int LOOP_TIMEOUT = 10;
#define FRAME_SIZE  (320*240*2*4)
void SmartTofStreamImpl::mainLoop()
{
	m_running = true;
	int frameId = 1;
	unsigned char *fbuf = new unsigned char[FRAME_SIZE];
	if (NULL == fbuf)
	{
		return;
	}

	while (m_running)
	{
	//	if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextFrameEvent, LOOP_TIMEOUT) && m_running)
		{
			dmcam_frame_t finfo;	
			int ret,w,h;
			ret = dmcam_cap_get_frames(m_pSmartTofSensor, 1, fbuf, FRAME_SIZE, &finfo);	
			w = finfo.frame_info.width;
			h = finfo.frame_info.height;
			if (ret > 0)
			{
				float *dist = new float[w*h];
				dmcam_frame_get_distance(m_pSmartTofSensor, dist,w*h, fbuf, FRAME_SIZE, &finfo.frame_info);
				Sleep(3);
				m_pBaseStream->frameReceived(m_pBaseStream, dist, finfo.frame_info);
				delete(dist);
			}
			Sleep(27);
		}
	}
}

OniStatus SmartTofStreamImpl::setAutoWhiteBalance(BOOL val)
{
	return ONI_STATUS_OK;
}

OniStatus SmartTofStreamImpl::getAutoWhitBalance(BOOL *val)
{
	return ONI_STATUS_OK;
}

OniStatus SmartTofStreamImpl::setAutoExposure(BOOL val)
{
	dmcam_param_item_t wparam;
	wparam.param_id = PARAM_INTG_TIME;
	wparam.param_val.intg.intg_us = val;
	wparam.param_val_len = sizeof(wparam.param_val.intg.intg_us);
	if (dmcam_param_batch_set(m_pSmartTofSensor, &wparam, 1)<0)
	{
		return ONI_STATUS_ERROR;
	}
	else{
	return ONI_STATUS_OK;
	}
}

OniStatus SmartTofStreamImpl::convertDepthToColorCoordinates(StreamBase* colorStream, int depthX, int depthY, OniDepthPixel depthZ, int* pColorX, int* pColorY)
{
	return ONI_STATUS_OK;
}

void SmartTofStreamImpl::setDefaultVideoMode()
{
	switch (m_sensorType)
	{
	case ONI_SENSOR_COLOR:
		m_videoMode.pixelFormat = ONI_PIXEL_FORMAT_RGB888;
		m_videoMode.fps = DEFAULT_FPS;
		m_videoMode.resolutionX = SMARTTOF_RESOLUTION_X_320;
		m_videoMode.resolutionY = SMARTTOF_RESOLUTION_Y_240;
		break;

	case ONI_SENSOR_DEPTH:
		m_videoMode.pixelFormat =  ONI_PIXEL_FORMAT_DEPTH_1_MM;
		m_videoMode.fps = DEFAULT_FPS;
		m_videoMode.resolutionX = SMARTTOF_RESOLUTION_X_320;
		m_videoMode.resolutionY = SMARTTOF_RESOLUTION_Y_240;
		break;

	case ONI_SENSOR_IR:
		m_videoMode.pixelFormat = ONI_PIXEL_FORMAT_GRAY8;
		m_videoMode.fps = DEFAULT_FPS;
		m_videoMode.resolutionX = SMARTTOF_RESOLUTION_X_320;
		m_videoMode.resolutionY = SMARTTOF_RESOLUTION_Y_240;
		break;
	default:
		;
	}
}
int SmartTofStreamImpl::getSmartTofImageType()
{
	return 0;
}
DWORD WINAPI SmartTofStreamImpl::threadFunc(LPVOID pThreadParam)
{
	SmartTofStreamImpl *pStream = (SmartTofStreamImpl*)pThreadParam;
	pStream->mainLoop();
	return 0;
}
OniStatus SmartTofStreamImpl::filterenable(dmcam_filter_id_e id, dmcam_filter_args_u *args)
{
	printf("filter enable\n");
	return ONI_STATUS_OK;
}
OniStatus SmartTofStreamImpl::filterdisable(dmcam_filter_id_e id)
{
	printf("filter disable\n");
	return ONI_STATUS_OK;
}
OniStatus SmartTofStreamImpl::paramBatchSet(const dmcam_param_item_t*param_items, int item_cnt)
{
	printf("param set\n");
	if (dmcam_param_batch_set(m_pSmartTofSensor, param_items, 1)<0)
	{
		return ONI_STATUS_ERROR;
	}
	return ONI_STATUS_OK;
}
OniStatus SmartTofStreamImpl::paramBatchGet(dmcam_param_item_t*param_items, int item_cnt)
{
	printf("param get\n");
	if (dmcam_param_batch_get(m_pSmartTofSensor, param_items, 1)<0)
	{
		return ONI_STATUS_ERROR;
	}
	return ONI_STATUS_OK;
}











