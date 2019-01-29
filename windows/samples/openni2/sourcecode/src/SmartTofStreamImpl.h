#ifndef __SMARTTOF_STREAM_IMPL_H_
#define __SMARTTOF_STREAM_IMPL_H_


#include "BaseSmartTofStream.h"
//#include "Driver\OniDriverAPI.h"



#include "dmcam.h"

namespace smarttof{
class SmartTofStreamImpl{
	public:
		SmartTofStreamImpl(dmcam_dev_t* pSensor, OniSensorType sensorType);

		virtual ~SmartTofStreamImpl();

		void addStream(BaseSmartTofStream* stream);

		void removeStream(BaseSmartTofStream* stream);

		unsigned int getStreamCount();

		void setVideoMode(OniVideoMode* videoMode);

		OniStatus virtual start();

		void virtual stop();

		bool	isRunning() { return m_running; }

		OniSensorType getSensorType() { return m_sensorType; }

		void setSensorType(OniSensorType sensorType);

		void mainLoop();

		OniStatus setAutoWhiteBalance(BOOL val);

		OniStatus getAutoWhitBalance(BOOL *val);

		OniStatus setAutoExposure(BOOL val);

		OniStatus getAutoExposure(BOOL *val);
		OniStatus filterenable(dmcam_filter_id_e id, dmcam_filter_args_u *args);
		OniStatus filterdisable(dmcam_filter_id_e id);
		OniStatus paramBatchSet(const dmcam_param_item_t*param_items, int item_cnt);
		OniStatus paramBatchGet(dmcam_param_item_t*param_items, int item_cnt);
		OniImageRegistrationMode getImageRegistrationMode() const { return m_imageRegistrationMode; }

		void setImageRegistrationMode(OniImageRegistrationMode mode) { m_imageRegistrationMode = mode; }

		OniStatus convertDepthToColorCoordinates(oni::driver::StreamBase* colorStream,
			int depthX, int depthY, OniDepthPixel depthZ, int* pColorX, int* pColorY);

		static int getSmartTOFImageResolution(int resolutionX, int resolutionY);
		
		dmcam_dev_t* getSmartTofSensor(){ return m_pSmartTofSensor; }
	private:
		void setDefaultVideoMode();
		int getSmartTofImageType();
		static DWORD WINAPI threadFunc(LPVOID pThreadParam);
		OniImageRegistrationMode m_imageRegistrationMode;
		SmartTofStreamImpl *m_pDepthStream;
		BaseSmartTofStream *m_pBaseStream;
		//Thread
		dmcam_dev_t* m_pSmartTofSensor;
		OniSensorType m_sensorType;
		bool m_running;
		HANDLE m_hStreamHandle;
		HANDLE m_hNextFrameEvent;
		OniVideoMode m_videoMode;

		HANDLE m_threadHandle;
	};
};//endof smarttof namespace
#endif