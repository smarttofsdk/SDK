#ifndef _BASE_SMARTTOF_STREAM_H_
#define _BASE_SMARTTOF_STREAM_H_

#include "Driver\OniDriverAPI.h"

#include "dmcam.h"

namespace smarttof{
	class SmartTofStreamImpl;
	static const int SMARTTOF_RESOLUTION_X_80 = 80;
	static const int SMARTTOF_RESOLUTION_Y_60 = 60;
	static const int SMARTTOF_RESOLUTION_X_320 = 320;
	static const int SMARTTOF_RESOLUTION_Y_240 = 240;
	static const int SMARTTOF_RESOLUTION_X_640 = 640;
	static const int SMARTTOF_RESOLUTION_Y_480 = 480;
	static const int SMARTTOF_RESOLUTION_X_1280 = 1280;
	static const int SMARTTOF_RESOLUTION_Y_960 = 960;

	class BaseSmartTofStream :public oni::driver::StreamBase
	{
	public:
		BaseSmartTofStream(SmartTofStreamImpl* pStreamImpl);
		virtual ~BaseSmartTofStream();
		virtual OniStatus start();
		virtual void stop();
		virtual OniStatus getProperty(int propertyId, void *data, int *pDataSize);
		virtual OniStatus setProperty(int propertyId, const void *data, int DataSize);
		virtual OniBool isPropertySupported(int propertyId);
		virtual OniStatus SetVideoMode(OniVideoMode* pVideoMode);

		virtual OniStatus GetVideoMode(OniVideoMode* pVideoMode);

		virtual OniStatus SetCropping(OniCropping* cropping);

		virtual OniStatus GetCropping(OniCropping* cropping);

		bool isRunning() { return m_running; }
		virtual void frameReceived(BaseSmartTofStream *stream, float *fdata, dmcam_frame_info_t& finfo) = 0;
//		virtual void setServices(StreamServices* pStreamServices){ m_pServices = pStreamServices; }
//		StreamServices& getServices(){ return *m_pServices; }
	protected:
		SmartTofStreamImpl *m_pStreamImpl;
		OniVideoMode m_videoMode;
		OniCropping m_cropping;
		bool m_running;

	private:
//		StreamServices *m_pServices;
		void destroy();

	};
}//namespace smarttof
#endif