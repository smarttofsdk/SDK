#ifndef _SMARTTOF_DEPTHSTREAM_H_
#define _SMARTTOF_DEPTHSTREAM_H_

#include "smarttofDevice.h"
#include "Driver\OniDriverAPI.h"
#include "BaseSmartTofStream.h"
#include "dmcam.h"
#define DEFAULT_RESOLUTION_X 320
#define DEFAULT_RESOLUTION_Y 240
#define DEFAULT_FPS 30

namespace smarttof
{
	class smarttofStream : public BaseSmartTofStream
	{
	public:
		//smarttofStream(OniSensorType sensorType);
		smarttofStream(SmartTofStreamImpl *pStreamImpl);
		virtual ~smarttofStream();

		virtual void frameReceived(BaseSmartTofStream *stream, float *fdata, dmcam_frame_info_t& imageFrame);


		virtual OniStatus getProperty(int propertyId, void* data, int* pDataSize);

		virtual OniStatus setProperty(int propertyId, const void* data, int dataSize);

		virtual OniBool isPropertySupported(int propertyId);

		virtual void notifyAllProperties();
		int m_frameId;
	protected:
		//smarttofDevice &m_device;
		OniStatus setVideoMode(OniVideoMode *);
		OniStatus getVideoMode(OniVideoMode *);
		BaseSmartTofStream *m_pDepthStream;
	
	private:
		void populateFrameImageMetadata(OniFrame* pFrame, int dataUnitSize);
		void copyDepthPixelsStraight(const float* source, int numPoints, OniFrame* pFrame);
		void copyDepthPixelsWithImageRegistration(const float* source, int numPoints, OniFrame* pFrame);
	};
}

#endif