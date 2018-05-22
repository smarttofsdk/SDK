#include <OpenNI.h>
#include "dmcam.h"
#include "common.h"

int main(int argc, char** argv)
{
	
		openni::Status rc = openni::STATUS_OK;

		openni::Device device;
		openni::VideoStream depth, color;
		const char* deviceURI = openni::ANY_DEVICE;
		//const char* deviceURI = "SMARTTOFURI";
		unsigned short intg = 1000;

		if (argc > 1)
		{
			deviceURI = argv[1];
		}

		rc = openni::OpenNI::initialize();

		//ni
		if (rc != openni::STATUS_OK)
		{
			printf("smarttof initialization failed\n");
		}

		printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

		rc = device.open(deviceURI);
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
			openni::OpenNI::shutdown();
			return 1;
		}
		else{
			printf("smarttof viewer open ok\n");
		}

		rc = depth.create(device, openni::SENSOR_DEPTH);
		if (rc == openni::STATUS_OK)
		{
			//rc = depth.start();
			//if (rc != openni::STATUS_OK)
			//{
			//	printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			//	depth.destroy();
			//}
		}
		else
		{
			printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			return 0;
		}
		{//*param setting */
			dmcam_param_item_t wparam;  //**setting integration time
			wparam.param_id = PARAM_INTG_TIME;
			wparam.param_val.intg.intg_us = intg;
			wparam.param_val_len = sizeof(wparam.param_val.intg.intg_us);
			depth.setProperty(PROPERTY_ID_PARAM_SET, (void *)&wparam, sizeof(wparam));
		}
		//get intg ...
		{
			dmcam_param_item_t rpm_intg;
			rpm_intg.param_id = PARAM_INTG_TIME;
			rpm_intg.param_val_len = sizeof(rpm_intg.param_val.intg.intg_us);
			depth.getProperty(PROPERTY_ID_PARAM_GET, &rpm_intg);
			printf("INTG is:%d us\n", rpm_intg.param_val.intg.intg_us);
		}

		//filter enable minamp ...
		{
			dmcam_filter_args_u amp_min_val;
			//dmcam_filter_id_e filter_id = DMCAM_FILTER_ID_AMP;
			amp_min_val.min_amp = 30;
			depth.setProperty(PROPERTY_ID_FILTER_AMP_ENABLE, &amp_min_val);
			printf("set filter amp enable is %d\n", amp_min_val.min_amp);

		}

		{/*param getting*/
			dmcam_param_item_t rparam;  //**getting framerate
			rparam.param_id = PARAM_FRAME_RATE;
			rparam.param_val_len = sizeof(rparam.param_val.frame_rate.fps);
			depth.getProperty(PROPERTY_ID_PARAM_GET, &rparam);
			printf("frame rate:%d fps\n", rparam.param_val.frame_rate.fps);

		}
		Sleep(1000);
#if 0
		openni::VideoFrameRef frameDepth;


		printf("Start process depth data\n");
		for (int i = 0; i < 10; ++i)
		{
			rc = depth.readFrame(&frameDepth);
			printf("%d ,%d %d,**********\n", frameDepth.getDataSize(), frameDepth.getVideoMode().getResolutionX(), frameDepth.getVideoMode().getResolutionY());
			if (openni::STATUS_OK == rc)
			{
				printf("Read %d frame ok\n", i);
			}
			else
			{
				printf("Read frame failed\n");
			}
		}
#endif
		depth.destroy();
		device.close();
		getchar();
		openni::OpenNI::shutdown();
		return 0;
}