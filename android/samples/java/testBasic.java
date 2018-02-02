package com.smarttof.dmcam.test;
import java.nio.ByteBuffer;

import com.smarttof.dmcam.*;

public class testBasic{
	public static void main(String[] args) {
		// dmcam.init(null);
		dmcam.log_cfg(log_level_e.LOG_LEVEL_NONE, log_level_e.LOG_LEVEL_WARN,
				log_level_e.LOG_LEVEL_NONE);
		dmcamDevArray devs = new dmcamDevArray(16);
		int cnt = dmcam.dev_list(devs.cast(), 16);

		System.out.printf("found %d device\n", cnt);
		for (int i = 0; i < cnt; i++) {
			System.out
					.printf("DMCAM#%d [%03d:%03d:%03d]: VENDOR=%s, PROD=%s, SERIAL=%s\n",
							i, devs.getitem(i).getUsb_port_num(),
							devs.getitem(i).getUsb_bus_num(), devs.getitem(i)
									.getUsb_dev_addr(), devs.getitem(i)
									.getVendor(), devs.getitem(i).getProduct(),
							devs.getitem(i).getSerial());
		}
		if (cnt == 0) {
			return;
		}
		System.out.println(" Open dmcam device ..");
		dev_t dev = dmcam.dev_open(null);
		if (dev == null) {
			System.out.println(" Open device failed");
			return;
		}

		int pwr_percent = 100;
		param_item_t wparam = new param_item_t();
		dmcamParamArray wparams = new dmcamParamArray(1);

		wparam.setParam_id(dev_param_e.PARAM_ILLUM_POWER);
		wparam.getParam_val().getIllum_power().setPercent((short) pwr_percent);

		wparams.setitem(0, wparam);
		System.out.printf(" pwr = %d\n", wparams.getitem(0).getParam_val()
				.getIllum_power().getPercent());
		if (!dmcam.param_batch_set(dev, wparams.cast(), 1)) {
			System.out.printf(" set illu_power to %d %% failed\n", pwr_percent);
		}

		dmcam.cap_set_frame_buffer(dev, null, 10 * 320 * 240 * 4);
		// dmcam.cap_set_callback_on_error(dev, null);
		System.out.println(" Start capture ...");
		dmcam.cap_start(dev);

		ByteBuffer f = ByteBuffer.allocateDirect(1920 * 1080 * 3); /* should be large enough to have one frame */
		System.out.println(" sampling 100 frames ...");
		int count = 0;
		boolean run = true;

		
		while (run) {
			// get one frame
			frame_t finfo = new frame_t();
			int ret = dmcam.cap_get_frames(dev, 1, f, f.capacity(), finfo);
			if (ret > 0) {
                int img_w = (int)finfo.getFrame_info().getWidth();
                int img_h = (int)finfo.getFrame_info().getHeight();
				System.out.printf(" frame @ %dx%d, %d [",
						finfo.getFrame_info().getWidth(), 
						finfo.getFrame_info().getHeight(), 
                        finfo.getFrame_info().getFrame_idx());

				byte[] first_seg = new byte[16];
				f.get(first_seg, 0, 16);
				for (int n = 0; n < 16; n++) {
					System.out.printf("0x%02X, ", first_seg[n]);
				}
				System.out.printf("]\n");
				
                float[] dist = new float[img_w * img_h];

				//dmcam.raw2dist(dist, dist.length, f, f.capacity());
                dmcam.frame_get_distance(dev, dist, dist.length, f, f.capacity(), finfo.getFrame_info());
				for (int n = 0; n < 16; n++) {
					System.out.printf("%.3f, ", dist[n]);
				}
				System.out.printf("]\n");
				count += 1;
				if (count >= 100)
					break;
			}
			// print(" frame @ %d, %d, %d" %
			// (finfo.frame_fbpos, finfo.frame_count, finfo.frame_size))
			// # print the first 16bytes of the frame
			// # print([hex(n) for n in f][:16])
		}

		// try {
		// Thread.sleep(3000);
		// } catch (InterruptedException ex) {
		// Thread.currentThread().interrupt();
		// }
		System.out.println(" Stop capture ...");
		dmcam.cap_stop(dev);
		System.out.println(" Close dmcam device ..");
		dmcam.dev_close(dev);

		// System.out.println("hello world " + cnt);
		dmcam.uninit();
	}
}
