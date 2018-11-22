using com.smarttof;
using System;

namespace sampleBasic {
    public class sampleBasic{
        public static void Main(string[] argv) {
            dmcam.init(null);
            dmcam.log_cfg(log_level_e.LOG_LEVEL_WARN, log_level_e.LOG_LEVEL_DEBUG,
                    log_level_e.LOG_LEVEL_NONE);
            dmcamDevArray devs = new dmcamDevArray(16);
            int cnt = dmcam.dev_list(devs.cast(), 16);

            Console.Write("found {0} device\n", cnt);

            if (cnt == 0) {
                return;
            }
            Console.WriteLine(" Open dmcam device ..");
            dev_t dev = dmcam.dev_open(null);
            if (dev == null) {
                Console.WriteLine(" Open device failed");
                return;
            }

            //int pwr_percent = 100;
            //param_item_t wparam = new param_item_t();
            //dmcamParamArray wparams = new dmcamParamArray(1);

            //wparam.setParam_id(dev_param_e.PARAM_ILLUM_POWER);
            //wparam.getParam_val().getIllum_power().setPercent((short) pwr_percent);

            //wparams.setitem(0, wparam);
            //Console.Write(" pwr = %d\n", wparams.getitem(0).getParam_val()
                    //.getIllum_power().getPercent());
            //if (!dmcam.param_batch_set(dev, wparams.cast(), 1)) {
                //Console.Write(" set illu_power to %d %% failed\n", pwr_percent);
            //}
            cap_cfg_t cfg = new cap_cfg_t();
            cfg.cache_frames_cnt = 10;
            cfg.on_error= null;
            cfg.on_frame_ready= null;
            cfg.en_save_replay= 0;
            cfg.en_save_dist_u16= 0;
            cfg.en_save_gray_u16= 0;
            cfg.fname_replay= null;

            dmcam.cap_config_set(dev, cfg);
            //dmcam.cap_set_frame_buffer(dev, null, 10 * 320 * 240 * 4);
            // dmcam.cap_set_callback_on_error(dev, null);
            Console.WriteLine(" Start capture ...");
            dmcam.cap_start(dev);

            byte[] f = new byte[640 * 480 * 4 * 3];
            //ByteBuffer f = ByteBuffer.allocateDirect(1920 * 1080 * 3); [> should be large enough to have one frame <]
            Console.WriteLine(" sampling 100 frames ...");
            int count = 0;
            bool run = true;

            while (run) {
                // get one frame
                frame_t finfo = new frame_t();
                int ret = dmcam.cap_get_frames(dev, 1, f, (uint)f.Length, finfo);
                if (ret > 0) {
                    int img_w = (int)finfo.frame_info.width;
                    int img_h = (int)finfo.frame_info.height;
                    Console.Write(" frame @ {0}x{1}, {2} [",
                            finfo.frame_info.width, 
                            finfo.frame_info.height, 
                            finfo.frame_info.frame_idx);

                    //byte[] first_seg = new byte[16];
                    //f.get(first_seg, 0, 16);
                    for (int n = 0; n < 16; n++) {
                        Console.Write("{0:X2}, ", f[n]);
                    }
                    Console.Write("]\n");

                    ushort[] dist = new ushort[img_w * img_h];

                    //dmcam.raw2dist(dist, dist.length, f, f.capacity());
                    dmcam.frame_get_dist_u16(dev, dist, dist.Length, f, f.Length, finfo.frame_info);
                    for (int n = 0; n < 16; n++) {
                        Console.Write("{0},", dist[n]);
                    }
                    Console.Write("]\n");
                    count += 1;
                    if (count >= 30)
                        break;
                }
                // print(" frame @ %d, %d, %d" %
                // (finfo.frame_fbpos, finfo.frame_count, finfo.frame_size))
                // # print the first 16bytes of the frame
                // # print([hex(n) for n in f][:16])
            }

            Console.WriteLine(" Stop capture ...");
            dmcam.cap_stop(dev);
            Console.WriteLine(" Close dmcam device ..");
            dmcam.dev_close(dev);

            // Console.WriteLine("hello world " + cnt);
            dmcam.uninit();
        }
    }
}
