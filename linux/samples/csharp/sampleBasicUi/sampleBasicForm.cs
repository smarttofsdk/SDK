using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;
using com.smarttof;

namespace sampleBasicUi
{
    public partial class SampleBasicForm : Form
    {
        private readonly bool _testMode;
        public SampleBasicForm(int testMode = 0)
        {
            _testMode = testMode == 1;
            InitializeComponent();

            if (_testMode)
            {
                Text += @"[TEST MODE]";
            }
            // start dev thread
            var capTh = new Thread(CaptureThread) { IsBackground = true };
            capTh.Start();
        }

        public sealed override string Text
        {
            get { return base.Text; }
            set { base.Text = value; }
        }

        private void Log(string msg)
        {
            if (!msg.EndsWith("\n"))
                msg += "\r\n";

            if (InvokeRequired)
                Invoke(new MethodInvoker(delegate { tbLog.AppendText(msg); }));
            else
            {
                tbLog.AppendText(msg);
            }
            Console.Write(msg);
        }

        private void RefreshUi(Bitmap imgDepth, Bitmap imgIR)
        {
            if (InvokeRequired)
                Invoke(new MethodInvoker(delegate { RefreshUi(imgDepth, imgIR); }));
            else
            {
                pbDepth.Image = imgDepth;
                pbIR.Image = imgIR;
            }
        }

        private void CaptureThread()
        {
            int n_frames = 60;
            int fps = 25;
            string result = "NG";

            dmcam.init(null);
            dmcam.log_cfg(log_level_e.LOG_LEVEL_WARN, log_level_e.LOG_LEVEL_DEBUG,
                log_level_e.LOG_LEVEL_NONE);

            var devs = new dmcamDevArray(16);
            var cnt = dmcam.dev_list(devs.cast(), 16);

            Log(string.Format("found {0} device", cnt));

            if (cnt == 0)
            {
                goto FINAL;
            }

            /* open device */
            Log(" Open dmcam device ..");
            var dev = dmcam.dev_open(null);
            if (dev == null)
            {
                Log(" Open device failed");
                goto FINAL;
            }

            Log(string.Format(" Set fps to {0} ..", fps));
            {
                param_item_t pFps = new param_item_t {param_id = dev_param_e.PARAM_FRAME_RATE};
                pFps.param_val.frame_rate.fps = (uint)fps;
                
                dmcamParamArray param = new dmcamParamArray(1);
                param.setitem(0, pFps);

                if (!dmcam.param_batch_set(dev, param.cast(), 1))
                {
                    Log("set fps failed");
                    goto FINAL;
                }

                param.setitem(0, new param_item_t { param_id = dev_param_e.PARAM_FRAME_RATE });

                if (!dmcam.param_batch_get(dev, param.cast(), 1))
                {
                    Log("get fps failed");
                    goto FINAL;
                }

                Log(string.Format(" Get fps : {0} ..", param.getitem(0).param_val.frame_rate.fps));
            }

            cap_cfg_t cfg = new cap_cfg_t
            {
                cache_frames_cnt = 10,
                on_error = null,
                on_frame_ready = null,
                en_save_replay = 0,
                en_save_dist_u16 = 0,
                en_save_gray_u16 = 0,
                fname_replay = null
            };

            dmcam.cap_config_set(dev, cfg);
            //dmcam.cap_set_frame_buffer(dev, null, 10 * 320 * 240 * 4);
            Log(" Start capture ...");
            dmcam.cap_start(dev);

            var f = new byte[640 * 480 * 4 * 3];
            Log(string.Format(" sampling {0} frames ...", n_frames));
            var count = 0;

            while (true)
            {
                // get one frame
                var finfo = new frame_t();
                var ret = dmcam.cap_get_frames(dev, 1, f, (uint) f.Length, finfo);
                if (ret > 0)
                {
                    var imgW = (int) finfo.frame_info.width;
                    var imgH = (int) finfo.frame_info.height;
                    Console.Write(@" frame @ {0}x{1}, {2} [",
                        finfo.frame_info.width,
                        finfo.frame_info.height,
                        finfo.frame_info.frame_idx);

                    for (var n = 0; n < 16; n++) Console.Write(@"{0:X2}, ", f[n]);

                    Console.WriteLine(@"]");

                    var dist = new ushort[imgW * imgH];
                    var gray = new ushort[imgW * imgH];

                    /* calc distance */
                    dmcam.frame_get_dist_u16(dev, dist, dist.Length, f, f.Length, finfo.frame_info);
                    dmcam.frame_get_gray_u16(dev, gray, gray.Length, f, f.Length, finfo.frame_info);

                    for (var n = 0; n < 16; n++) Console.Write(@"{0:F},", dist[n]);

                    /* convert depth to pseudo color image */
                    var distRgb = new byte[3 * imgW * imgH];
                    dmcam.cmap_dist_u16_to_RGB(distRgb, distRgb.Length, dist, dist.Length,
                        cmap_outfmt_e.DMCAM_CMAP_OUTFMT_BGR, 0, 5000, null);
                    /* convert gray to IR image */
                    var grayU8 = new byte[imgW * imgH];
                    dmcam.cmap_gray_u16_to_IR(grayU8, grayU8.Length, gray, gray.Length, 0);

                    {
                        var imgDepth = new Bitmap(imgW, imgH, PixelFormat.Format24bppRgb);
                        var imgIR = new Bitmap(imgW, imgH, PixelFormat.Format24bppRgb);

                        BitmapData bmData = imgDepth.LockBits(new Rectangle(0, 0, imgDepth.Width, imgDepth.Height),
                            ImageLockMode.ReadWrite, imgDepth.PixelFormat);
                        IntPtr pNative = bmData.Scan0;
                        Marshal.Copy(distRgb, 0, pNative, distRgb.Length);
                        imgDepth.UnlockBits(bmData);

//                        bmData = img_ir.LockBits(new Rectangle(0, 0, img_ir.Width, img_ir.Height),
//                            ImageLockMode.ReadWrite, img_ir.PixelFormat);
//                        pNative = bmData.Scan0;
//                        Marshal.Copy(gray_u8, 0, pNative, gray_u8.Length);
//                        img_depth.UnlockBits(bmData);
                        var w = imgW;
                        var h = imgH;
                        for (int y = 0; y < h; y++)
                        {
                            for (int x = 0; x < w; x++)
                            {
                                int r = grayU8[(y * w + x)] & 0xff;
                                int g = grayU8[(y * w + x)] & 0xff;
                                int b = grayU8[(y * w + x)] & 0xff;
                                imgIR.SetPixel(x, y, Color.FromArgb(0, r, g, b));
                            }
                        }

                        RefreshUi(imgDepth, imgIR);
                    }

                    Console.WriteLine(@"]");
                    count += 1;
                    if (count >= n_frames)
                        break;
                }
            }

            Log(" Stop capture ...");
            dmcam.cap_stop(dev);
            Log(" Close dmcam device ..");
            dmcam.dev_close(dev);

            dmcam.uninit();
            result = "OK";
FINAL:
            if (_testMode)
            {
                Log(" Loop test " + result);
                Application.Exit();
            }
        }
    }
}
