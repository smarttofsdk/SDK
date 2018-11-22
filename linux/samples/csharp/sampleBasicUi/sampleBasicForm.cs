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
        public SampleBasicForm()
        {
            InitializeComponent();

            // start dev thread
            var capTh = new Thread(CaptureThread) {IsBackground = true};
            capTh.Start();
        }

        private void Log(string msg)
        {
            if (!msg.EndsWith("\n"))
                msg += "\n";

            if (InvokeRequired)
                Invoke(new MethodInvoker(delegate { tbLog.AppendText(msg); }));
            else
                tbLog.AppendText(msg);
        }

        private void RefreshUi(Bitmap img_depth, Bitmap img_ir)
        {
            if (InvokeRequired)
                Invoke(new MethodInvoker(delegate { RefreshUi(img_depth, img_ir); }));
            else
            {
                pbDepth.Image = img_depth;
                pbIR.Image = img_ir;
            }
        }

        private void CaptureThread()
        {
            dmcam.init(null);
            dmcam.log_cfg(log_level_e.LOG_LEVEL_WARN, log_level_e.LOG_LEVEL_DEBUG,
                log_level_e.LOG_LEVEL_NONE);

            var devs = new dmcamDevArray(16);
            var cnt = dmcam.dev_list(devs.cast(), 16);

            Log(string.Format("found {0} device\n", cnt));

            if (cnt == 0)
                Invoke(new MethodInvoker(() =>
                {
                    MessageBox.Show(string.Format("found {0} device\n", cnt));
                    Application.Exit();
                }));

            /* open device */
            Log(" Open dmcam device ..");
            var dev = dmcam.dev_open(null);
            if (dev == null)
            {
                Log(" Open device failed");
                return;
            }

            cap_cfg_t cfg = new cap_cfg_t();
            cfg.cache_frames_cnt = 10;
            cfg.on_error = null;
            cfg.on_frame_ready = null;
            cfg.en_save_replay = 0;
            cfg.en_save_dist_u16 = 0;
            cfg.en_save_gray_u16 = 0;
            cfg.fname_replay = null;

            dmcam.cap_config_set(dev, cfg);
            //dmcam.cap_set_frame_buffer(dev, null, 10 * 320 * 240 * 4);
            Log(" Start capture ...");
            dmcam.cap_start(dev);

            var f = new byte[640 * 480 * 4 * 3];
            Log(" sampling 100 frames ...");
            var count = 0;
            var run = true;

            while (run)
            {
                // get one frame
                var finfo = new frame_t();
                var ret = dmcam.cap_get_frames(dev, 1, f, (uint) f.Length, finfo);
                if (ret > 0)
                {
                    var img_w = (int) finfo.frame_info.width;
                    var img_h = (int) finfo.frame_info.height;
                    Console.Write(" frame @ {0}x{1}, {2} [",
                        finfo.frame_info.width,
                        finfo.frame_info.height,
                        finfo.frame_info.frame_idx);

                    for (var n = 0; n < 16; n++) Console.Write("{0:X2}, ", f[n]);

                    Console.Write("]\n");

                    var dist = new ushort[img_w * img_h];
                    var gray = new ushort[img_w * img_h];

                    /* calc distance */
                    dmcam.frame_get_dist_u16(dev, dist, dist.Length, f, f.Length, finfo.frame_info);
                    dmcam.frame_get_gray_u16(dev, gray, gray.Length, f, f.Length, finfo.frame_info);

                    for (var n = 0; n < 16; n++) Console.Write("{0:F},", dist[n]);

                    /* convert depth to pseudo color image */
                    var dist_rgb = new byte[3 * img_w * img_h];
                    dmcam.cmap_dist_u16_to_RGB(dist_rgb, dist_rgb.Length, dist, dist.Length,
                        cmap_outfmt_e.DMCAM_CMAP_OUTFMT_BGR, 0, 5000);
                    /* convert gray to IR image */
                    var gray_u8 = new byte[img_w * img_h];
                    dmcam.cmap_gray_u16_to_IR(gray_u8, gray_u8.Length, gray, gray.Length, 0);

                    {
                        var img_depth = new Bitmap(img_w, img_h, PixelFormat.Format24bppRgb);
                        var img_ir = new Bitmap(img_w, img_h, PixelFormat.Format24bppRgb);

                        BitmapData bmData = img_depth.LockBits(new Rectangle(0, 0, img_depth.Width, img_depth.Height),
                            ImageLockMode.ReadWrite, img_depth.PixelFormat);
                        IntPtr pNative = bmData.Scan0;
                        Marshal.Copy(dist_rgb, 0, pNative, dist_rgb.Length);
                        img_depth.UnlockBits(bmData);

//                        bmData = img_ir.LockBits(new Rectangle(0, 0, img_ir.Width, img_ir.Height),
//                            ImageLockMode.ReadWrite, img_ir.PixelFormat);
//                        pNative = bmData.Scan0;
//                        Marshal.Copy(gray_u8, 0, pNative, gray_u8.Length);
//                        img_depth.UnlockBits(bmData);
                        var w = img_w;
                        var h = img_h;
                        for (int y = 0; y < h; y++)
                        {
                            for (int x = 0; x < w; x++)
                            {
                                int r = gray_u8[(y * w + x)] & 0xff;
                                int g = gray_u8[(y * w + x)] & 0xff;
                                int b = gray_u8[(y * w + x)] & 0xff;
                                img_ir.SetPixel(x, y, Color.FromArgb(0, r, g, b));
                            }
                        }

                        RefreshUi(img_depth, img_ir);
                    }

                    Console.Write("]\n");
                    count += 1;
                    if (count >= 100)
                        break;
                }
            }

            Log(" Stop capture ...");
            dmcam.cap_stop(dev);
            Log(" Close dmcam device ..");
            dmcam.dev_close(dev);

            dmcam.uninit();
        }
    }
}
