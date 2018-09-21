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

        private void RefreshUi(Bitmap img)
        {
            if (InvokeRequired)
                Invoke(new MethodInvoker(delegate { RefreshUi(img); }));
            else
                pbPreview.Image = img;
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

            dmcam.cap_set_frame_buffer(dev, null, 10 * 320 * 240 * 4);
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

                    var dist = new float[img_w * img_h];

                    /* calc distance */
                    dmcam.frame_get_distance(dev, dist, dist.Length, f, f.Length, finfo.frame_info);
                    for (var n = 0; n < 16; n++) Console.Write("{0:F},", dist[n]);

                    /* convert to pseudo color image */
                    var dist_rgb = new byte[3 * img_w * img_h];
                    dmcam.cmap_float(dist_rgb, dist_rgb.Length, dist, dist.Length, cmap_outfmt_e.DMCAM_CMAP_OUTFMT_BGR,
                        0f, 5.0f);
                    {
                        var w = img_w;
                        var h = img_h;

//                        var img = new Bitmap(w, h, 3 * w, PixelFormat.Format24bppRgb,
//                            Marshal.UnsafeAddrOfPinnedArrayElement(dist_rgb, 0));

                        var img = new Bitmap(w, h, PixelFormat.Format24bppRgb);
                        BitmapData bmData = img.LockBits(new Rectangle(0, 0, img.Width, img.Height), ImageLockMode.ReadWrite, img.PixelFormat);
                        IntPtr pNative = bmData.Scan0;
                        Marshal.Copy(dist_rgb, 0, pNative, dist_rgb.Length);
                        img.UnlockBits(bmData);
//                        for (int y = 0; y < h; y++)
//                        {
//                            for (int x = 0; x < w; x++)
//                            {
//                                int r = dist_rgb[(y*w+x) * 3] & 0xff;
//                                int g = dist_rgb[(y*w+x) * 3 + 1] & 0xff;
//                                int b = dist_rgb[(y*w+x) * 3 + 2] & 0xff;
//                                img.SetPixel(x, y, Color.FromArgb(0, r, g, b));
//                            }
//                        }

                        RefreshUi(img);
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