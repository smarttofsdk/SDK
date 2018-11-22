package com.smarttof.dmcam.sample;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import javax.swing.JFrame;
import javax.swing.JPanel;

import com.smarttof.dmcam.*;

public class sampleBasicUi {
	public static class ResizableImagePanel extends JPanel {
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;
		private Image img;

		public ResizableImagePanel() {
		}

		public void setImage(Image value) {
			if (img != value) {
				Image old = img;
				this.img = value;
				firePropertyChange("image", old, img);
				revalidate();
				repaint();
			}
		}

		public Image getImage() {
			return img;
		}

		@Override
		public Dimension getPreferredSize() {
			return img == null ? new Dimension(320, 240) : new Dimension(
					img.getWidth(this), img.getHeight(this));
		}

		@Override
		protected void paintComponent(Graphics g) {
			super.paintComponent(g);
			if (img != null) {
				Graphics2D g2d = (Graphics2D) g.create();

				int width = getWidth();
				int height = getHeight();

				double scaleFactor = getScaleFactorToFit(
						new Dimension(img.getWidth(this), img.getHeight(this)),
						getSize());

				int x = (int) ((width - (img.getWidth(this) * scaleFactor)) / 2);
				int y = (int) ((height - (img.getHeight(this) * scaleFactor)) / 2);

				AffineTransform at = new AffineTransform();
				at.translate(x, y);
				at.scale(scaleFactor, scaleFactor);
				g2d.setTransform(at);
				g2d.drawImage(img, 0, 0, this);
				g2d.dispose();
			}
		}

		public double getScaleFactor(int iMasterSize, int iTargetSize) {

			return (double) iTargetSize / (double) iMasterSize;

		}

		public double getScaleFactorToFit(Dimension original, Dimension toFit) {

			double dScale = 1d;

			if (original != null && toFit != null) {

				double dScaleWidth = getScaleFactor(original.width, toFit.width);
				double dScaleHeight = getScaleFactor(original.height,
						toFit.height);

				dScale = Math.min(dScaleHeight, dScaleWidth);

			}

			return dScale;

		}
	}

	private static JFrame frame = new JFrame("Testing");
	private static ResizableImagePanel panel = new ResizableImagePanel();

	private static void guiInit() {
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.add(panel);
		frame.pack();
		frame.setLocationRelativeTo(null);
		frame.setVisible(true);
	}
	private static void guiExit()
	{
		frame.dispose();
	}

	// public static void showImage(Mat img) {
	// // Imgproc.resize(img, img, new Size(640, 480));
	// MatOfByte matOfByte = new MatOfByte();
	// Highgui.imencode(".png", img, matOfByte);
	// BufferedImage bufImage = null;
	// try {
	// bufImage = ImageIO.read(new ByteArrayInputStream(matOfByte
	// .toArray()));
	// panel.setImage(bufImage);
	// } catch (Exception e) {
	// e.printStackTrace();
	// }
	// }
	public static void showImage(Image img) {
		panel.setImage(img);
	}

	public static void main(String[] args) {
		// System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		// Mat img = Highgui.imread("test.png");
		// showImage(img);
		// return;
		dmcam.init(null);
		dmcam.log_cfg(log_level_e.LOG_LEVEL_NONE, log_level_e.LOG_LEVEL_DEBUG,
				log_level_e.LOG_LEVEL_NONE);
		dmcamDevArray devs = new dmcamDevArray(16);
		int cnt = dmcam.dev_list(devs.cast(), 16);

		System.out.printf("found %d device\n", cnt);
		//for (int i = 0; i < cnt; i++) {
			//System.out
					//.printf("DMCAM#%d [%03d:%03d:%03d]: VENDOR=%s, PROD=%s, SERIAL=%s\n",
							//i, devs.getitem(i).getUsb_port_num(),
							//devs.getitem(i).getUsb_bus_num(), devs.getitem(i)
									//.getUsb_dev_addr(), devs.getitem(i)
									//.getVendor(), devs.getitem(i).getProduct(),
							//devs.getitem(i).getSerial());
		//}
		if (cnt == 0) {
			return;
		}
		guiInit();

		System.out.println(" Open dmcam device ..");
		dev_t dev = dmcam.dev_open(null);
		if (dev == null) {
			System.out.println(" Open device failed");
			guiExit();
			return;
		}

		// write illumination power: 100%
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
        cap_cfg_t cfg = new cap_cfg_t();
        cfg.setCache_frames_cnt(10);
        cfg.setOn_error(null);
        cfg.setOn_frame_ready(null);
        cfg.setEn_save_replay((short)0);
        cfg.setEn_save_dist_u16((short)0);
        cfg.setEn_save_gray_u16((short)0);
        cfg.setFname_replay(null);

        dmcam.cap_config_set(dev, cfg);
		//dmcam.cap_set_frame_buffer(dev, null, 10 * 320 * 240 * 4);
		// dmcam.cap_set_callback_on_error(dev, null);
		System.out.println(" Start capture ...");
		dmcam.cap_start(dev);

		ByteBuffer f = ByteBuffer.allocateDirect(1920 * 1080 * 3); /* should be large enough to have one frame */
		System.out.println(" sampling 100 frames ...");
		int count = 0;
        boolean run = true;

        /* enable filter */
        filter_args_u amp_min_val = new filter_args_u();
        amp_min_val.setMin_amp(15);
        dmcam.filter_enable(dev, filter_id_e.DMCAM_FILTER_ID_AMP, amp_min_val, 4);

        /* disable pix calib */
        dmcam.filter_disable(dev, filter_id_e.DMCAM_FILTER_ID_PIXEL_CALIB);

        while (run) {
			// get one frame
			frame_t finfo = new frame_t();
			int ret = dmcam.cap_get_frames(dev, 1, f, f.capacity(), finfo);
			if (ret > 0) {
                int img_w = (int)finfo.getFrame_info().getWidth();
                int img_h = (int)finfo.getFrame_info().getHeight();
				//System.out.printf(" frame @ %dx%d, %d [",
						//finfo.getFrame_info().getWidth(), 
						//finfo.getFrame_info().getHeight(), 
                        //finfo.getFrame_info().getFrame_idx());

//				byte[] first_seg = new byte[16];
//				f.get(first_seg, 0, 16);
//				for (int n = 0; n < 16; n++) {
//					System.out.printf("0x%02X, ", first_seg[n]);
//				}
//				System.out.printf("]\n");
				if (img_w > 1920 || img_h > 1080 ) {
    				System.out.printf("Wrong frame info: %dx%d!\n", img_w, img_h);
                    continue;
                }

                //float[] dist = new float[img_w * img_h];
                int [] dist = new int[img_w * img_h];
                byte[] dist8U = new byte[img_w * img_h];

                dmcam.frame_get_dist_u16(dev, dist, dist.length, f, f.capacity(), finfo.getFrame_info());

                /* ---- visualize ---- */
                ByteBuffer dist_rgb = ByteBuffer.allocateDirect(3 * img_w * img_h);
                dmcam.cmap_dist_u16_to_RGB(dist_rgb, dist_rgb.capacity(), dist, dist.length, cmap_outfmt_e.DMCAM_CMAP_OUTFMT_RGB, 0, 5000);
                {
                    int w = img_w;
                    int h = img_h;
                    BufferedImage img = new BufferedImage(w, h,
                            BufferedImage.TYPE_3BYTE_BGR);
                    for (int r = 0; r < h; r++)
                        for (int c = 0; c < w; c++) {
                            int index = r * w + c;
                            int red = dist_rgb.get(3*index) & 0xFF;
                            int green = dist_rgb.get(3*index+1) & 0xFF;
                            int blue = dist_rgb.get(3*index+2) & 0xFF;
                            int rgb = (red << 16) | (green << 8) | blue;
                            img.setRGB(c, r, rgb);
                        }
                    showImage(img);
                }
                count += 1;
                if (count % 10 == 0) 
                    System.out.printf("  frm_count=%d\n", count);
                if (count >= 30)
                    break;
            }
        }
        System.out.println(" Stop capture ...");
        dmcam.cap_stop(dev);

        guiExit();

        System.out.println(" Close dmcam device ..");
        dmcam.dev_close(dev);
        // System.out.println("hello world " + cnt);
        dmcam.uninit();
    }
}
