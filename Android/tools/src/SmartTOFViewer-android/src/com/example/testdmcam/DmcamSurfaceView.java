package com.example.testdmcam;

import java.nio.ByteBuffer;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.drawable.BitmapDrawable;
import android.util.Log;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceHolder.Callback;
import android.view.SurfaceView;

import com.smarttof.dmcam.cmap_outfmt_e;
import com.smarttof.dmcam.dev_t;
import com.smarttof.dmcam.dmcam;
import com.smarttof.dmcam.frame_t;

public class DmcamSurfaceView extends SurfaceView implements Runnable, Callback {
	private SurfaceHolder sufHolder; // 用于控制SurfaceView
	private Paint pen; // 声明一支画笔

	private Thread thRefresh = null; // 声明一条线程
	private boolean thRefreshStarted = false; // 线程运行的标识，用于控制线程

	private boolean startRefreshFrame = false;
	private boolean reqRefreshBg = false;
	private Bitmap imgBg; // background image when not start refresh

	private dev_t dev; // dmcam device
	private int devFrameCnt = 0;
	private ByteBuffer devFrameBuffer = ByteBuffer
			.allocateDirect(1024 * 768 * 4);

	public int paramViewType = 0;
	public float paramRangeMin = 0.0f;// 最小视距
	public float paramRangeMax = 5000.0f;// 最大视距
	public float alphagray = 10.0f;

	public DmcamSurfaceView(Context context) {
		super(context);

		Log.d("DMCAM", String.format("surface construct is called\n"));

		sufHolder = getHolder(); // 获得SurfaceHolder对象
		sufHolder.addCallback(this); // 为SurfaceView添加状态监听

		pen = new Paint(); // 创建一个画笔对象
		pen.setColor(Color.WHITE); // 设置画笔的颜色为白色
		pen.setStyle(Paint.Style.STROKE);
		setFocusable(true); // 设置焦点

		imgBg = ((BitmapDrawable) getResources().getDrawable(R.drawable.cover))
				.getBitmap();

		// create refresh thread
		if (!thRefreshStarted) {
			thRefresh = new Thread(this); // 创建一个线程对象
			thRefresh.setDaemon(true);
			thRefreshStarted = true; // 把线程运行的标识设置成true
			thRefresh.start(); // 启动线程

			Log.i("DMCAM", "start refresh thread\n");
		}
	}

	/** start refresh with specified dmcam device */
	public void start(dev_t dmcamDev) {
		dev = dmcamDev;
		startRefreshFrame = true;
	}

	/* stop refresh */
	public void stop() {
		startRefreshFrame = false;
		reqRefreshBg = true; // refresh to cover
		dev = null;
	}

	/**
	 * 当SurfaceView创建的时候，调用此函数: NOTE： some phone may call more than twice
	 */
	@Override
	public void surfaceCreated(SurfaceHolder holder) {
		Log.d("DMCAM", String.format("surfaceCreated is called: %d\n",
				reqRefreshBg ? 1 : 0));
		this.reqRefreshBg = true;
	}

	/**
	 * 当SurfaceView的视图发生改变的时候，调用此函数
	 */
	@Override
	public void surfaceChanged(SurfaceHolder holder, int format, int width,
			int height) {
		Log.d("DMCAM", String.format("surfaceChanged is called, wxh=%dx%d\n",
				width, height));
		this.reqRefreshBg = true;
	}

	/**
	 * 当SurfaceView销毁的时候，调用此函数
	 */
	@Override
	public void surfaceDestroyed(SurfaceHolder holder) {
		// th_started = false; // 把线程运行的标识设置成false
		// mHolder.removeCallback(this);
		Log.d("DMCAM", "surfaceDestroyed is called");
	}

	/**
	 * 当屏幕被触摸时调用
	 */
	@Override
	public boolean onTouchEvent(MotionEvent event) {

		return true;
	}

	/**
	 * 当用户按键时调用
	 */
	@Override
	public boolean onKeyDown(int keyCode, KeyEvent event) {
		if (keyCode == KeyEvent.KEYCODE_DPAD_UP) {
		}
		return super.onKeyDown(keyCode, event);
	}

	@Override
	public boolean onKeyUp(int keyCode, KeyEvent event) {
		surfaceDestroyed(sufHolder);
		return super.onKeyDown(keyCode, event);
	}

	@Override
	public void run() {
		Canvas canvas;
		while (thRefreshStarted) {
			try {
				if (!this.startRefreshFrame) {
					if (this.reqRefreshBg) {
						/* refresh background */
						this.reqRefreshBg = false;

						Log.d("DMCAM", String.format(
								"refresh background: cur_fr_cnt=%d\n",
								devFrameCnt));
						canvas = sufHolder.lockCanvas(); // 获得画布对象，开始对画布画画
						if (canvas != null) {
							// mCanvas.drawBitmap(pic, 0, 0, paint);
							Rect src = new Rect(0, 0, imgBg.getWidth() - 1,
									imgBg.getHeight() - 1);
							Rect dst = new Rect(0, 0, canvas.getWidth() - 1,
									canvas.getHeight() - 1);
							canvas.drawBitmap(imgBg, src, dst, null);
							// canvas.drawCircle(100, 100, 50, this.pen);
							sufHolder.unlockCanvasAndPost(canvas); // 完成画画，把画布显示在屏幕上
						}
					}
					Thread.sleep(500);
				} else {
					// get one frame
					frame_t finfo = new frame_t();
					int ret = dmcam.cap_get_frames(dev, 1, devFrameBuffer,
							devFrameBuffer.capacity(), finfo);
					if (ret < 0) {
						Log.e("DMCAM",
								String.format("capture failed: code=%d\n", ret));
						break;
					}
					if (ret == 0)
						continue;

					devFrameCnt++;

					int w = (int) finfo.getFrame_info().getWidth();
					int h = (int) finfo.getFrame_info().getHeight();
					float[] dist = new float[w * h];
					int dispImage[] = new int[dist.length];

					if (paramViewType == 0) {
						/* get distance in meter */
						dmcam.frame_get_distance(dev, dist, dist.length,
								devFrameBuffer, devFrameBuffer.capacity(),
								finfo.getFrame_info());
						float min_dist = paramRangeMin / 1000.0f;
						float max_dist = paramRangeMax / 1000.0f;

						/* cmap to pseudo color map */
						ByteBuffer dist_rgb = ByteBuffer.allocateDirect(3 * w
								* h);
						dmcam.cmap_dist_f32_to_RGB(dist_rgb, dist_rgb.capacity(), dist,
								dist.length,
								cmap_outfmt_e.DMCAM_CMAP_OUTFMT_RGB, min_dist,
								max_dist);
						for (int n = 0; n < dist.length; n++) {
							int red = dist_rgb.get(3 * n) & 0xFF;
							int green = dist_rgb.get(3 * n + 1) & 0xFF;
							int blue = dist_rgb.get(3 * n + 2) & 0xFF;
							dispImage[n] = Color.rgb(red, green, blue);
						}

					} else if (paramViewType == 1) {
						dmcam.frame_get_distance(dev, dist, dist.length,
								devFrameBuffer, devFrameBuffer.capacity(),
								finfo.getFrame_info());
						float min_dist = paramRangeMin / 1000.0f;
						float max_dist = paramRangeMax / 1000.0f;

						/* cmap to pseudo color map */
						ByteBuffer dist_rgb = ByteBuffer.allocateDirect(3 * w
								* h);
						dmcam.cmap_dist_f32_to_RGB(dist_rgb, dist_rgb.capacity(), dist,
								dist.length,
								cmap_outfmt_e.DMCAM_CMAP_OUTFMT_RGB, min_dist,
								max_dist);
						for (int n = 0; n < dist.length; n++) {
							int red = dist_rgb.get(3 * n) & 0xFF;
							int green = dist_rgb.get(3 * n + 1) & 0xFF;
							int blue = dist_rgb.get(3 * n + 2) & 0xFF;
							if (red > 100 && blue > 100)
								dispImage[n] = Color.rgb(255, 255, 255);
							else {
								int tt = (red * 5 + green * 30 + blue * 65) / 100;
								dispImage[n] = Color.rgb(tt, tt, tt);
							}
						}
					} else if (paramViewType == 2) {
						dmcam.frame_get_gray(dev, dist, dist.length,
								devFrameBuffer, devFrameBuffer.capacity(),
								finfo.getFrame_info());
						for (int n = 0; n < dist.length; n++) {
							dist[n] = (dist[n] / alphagray); // 降低曝光
							if (dist[n] >= 255.0f) // 过滤过曝光点
								dist[n] = 0.0f;
						}
						for (int n = 0; n < dist.length; n++) {
							int v = (int) dist[n] & 0xFF;
							dispImage[n] = Color.rgb(v, v, v);
						}
					} else {
						//
					}

					/* refresh canvas */
					canvas = sufHolder.lockCanvas();
					if (canvas != null) {
						Bitmap img = Bitmap.createBitmap(dispImage, w, h,
								Bitmap.Config.ARGB_8888);
						Rect src = new Rect(0, 0, img.getWidth() - 1,
								img.getHeight() - 1);
						Rect dst = new Rect(0, 0, canvas.getWidth() - 1,
								canvas.getHeight() - 1);
						canvas.drawBitmap(img, src, dst, null);
						Log.v("DMCAM", String.format("render... (fr_cnt=%d)\n",
								devFrameCnt));
						sufHolder.unlockCanvasAndPost(canvas);
					}
				}
			} catch (Exception e) {
				e.printStackTrace();
			} finally {

			}
		}
	}
	/**
	 * 自定义一个方法，在画布上画一个圆
	 */
	// protected void Draw() {
	// mCanvas = mHolder.lockCanvas(); // 获得画布对象，开始对画布画画
	// if (mCanvas != null) {
	// // Create bitmap with width, height, and 4 bytes color (RGBA)
	//
	// Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
	// paint.setColor(Color.BLUE);
	// paint.setStrokeWidth(10);
	// paint.setStyle(Style.FILL);
	// if (m_circle_r >= (getWidth() / 10)) {
	// m_circle_r = 0;
	// } else {
	// m_circle_r++;
	// }
	// Bitmap pic = ((BitmapDrawable) getResources().getDrawable(
	// R.drawable.ic_launcher)).getBitmap();
	// mCanvas.drawBitmap(pic, 0, 0, paint);
	// for (int i = 0; i < 5; i++)
	// for (int j = 0; j < 8; j++)
	// mCanvas.drawCircle(
	// (getWidth() / 5) * i + (getWidth() / 10),
	// (getHeight() / 8) * j + (getHeight() / 16),
	// m_circle_r, paint);
	// mHolder.unlockCanvasAndPost(mCanvas); // 完成画画，把画布显示在屏幕上
	// }
	// }
}
