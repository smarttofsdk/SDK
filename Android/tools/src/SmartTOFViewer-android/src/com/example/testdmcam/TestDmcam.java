package com.example.testdmcam;

import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.Gravity;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.FrameLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import com.smarttof.dmcam.dev_param_e;
import com.smarttof.dmcam.dev_t;
import com.smarttof.dmcam.dmcam;
import com.smarttof.dmcam.dmcamParamArray;
import com.smarttof.dmcam.filter_args_u;
import com.smarttof.dmcam.filter_id_e;
import com.smarttof.dmcam.log_level_e;
import com.smarttof.dmcam.param_item_t;

import com.smarttof.dmcam.cap_cfg_t;
//import com.smarttof.dmcam.*;

public class TestDmcam extends Activity implements OnClickListener {
	public static TestDmcam td;
	// private Context context;
	private FrameLayout layoutDmcamPreview;

	@SuppressWarnings("unused")
	private Button btnCapture, btnExit;

	
	private TextView tvBase, tvAdvance;
	private TextView tvLog;
	private CheckBox checkBox;
	private ScrollView svLog;
	private UsbManager usbManager;
	private UsbDevice usbDevice;
	private static final String ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION";
	private PendingIntent pendingIntent;
	static int fd = -1;
	static int dev_minamp, dev_exp, dev_corr;
	static long dev_freq, dev_frame;
	private boolean devCapturing = false;
	private boolean bscSettingDialogIsHide = false;
	Dialog1 bscSettingDialog = null;
	Dialog2 myDialog2 = null;
	private static DmcamSurfaceView dmcamView;
	private static dev_t dev = null; // current dmcam dev handler
	private Handler handler = new Handler() {
		@SuppressLint("ShowToast")
		@Override
		public void handleMessage(Message msg) {
			super.handleMessage(msg);
			switch (msg.what) {
			case 3:
				// Toast.makeText(TestDmcam.this, "33333333333333333",
				// 0).show();
				break;
			case 4:
				// Toast.makeText(TestDmcam.this, "44444444444444444",
				// 0).show();
				break;
			}
		}
	};

	@Override
	protected void onPause() {
		super.onPause();
		Log.d("DMCAM", "onPause...\n");
		if (devCapturing && dev != null) {
			// Thread th = new ThreadStartCapture();
			// th.start();
			// try {
			// th.join(1000);
			// } catch (InterruptedException e) {
			// e.printStackTrace();
			// }
			Log.i("DMCAM", "dev cap stop...\n");
			dmcamView.stop();
			dmcam.cap_stop(dev);
		}
	}

	@Override
	protected void onResume() {
		super.onResume();
		Log.d("DMCAM", "onResume...\n");
		// dmcamView = new DmcamSurfaceView(this); //
		// 在恢复后产生一个新的surfaceview，因为锁屏会将原来的surfaceview销毁
		// layoutDmcamPreview.addView(dmcamView);
		if (devCapturing && dev != null) {
			Log.i("DMCAM", "dev cap start...\n");
			devCapturing = dmcam.cap_start(dev);
			dmcamView.start(dev);
		}
	}

	@Override
	protected void onDestroy() {
		Log.d("DMCAM", "onDestroy...\n");
		unregisterReceiver(mUsbReceiver);
		if (bscSettingDialogIsHide)
			bscSettingDialog.dismiss();
		// if(d2) myDialog2.dismiss();
		super.onDestroy();

	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		Log.d("DMCAM", "onCreate...\n");
		setContentView(R.layout.activity_testdmcam);
		td = this;//
		usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
		pendingIntent = PendingIntent.getBroadcast(this, 0, new Intent(
				ACTION_USB_PERMISSION), 0);
		IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
		registerReceiver(mUsbReceiver, filter);

		btnCapture = (Button) findViewById(R.id.btnCapture);
		btnExit = (Button) findViewById(R.id.btnExit);
		tvBase = (TextView) findViewById(R.id.tv_base2);
		tvAdvance = (TextView) findViewById(R.id.tv_advanced2);
		tvLog = (TextView) findViewById(R.id.tvLog);
		svLog = (ScrollView) findViewById(R.id.svLog);
		layoutDmcamPreview = (FrameLayout) findViewById(R.id.layoutDmcamPreview);
		checkBox = (CheckBox) findViewById(R.id.ckb);

		// 按下基础参数文本触发
		tvBase.setOnClickListener(this);
		tvAdvance.setOnClickListener(this);
		tvLog.setText("");
		tvLog.setMovementMethod(new ScrollingMovementMethod());
		dmcamView = new DmcamSurfaceView(this);
		layoutDmcamPreview.addView(dmcamView);

		dmcam.log_cfg(log_level_e.LOG_LEVEL_INFO, log_level_e.LOG_LEVEL_DEBUG,
				log_level_e.LOG_LEVEL_NONE);

		checkBox.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
			@Override
			public void onCheckedChanged(CompoundButton compoundButton,
					boolean b) {
				if (b) {
					svLog.setVisibility(View.VISIBLE);
				} else {
					svLog.setVisibility(View.INVISIBLE);
				}
			}
		});
	}

	public static DmcamSurfaceView getDmcamView() {
		return dmcamView;
	}

	public static dev_t getDev() {
		return dev;
	}

	private void logUI(String tag, String msg) {
		Log.i(tag, msg);

		if (!msg.endsWith("\n")) {
			msg += "\n";
		}
		final String _msg = msg;
		runOnUiThread(new Runnable() {
			public void run() {
				tvLog.append(_msg);
				svLog.scrollTo(0, svLog.getBottom());
			}
		});
	}

	public void onClick_Capture(View view) {
		// Toast.makeText(getApplicationContext(), "start test",
		// Toast.LENGTH_LONG)
		// .show();
		logUI("test", "capture button clicked\n");

		/* disable button */
		btnCapture.setEnabled(false);

		new ThreadStartCapture().start();

	}

	// @Override
	// protected void onNewIntent(Intent intent) {
	// super.onNewIntent(intent);
	// if (intent != null) {
	// boolean isExitApp = intent.getBooleanExtra("exit", false);
	// if (isExitApp) {
	// this.finish();
	// }
	// }
	// }

	// private void exitAPP2() {
	// BaseApplication.showList();
	// BaseApplication.exitAppList();
	// System.exit(0);
	// }
	//
	// private void exitAPP3() {
	// Intent intent = new Intent(context, TestDmcam.class);
	// intent.putExtra("exit", true);
	// context.startActivity(intent);
	// System.exit(0);
	// }

	public void onClick_Exit(View view) {
		if (devCapturing) {
			// btnCapture.setText("开始采集");
			// logUI("DMCAM", "Stop capture ...");
			dmcam.cap_stop(dev);
			// logUI("DMCAM", "Close dmcam device ..");
			dmcam.dev_close(dev);
			dev = null;
			devCapturing = false;
		}

		// Log.d(BaseApplication.getTAG(),
		// "按下Close———————————————————————————— ");
		// exitAPP1();
		// exitAPP2();
		// exitAPP3();
		// exitAPP4();
		this.finish();
		System.exit(0);
	}

	public boolean devSetFrameRate(dev_t dev, int fps) {
		if (dev == null)
			return false;

		param_item_t wparam = new param_item_t();
		dmcamParamArray wparams = new dmcamParamArray(1);

		wparam.setParam_id(dev_param_e.PARAM_FRAME_RATE);
		wparam.getParam_val().getFrame_rate().setFps(fps);

		wparams.setitem(0, wparam);
		logUI("DMCAM",
				String.format("set fps = %d\n", wparams.getitem(0)
						.getParam_val().getFrame_rate().getFps()));
		if (!dmcam.param_batch_set(dev, wparams.cast(), 1)) {
			logUI("DMCAM", String.format(" set fps to %d failed\n", fps));
			return false;
		}
		return true;
	}

	public boolean devSetExposure(dev_t dev, int expoUs) {
		if (dev == null)
			return false;

		param_item_t wparam = new param_item_t();
		dmcamParamArray wparams = new dmcamParamArray(1);

		wparam.setParam_id(dev_param_e.PARAM_INTG_TIME);
		wparam.getParam_val().getIntg().setIntg_us(expoUs);

		wparams.setitem(0, wparam);
		logUI("DMCAM",
				String.format("set exposure = %d us\n", wparams.getitem(0)
						.getParam_val().getIntg().getIntg_us()));
		if (!dmcam.param_batch_set(dev, wparams.cast(), 1)) {
			logUI("DMCAM",
					String.format(" set exposure to %d us failed\n", expoUs));
			return false;
		}
		return true;
	}

	public boolean devSetMinAmp(dev_t dev, int amp) {
		if (dev == null)
			return false;

		filter_args_u amp_min_val = new filter_args_u();
		amp_min_val.setMin_amp(amp);
		logUI("DMCAM",
				String.format("set minimal amplitude = %d\n",
						amp_min_val.getMin_amp()));
		if (dmcam.filter_enable(dev, filter_id_e.DMCAM_FILTER_ID_AMP,
				amp_min_val, 4) == 0) {
			logUI("DMCAM",
					String.format("set minimal amplitude to %d failed\n", amp));
			return false;
		}
		return true;
	}

	public boolean devSetAutoExpo(dev_t dev) {
		if (dev == null)
			return false;

		filter_args_u auto_expo = new filter_args_u();
		auto_expo.setSat_ratio(5);
		logUI("DMCAM",
				String.format("Auto Exposure. Sat ratio = %d\n",
						auto_expo.getSat_ratio()));
		dmcam.filter_enable(dev, filter_id_e.DMCAM_FILTER_ID_AUTO_INTG,
				auto_expo, 2);
		return true;
	}

	public boolean devUndoAutoExpo(dev_t dev) {
		logUI("DMCAM", String.format("Cancel Auto Exposure.\n"));
		dmcam.filter_disable(dev, filter_id_e.DMCAM_FILTER_ID_AUTO_INTG);
		return true;
	}

	@Override
	public void onClick(View view) {
		switch (view.getId()) {
		case R.id.tv_base2:
			if (bscSettingDialogIsHide) {
				bscSettingDialog.show();
			} else {
				bscSettingDialog = new Dialog1(this, R.style.myDialogTheme);
				Window dialogWindow = bscSettingDialog.getWindow();
				WindowManager.LayoutParams lp = dialogWindow.getAttributes();
				lp.width = WindowManager.LayoutParams.MATCH_PARENT;
				lp.height = WindowManager.LayoutParams.WRAP_CONTENT;
				dialogWindow.setGravity(Gravity.CENTER_HORIZONTAL
						| Gravity.BOTTOM);
				dialogWindow.setAttributes(lp);
				bscSettingDialog.show();
				bscSettingDialogIsHide = true;
			}
			break;
		// case R.id.tv_advanced2:
		// if(d2) {
		// myDialog2.show();
		// }
		// else {
		// myDialog2 = new Dialog2(this,R.style.myDialogTheme);
		// Window dialogWindow2 = myDialog2.getWindow();
		// WindowManager.LayoutParams lp2 = dialogWindow2.getAttributes();
		// lp2.width = WindowManager.LayoutParams.MATCH_PARENT;
		// lp2.height = WindowManager.LayoutParams.WRAP_CONTENT;
		// dialogWindow2.setGravity(Gravity.CENTER_HORIZONTAL|Gravity.BOTTOM);
		// dialogWindow2.setAttributes(lp2);
		// myDialog2.show();
		// d2 = true;
		// }
		// break;
		}
	}

	class ThreadStartCapture extends Thread {
		@Override
		public void run() {
			super.run();
			try {
				if (devCapturing && dev != null) {
					dmcamView.stop();

					logUI("DMCAM", "Stop capture ...");
					dmcam.cap_stop(dev);
					logUI("DMCAM", "Close dmcam device ..");
					dmcam.dev_close(dev);
					dev = null;
					devCapturing = false;
					bscSettingDialogIsHide = false;

					runOnUiThread(new Runnable() {
						public void run() {
							btnCapture.setText("开始采集");
							btnCapture.setEnabled(true);
						}
					});
					return;
				}
				devCapturing = true;
				UsbDevice ud = null;
				// 链接到设备的USB设备列表
				HashMap<String, UsbDevice> map = usbManager.getDeviceList();

				Collection<UsbDevice> usbDevices = map.values();
				Iterator<UsbDevice> usbDeviceIter = usbDevices.iterator();
				while (usbDeviceIter.hasNext()) {
					ud = usbDeviceIter.next();
					// VendorID 和ProductID 十进制
					if (0x111B == ud.getVendorId()
							&& 0x1238 == ud.getProductId()) {
						usbDevice = ud;
						logUI("device", "find dmcam device"); // bhw
					}
					logUI("device", String.format(
							"find usb device: vid=0x%X, pid=0x%X, id=0x%x",
							ud.getVendorId(), ud.getProductId(),
							ud.getDeviceId()));
				}

				// 检测USB设备权限受否授权
				if (usbManager.hasPermission(usbDevice)) {
					handler.sendEmptyMessage(3);

					logUI("device", "usbManager.hasPermission");// bhw
					new MyThread3().start();
				} else {
					// 如果没有授权就授予权限
					handler.sendEmptyMessage(4);
					usbManager.requestPermission(usbDevice, pendingIntent); // 在此Android系统会弹出对话框，选择是否授予权限
					logUI("device", "usbManager.requestPermission"); // bhw
				}
			} catch (Exception e) {
				logUI("device", "no dmcam device found");
				runOnUiThread(new Runnable() {
					public void run() {
						btnCapture.setEnabled(true);
					}
				});
				e.printStackTrace();
			} finally {

			}
		}
	}

	public static int openUsbDevice(int vid, int pid)
			throws InterruptedException {

		Log.e("device", "openUsbDevice  " + fd);
		return fd;

	}

	class MyThread3 extends Thread {
		@Override
		public void run() {
			super.run();
			Log.w("device", "MyThread3");// bhw
			UsbDeviceConnection connection = usbManager.openDevice(usbDevice);
			fd = connection.getFileDescriptor();// 获取文件描述符
			Log.w("device", "MyThread3  " + fd);
			
			dmcam.log_cfg(log_level_e.LOG_LEVEL_INFO,log_level_e.LOG_LEVEL_TRACE,log_level_e.LOG_LEVEL_NONE);
			
			dev = dmcam.dev_open_by_fd(fd);
			if (dev == null) {
				logUI("device", "open failed!\n");
				runOnUiThread(new Runnable() {
					public void run() {
						btnCapture.setEnabled(true);
					}
				});
				return;
			}
			logUI("DMCAM", "device open ok with fd!\n");
			
			
//	filter_args_u pix_args = new filter_args_u();
//	short case_idx =1;
//	pix_args.setCase_idx(case_idx);
//	logUI("DMCAM",
//			String.format("enable filter pix_calible\n"));
//	if (dmcam.filter_enable(dev, filter_id_e.DMCAM_FILTER_ID_PIXEL_CALIB,
//			pix_args, 4) == 0) {
//
//	}




			dev_minamp = Dialog1.getMinAmp();
			dev_exp = Dialog1.getExp();

			devSetMinAmp(dev, dev_minamp);
			devSetExposure(dev, dev_exp);

//			dmcam.cap_set_frame_buffer(dev, null, 10 * 320 * 240 * 4);
			
	        cap_cfg_t cfg = new cap_cfg_t();
	        cfg.setCache_frames_cnt(10);
	        cfg.setOn_error(null);
	        cfg.setOn_frame_ready(null);
	        cfg.setEn_save_replay((short)0);
	        cfg.setEn_save_dist_u16((short)0);
	        cfg.setEn_save_gray_u16((short)0);
	        cfg.setFname_replay(null);

	        dmcam.cap_config_set(dev, cfg);
			
			// dmcam.cap_set_callback_on_error(dev, null);
			logUI("DMCAM", " Start dmcam view ...");

			logUI("DMCAM", " Start capture ...");
			devCapturing = dmcam.cap_start(dev);
			runOnUiThread(new Runnable() {
				public void run() {
					btnCapture.setText("停止采集");
					dmcamView.start(dev);
					btnCapture.setEnabled(true);
				}
			});
		}
	}

	private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {
		public void onReceive(Context context, Intent intent) {
			String action = intent.getAction();
			Log.e("action", action);

			if (ACTION_USB_PERMISSION.equals(action)) {
				synchronized (this) {
					usbDevice = (UsbDevice) intent
							.getParcelableExtra(UsbManager.EXTRA_DEVICE);
					if (intent.getBooleanExtra(
							UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
						handler.sendEmptyMessage(1);
						if (usbDevice != null) {
							new MyThread3().start();// MyThread3
						}
					} else {
						Log.d("denied", "permission denied for device "
								+ usbDevice);
					}
				}
			}
		}
	};
}
