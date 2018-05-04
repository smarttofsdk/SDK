package com.example.testdmcam;

import android.app.Dialog;
import android.content.Context;
import android.os.Bundle;
import android.view.View;
import android.widget.AdapterView;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;

import com.smarttof.dmcam.dev_param_e;
import com.smarttof.dmcam.dmcam;
import com.smarttof.dmcam.param_item_t;

/**
 * Created by Administrator on 2017/8/15 0015.
 */

public class Dialog1 extends Dialog {

    private static int minamp=0, exp=750;
    private SeekBar sbarMinAmp, sbarExposure,sbarMinDistance,sbarMaxDistance;
    private TextView tvMinAmpValue, tvExposureValue,tvMinDistance,tvMaxDistance;
    private Spinner spin1,spin2;
    private CheckBox autocheckBox;
    public Dialog1(Context context, int theme) {
        super(context,theme);
    }
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.f1);
        initView();
        initListener();
//        setCancelable(false);
    }
    
//    @Override
//    public void onBackPressed() {
//        this.hide();
//    }

    private  void initView()
    {
    	spin1 = (Spinner)findViewById(R.id.spin1);
        spin2 = (Spinner)findViewById(R.id.spin2);
        sbarMinAmp = (SeekBar)findViewById(R.id.sbarMinAmp);
        sbarExposure = (SeekBar)findViewById(R.id.sbarExposure);
        sbarMinDistance = (SeekBar)findViewById(R.id.sbarMinDistance);
        sbarMaxDistance =(SeekBar)findViewById(R.id.sbarMaxDistance);
        autocheckBox =(CheckBox)findViewById(R.id.ckb2);
        //sbarRevise = (SeekBar)findViewById(R.id.sbarRevise);
        tvMinAmpValue=(TextView)findViewById(R.id.tvMinAmpValue);
        tvExposureValue= (TextView)findViewById(R.id.tvExposureValue);
        tvMinDistance =(TextView)findViewById(R.id.tvMinDistance);
        tvMaxDistance = (TextView)findViewById(R.id.tvMaxDistance);
    }
    private void initListener() {
    	spin1.setOnItemSelectedListener(new spinItemSelectedListener1());
    	spin1.setSelection(2, true);
        spin2.setOnItemSelectedListener(new spinItemSelectedListener2());
        sbarMinAmp.setOnSeekBarChangeListener(new sbarMinAmpChangeListener());
        sbarExposure
                .setOnSeekBarChangeListener(new sbarExposureChangeListener());
        sbarMinDistance.setOnSeekBarChangeListener(new sbarMinDistanceChangeListener());
        sbarMaxDistance.setOnSeekBarChangeListener(new sbarMaxDistanceChangeListener());
        autocheckBox.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
			@Override
			public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
				if (TestDmcam.getDev() == null)
					return;
				if(b){
					//sbarExposure.setEnabled(false);
					TestDmcam.td.devSetAutoExpo(TestDmcam.getDev());
					
					param_item_t rparam = new param_item_t();
					//dmcamParamArray rparams = new dmcamParamArray(1);
		
					rparam.setParam_id(dev_param_e.PARAM_INTG_TIME);
		
					//rparams.setitem(0, rparam);
					if (dmcam.param_batch_get(TestDmcam.getDev(), rparam, 1)) {
						int expoUs = rparam.getParam_val().getIntg().getIntg_us();
						sbarExposure.setProgress(expoUs/15);
						tvExposureValue.setText(String.format("%dus", expoUs));
						sbarExposure.setEnabled(false);
					}
				}
				else{
					sbarExposure.setEnabled(true);
					TestDmcam.td.devUndoAutoExpo(TestDmcam.getDev());					
				}
			}
		});
    }
    
    public  static int getMinAmp(){
        return minamp;
    }
    public  static  int getExp(){
        return exp;
    }
    
    private class spinItemSelectedListener1 implements Spinner.OnItemSelectedListener {

        @Override
        public void onItemSelected(AdapterView<?> adapterView, View view, int i, long l) {
            String str = (String) spin1.getSelectedItem();
            if(str.equals("FPS = 10")) {
            	TestDmcam.td.devSetFrameRate(TestDmcam.getDev(), 10);
            }
            else if (str.equals("FPS = 15")){
            	TestDmcam.td.devSetFrameRate(TestDmcam.getDev(), 15);
            }
            else if (str.equals("FPS = 20")) {
            	TestDmcam.td.devSetFrameRate(TestDmcam.getDev(), 20);
            }
            else if(str.equals("FPS = 30")){
            	TestDmcam.td.devSetFrameRate(TestDmcam.getDev(), 30);
            }
        }

        @Override
        public void onNothingSelected(AdapterView<?> adapterView) {

        }
    }

    private class spinItemSelectedListener2 implements Spinner.OnItemSelectedListener {

        @Override
        public void onItemSelected(AdapterView<?> adapterView, View view, int i, long l) {
            String str = (String) spin2.getSelectedItem();
            if(str.equals("深度图(彩色编码)")) {
                TestDmcam.getDmcamView().paramViewType =0;
            }
            else if (str.equals("深度图(灰度编码)")){
                TestDmcam.getDmcamView().paramViewType=1;
            }
            else if (str.equals("灰度图")) {
                TestDmcam.getDmcamView().paramViewType=2;
            }
            else if(str.equals("点云图")){
                TestDmcam.getDmcamView().paramViewType=3;
            }
        }

        @Override
        public void onNothingSelected(AdapterView<?> adapterView) {

        }
    }

	private class sbarExposureChangeListener implements
			SeekBar.OnSeekBarChangeListener {
		//private long lastUpdateMs = -1;
		
		private void updateText(int progress) {
			tvExposureValue.setText(String.format("%dus", progress * 15));
		}
		
		@Override
		public void onProgressChanged(SeekBar seekBar, int progress,
				boolean fromUser) {
			this.updateText(progress);
		
			if (TestDmcam.getDev() == null)
				return;
		}
		
		@Override
		public void onStartTrackingTouch(SeekBar seekBar) {
		}
		
		@Override
		public void onStopTrackingTouch(SeekBar seekBar) {
			this.updateText(seekBar.getProgress());
		
			int expoUs = seekBar.getProgress() * 15;
			TestDmcam.td.devSetExposure(TestDmcam.getDev(), expoUs);
		}
		
	}
	
	private class sbarMinAmpChangeListener implements SeekBar.OnSeekBarChangeListener {
        //private long lastUpdateMs = -1;

		private void updateText(int progress) {
			tvMinAmpValue.setText(String.format("%d", progress));
		}
		
		@Override
		public void onProgressChanged(SeekBar seekBar, int progress,
				boolean fromUser) {
			this.updateText(progress);
		
			if (TestDmcam.getDev() == null)
				return;
		}

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {

        }

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
        	//dmcam.filter_disable(TestDmcam.getDev(), filter_id_e.DMCAM_FILTER_ID_AMP);
        	this.updateText(seekBar.getProgress());
        	
        	int tempamp=seekBar.getProgress();
        	TestDmcam.td.devSetMinAmp(TestDmcam.getDev(), tempamp);
        }
    }

    private class sbarMinDistanceChangeListener implements SeekBar.OnSeekBarChangeListener {
        private long lastUpdateMs = -1;
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean b) {
            int minDis = 100*progress;
            long curMs = System.currentTimeMillis();
            tvMinDistance.setText(String.format("%dmm",minDis));

            if(TestDmcam.getDev()==null)
                return;
            if(curMs==-1 || curMs-lastUpdateMs>300)
            {
                lastUpdateMs=curMs;
                TestDmcam.getDmcamView().paramRangeMin = minDis;
            }
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {

        }

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
            int minDis = 100*seekBar.getProgress();
            TestDmcam.getDmcamView().paramRangeMin=minDis;
        }
    }

    private class sbarMaxDistanceChangeListener implements SeekBar.OnSeekBarChangeListener {
        private long lastUpdateMs = -1;
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean b) {
            int maxDis = 100*progress;
            long curMs = System.currentTimeMillis();
            tvMaxDistance.setText(String.format("%dmm",maxDis));

            if(TestDmcam.getDev()==null)
                return;
            if(curMs==-1 || curMs-lastUpdateMs>300)
            {
                lastUpdateMs=curMs;
                TestDmcam.getDmcamView().paramRangeMax = maxDis;
            }
        }
        
        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {

        }

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
            int maxDis = 100*seekBar.getProgress();
            TestDmcam.getDmcamView().paramRangeMax=maxDis;
        }
    }
    
}
