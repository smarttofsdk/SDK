package com.example.testdmcam;

import android.app.Dialog;
import android.content.Context;
import android.os.Bundle;
import android.widget.ArrayAdapter;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;

/**
 * Created by Administrator on 2017/8/22 0022.
 */

public class Dialog2 extends Dialog {
    private static int reviseValue;
    private static long modFreq=48,frameRate=30;
    @SuppressWarnings("unused")
	private SeekBar sbarRevise;
    @SuppressWarnings("unused")
	private TextView tvReviseValue;
    private Spinner spin_freq, spin_frame;
    private String[] freq = {"12", "24", "36", "48", "60"};
    private String[] frame = {"30", "15", "60"};

    public Dialog2(Context context,int theme) {
        super(context, theme);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.f2);
        initView();
//        initListener();
    }

    private void initView() {
        spin_freq = (Spinner) findViewById(R.id.spin_freq);
        spin_frame = (Spinner) findViewById(R.id.spin_frame_rate);
        ArrayAdapter<String> adapter1 = new ArrayAdapter<String>(getContext(), android.R.layout.simple_list_item_1, freq);
        spin_freq.setAdapter(adapter1);
        ArrayAdapter<String> adapter2 = new ArrayAdapter<String>(getContext(), android.R.layout.simple_list_item_1, frame);
        spin_frame.setAdapter(adapter2);
        sbarRevise = (SeekBar) findViewById(R.id.sbarRevise);
        tvReviseValue = (TextView) findViewById(R.id.tv_revise_Value);

    }

//    private void initListener() {
//        spin_freq.setOnItemSelectedListener(new spinItemSelectedListener3());
//        spin_frame.setOnItemSelectedListener(new spinItemSelectedListener4());
//        sbarRevise.setOnSeekBarChangeListener(new sbarReviseChangeListener());
//    }

    public static long getModFreq() {
        return modFreq;
    }

    public static long getFrameRate() {
        return frameRate;
    }

    public static int getReviseValue() {
        return reviseValue;
    }

//    private class spinItemSelectedListener3 implements  Spinner.OnItemSelectedListener{
//
//        @Override
//        public void onItemSelected(AdapterView<?> adapterView, View view, int i, long l) {
//            //Long.ValurOf("String")转成Long包装类
//            modFreq = Long.parseLong((String)spin_freq.getSelectedItem());
//            TestDmcam.td.devSetModifyFreq(TestDmcam.getDev(),modFreq);
//        }
//
//        @Override
//        public void onNothingSelected(AdapterView<?> adapterView) {
//
//        }
//    }
//    private class spinItemSelectedListener4 implements  Spinner.OnItemSelectedListener{
//
//        @Override
//        public void onItemSelected(AdapterView<?> adapterView, View view, int i, long l) {
//            frameRate =Long.parseLong((String)spin_frame.getSelectedItem());
//            TestDmcam.td.devSetFrameRate(TestDmcam.getDev(),frameRate);
//        }
//        @Override
//        public void onNothingSelected(AdapterView<?> adapterView) {
//
//        }
//    }
//    private class sbarReviseChangeListener implements SeekBar.OnSeekBarChangeListener{
//        private long lastUpdateMs = -1;
//        @Override
//        public void onProgressChanged(SeekBar seekBar, int progress, boolean b) {
//            long curMs =System.currentTimeMillis();
//            tvReviseValue.setText(String.format("%d %n",progress));
//            if(TestDmcam.getDev()==null)
//                return;
//            if (lastUpdateMs == -1 || curMs - lastUpdateMs > 300){
//                lastUpdateMs = curMs;
//                TestDmcam.td.devSetRevise(TestDmcam.getDev(),progress);
//            }
//        }
//
//        @Override
//        public void onStartTrackingTouch(SeekBar seekBar) {
//
//        }
//
//        @Override
//        public void onStopTrackingTouch(SeekBar seekBar) {
//            reviseValue = seekBar.getProgress();
//            TestDmcam.td.devSetRevise(TestDmcam.getDev(),reviseValue);
//        }
//    }
}

