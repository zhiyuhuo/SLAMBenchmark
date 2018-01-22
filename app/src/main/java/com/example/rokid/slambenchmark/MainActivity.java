package com.example.rokid.slambenchmark;

import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.TextView;
import com.example.rokid.slambenchmark.TinySLAM;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Example of a call to a native method
        TextView tv = (TextView) findViewById(R.id.sample_text);
        tv.setText("SLAM Benchmark");

        TinySLAM.nativeTinySLAMInit(Environment.getExternalStorageDirectory() + "/TinySLAM/calibration.txt",
                                    Environment.getExternalStorageDirectory() + "/TinySLAM/config.yaml",
                                    false);
    }

}
