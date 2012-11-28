package com.example.ballmaze;

import android.os.Bundle;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.view.Display;
import android.view.Menu;
import android.graphics.Point;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.widget.TextView;
import android.widget.ImageView;
import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends Activity implements SensorEventListener {
	private TextView tv;
	private ImageView iv;
	private SensorManager sManager;
	private Timer timer;
	private int screenHeight;
	private int screenWidth;
	private int ivHeight;
	private int ivWidth;
	private float[] gyroValues;

	@SuppressLint("NewApi")
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		// Get the text label thing
		tv = (TextView) findViewById(R.id.tv);
		// Get the android guy
		iv = (ImageView) findViewById(R.id.iv);
		ivHeight = iv.getHeight();
		ivWidth = iv.getWidth();

		// Get a hook to the system service
		sManager = (SensorManager) getSystemService(SENSOR_SERVICE);

		Display display = getWindowManager().getDefaultDisplay();
		Point size = new Point();
		display.getSize(size);
		screenHeight = size.y;
		screenWidth = size.x;
		gyroValues = new float[3];
		timer = new Timer();
		timer.schedule(new TimerTask() {
			@Override
			public void run() {
				TimerMethod();
			}
		}, 0, 10);
	}

	private void TimerMethod() {
		this.runOnUiThread(Timer_Tick);
	}

	private Runnable Timer_Tick = new Runnable(){
		@SuppressLint("NewApi")
		public void run() {
			float androidX = iv.getX();
			float androidY = iv.getY();

			iv.setX((float)Math.floor(androidX+gyroValues[1]*10));
			iv.setY((float)Math.floor(androidY+gyroValues[0]*10));
			if (iv.getX() < 0) {
				iv.setX(0);
			} else if (iv.getX() > (screenWidth - ivWidth - 40)) {
				iv.setX(screenWidth - ivWidth - 40);
			}
			if (iv.getY() < 0) {
				iv.setY(0);
			} else if (iv.getY() > (screenHeight - ivHeight - 100)) {
				iv.setY(screenHeight - ivHeight - 100);
			}
		}		
	};

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.activity_main, menu);
		return true;
	}

	@Override
	protected void onResume() 
	{
		super.onResume();
		/* register the sensor listener to listen to the gyroscope sensor, use the 
		 * callbacks defined in this class, and gather the sensor information as  
		 * quick as possible*/
		sManager.registerListener(this, sManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR),SensorManager.SENSOR_DELAY_FASTEST);
	}

	//When this Activity isn't visible anymore
	@Override
	protected void onStop() 
	{
		//unregister the sensor listener
		sManager.unregisterListener(this);
		super.onStop();
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// DO NOTHING!
	}

	@SuppressLint("NewApi")
	@Override
	public void onSensorChanged(SensorEvent event) {
		// If sensor is unreliable, return void
		if (event.accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE)
		{
			return;
		}

		float androidX = iv.getX();
		float androidY = iv.getY();


		// Else it will output the Roll, Pitch and Yawn values
		tv.setText("Orientation X (Roll) :"+ Float.toString(event.values[2]) +"\n"+
				"Orientation Y (Pitch) :"+ Float.toString(event.values[1]) +"\n"+
				"Orientation Z (Yaw) :"+ Float.toString(event.values[0]) +"\n"+
				"Gyro X"+ Float.toString(gyroValues[0]) +"\n"+
				"Gyro Y"+ Float.toString(gyroValues[1]) +"\n"+
				"Gyro Z"+ Float.toString(gyroValues[2]) +"\n"+
				"Android Guy X:" + Float.toString(androidX) +"\n"+
				"Android Guy Y:" + Float.toString(androidY) +"\n"+
				"Screen X:" + Integer.toString(screenWidth) +"\n"+
				"Screen Y:" + Integer.toString(screenHeight));


		
			gyroValues[0] = event.values[0];
		
			gyroValues[1] = event.values[1];
		
			gyroValues[2] = event.values[2];
	
	}

}


