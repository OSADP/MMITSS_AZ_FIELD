package edu.arizona.sie.mmitss.pedapp;

//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

import edu.arizona.sie.mmitss.pedapp.R;
import android.location.Location;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.app.Activity;
import android.content.res.Resources;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.widget.TextView;
import edu.arizona.sie.mmitss.pedapp.GPSHandler;

public class MainActivity extends Activity implements SensorEventListener {
	Button crossButton;
	GPSHandler gpsHandle;
	ImageHandler imgHandle;
	MAPHandler mapHandle;
	SPATHandler spathandle;
	float heading;
	TextView tvHeading;
	private SensorManager mSensorManager;
	private Sensor mOrientation;
	Resources resources;
	int phase = 0;
	int walktimeleft = -1;
	int notaligned ;
	Location currLocation;

	@SuppressWarnings("deprecation")
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		tvHeading = (TextView) findViewById(R.id.tvHeading);
		crossButton = (Button) findViewById(R.id.crossButton);

		// initialize your android device sensor capabilities
		mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		mOrientation = mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION);
		imgHandle = new ImageHandler(getApplicationContext(), (ImageView)findViewById(R.id.pedIndicator));
		gpsHandle = new GPSHandler(getApplicationContext());
		currLocation = new Location ("Current Phone Location");
		mapHandle = new MAPHandler();
		spathandle = new SPATHandler();
		
	}

	@Override
	protected void onResume() {
		super.onResume();
		mSensorManager.registerListener((SensorEventListener) this, mOrientation, SensorManager.SENSOR_DELAY_NORMAL);
	}

	@Override
	protected void onPause() {
		mSensorManager.unregisterListener(this );
		super.onPause();
	}

	public void onSensorChanged(SensorEvent event) {
		currLocation = gpsHandle.getLocation();
		if (currLocation == null) return;
		currLocation.setBearing(Math.round(event.values[0]));
		
		String tvText = new String();
		tvText = currLocation.getLatitude() + ", " + currLocation.getLongitude();
		tvText += ", ";
		tvText += currLocation.getBearing();
		tvText += "\nPhase : " + phase;
		tvText += " GPS Time : " + currLocation.getTime();
		tvHeading.setText(tvText);
	}

	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// not in use
	}

	public void crossButtonPressed (View view) {
		if (currLocation == null) return;
		phase = mapHandle.getPhase(currLocation);
		if (phase < MMITSS_CONSTANTS.NORMAL) {
			switch (phase) {
				case MMITSS_CONSTANTS.NOT_NEAR_CROSSWALK:
					imgHandle.notNearCrosswalk();
					break;
				case MMITSS_CONSTANTS.INVALID_MAP_RECEIVED:
					imgHandle.invalidMap();
					break;
				case MMITSS_CONSTANTS.NO_MAP_RECEIVED:
					imgHandle.noMapReceived();
					break;
				case MMITSS_CONSTANTS.NOT_FACING_CROSSWALK:
					imgHandle.notFacing();
					break;
				default:
					imgHandle.unknownError();
					break;
			}
			return;
		}

		new WaitForWalkTime().execute();
	}

	private class WaitForWalkTime extends AsyncTask<Void,Integer,Void> { 
		private int oldState = MMITSS_CONSTANTS.SPAT_STATE_UNAVAILABLE;
		private int currentState;
		private int oldTimeRemaining = 0;
		private int currTimeRemaining;
		
		@Override
		protected void onPreExecute () {
			spathandle.setPhase(phase);
			crossButton.setEnabled(false);
			Log.d("WaitForWalkTime:onPreExecute", "Waiting to receive remaining time");
			imgHandle.request();
		}
		
		private int isStateChanged () {
			currentState = spathandle.getSpatObject().getPhaseState(phase);
			
			if (currentState != MMITSS_CONSTANTS.SPAT_STATE_FLASH && oldState == MMITSS_CONSTANTS.SPAT_STATE_FLASH) {
				Log.d("isStateChanged", "End of this Ped Cycle");
				return -1;	
			}
			
			if (currentState == MMITSS_CONSTANTS.SPAT_STATE_FLASH && oldState == MMITSS_CONSTANTS.SPAT_STATE_FLASH) {
				return currentState;
			}
			
			if (currentState != oldState){
				Log.d ("StateChanged", "Phase " + phase + " state changed from " + oldState + " to " + currentState);
				oldState = currentState;
				return currentState;
			}
			return 0;
		}
		
		private int isTimeChanged() {
			currTimeRemaining = spathandle.getSpatObject().getPhaseTimes(phase) / 10;
			
			if (currTimeRemaining != oldTimeRemaining) {
				Log.d ("TimeChanged", "Phase " + phase + " time changed from " + oldTimeRemaining + " to " + currTimeRemaining);
				oldTimeRemaining = currTimeRemaining;
				return currTimeRemaining;
			}
			return 0;
		}
		
		@Override
		protected Void doInBackground(Void... params) {
			int timeChanged = 0;
			while (timeChanged >= 0) {
				switch (isStateChanged()) {
					case MMITSS_CONSTANTS.SPAT_STATE_UNAVAILABLE :
						break;
						
					case MMITSS_CONSTANTS.SPAT_STATE_DONTWALK :
						publishProgress (MMITSS_CONSTANTS.SPAT_STATE_DONTWALK);
						break;
						
					case MMITSS_CONSTANTS.SPAT_STATE_FLASH :
						timeChanged = isTimeChanged();
						if (timeChanged > 0)
							publishProgress (MMITSS_CONSTANTS.SPAT_STATE_FLASH, timeChanged);
						break;
						
					case MMITSS_CONSTANTS.SPAT_STATE_WALK :
						publishProgress (MMITSS_CONSTANTS.SPAT_STATE_WALK);
						break;
						
					default:
						timeChanged = -1;
				}
				try {Thread.sleep(25);} catch (InterruptedException e) {e.printStackTrace();}
			}
			return null;
		}
		
		@Override
		public void onProgressUpdate(Integer... values) {
			Log.d ("Progress Update", "Progress : " + values.toString());
			switch (values[0]) {
				case MMITSS_CONSTANTS.SPAT_STATE_DONTWALK :
					imgHandle.dontWalk();
					break;
				case MMITSS_CONSTANTS.SPAT_STATE_FLASH :
					imgHandle.talkNumber(values[1]);
					break;
				case MMITSS_CONSTANTS.SPAT_STATE_WALK :
					imgHandle.walkSignOn();
					break;
			}
		}
		
		@Override
		protected void onPostExecute (Void result) {
			phase = 0;
			imgHandle.timeUp();
			imgHandle.setHomeScreen();
			crossButton.setEnabled(true);
		}
	}
}
