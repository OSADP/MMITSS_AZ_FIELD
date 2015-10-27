package edu.arizona.sie.mmitss.pedapp;

//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

import android.app.AlertDialog;
import android.app.Service;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.IBinder;
import android.provider.Settings;
import android.util.Log;

public class GPSHandler extends Service {

	Location location; // location
	LocationManager locationManager;
	Context context;

	public GPSHandler(Context context) {
		this.context = context;
		locationManager = (LocationManager) this.context.getSystemService(Context.LOCATION_SERVICE);
		
		while (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER) == false) {
//			showGPSSettingsAlert();
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 
				MMITSS_CONSTANTS.GPS_UPDT_MIN_TIME_CHANGE, MMITSS_CONSTANTS.GPS_UPDT_MIN_DIST_CHANGE, locationListener);
		Log.d("GPSHandler", "Start Requesting Location Updates");
	}
	
	LocationListener locationListener = new LocationListener() {
		
		@Override
		public void onStatusChanged(String provider, int status, Bundle extras) {
			// TODO Auto-generated method stub
		}
		
		@Override
		public void onProviderEnabled(String provider) {
			// TODO Auto-generated method stub
		}
		
		@Override
		public void onProviderDisabled(String provider) {
			// TODO Auto-generated method stub
		}
		
		@Override
		public void onLocationChanged(Location location) {
			setLocation(location);
		}
	};
	
	public Location getLocation() {
		if (location == null) 
			return locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
		else
			return location;
	}
	
	protected void setLocation(Location location) {
//		Log.d("LocationListener", "Location Updated : " + location.getLatitude() + location.getLongitude());
		this.location = location;		
	}

	/**
	 * Function to show settings alert dialog
	 * On pressing Settings button will launch Settings Options
	 * */
	public void showGPSSettingsAlert(){
		AlertDialog.Builder alertDialog = new AlertDialog.Builder(context);
   	 
        // Setting Dialog Title
        alertDialog.setTitle("GPS is settings");
 
        // Setting Dialog Message
        alertDialog.setMessage("GPS is not enabled. Do you want to go to settings menu?");
 
        // On pressing Settings button
        alertDialog.setPositiveButton("Settings", new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialog,int which) {
            	Intent intent = new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS);
            	context.startActivity(intent);
            }
        });
 
        // on pressing cancel button
        alertDialog.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialog, int which) {
            	dialog.cancel();
            }
        });
 
        // Showing Alert Message
        alertDialog.show();
	}

	@Override
	public IBinder onBind(Intent arg0) {
		return null;
	}	
}