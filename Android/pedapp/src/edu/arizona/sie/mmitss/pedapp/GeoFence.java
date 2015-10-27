package edu.arizona.sie.mmitss.pedapp;
//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

import android.location.Location;
import android.util.Log;

public class GeoFence {
	MapValidate mapValid;
	int possibleApproach1 = -1, possibleApproach2 = -1;
	
	public GeoFence(MapValidate mapValid) {
		this.mapValid = mapValid;
	}

	public void setUpNearestFence(Location currLocation) {
		double minDistance = Integer.MAX_VALUE;
		
		Log.d("setUpNearestFence", "Current Location : " + currLocation.getLatitude() + ", " + currLocation.getLongitude());
		
		for (int i = 0; i < mapValid.num_approaches; i++) {
			if (mapValid.approaches[i].a.distanceTo(currLocation) < minDistance) {
				minDistance = mapValid.approaches[i].a.distanceTo(currLocation);
				possibleApproach1 = i;
			}
		}
		
		possibleApproach2 = (possibleApproach1 + 1) % mapValid.num_approaches;
		
		Log.d("setUpNearestFence", "Nearest Crosswalk : " + 
				mapValid.approaches[possibleApproach1].a.getLatitude() + ", " + 
				mapValid.approaches[possibleApproach1].a.getLongitude());
	}
	
	public boolean isNearFence (Location currLocation) {
		if (possibleApproach1 < 0) {
			Log.e("isNearFence", "No Possible Approaches Set");
			return false;
		}
		
		if (mapValid.approaches[possibleApproach1].a.distanceTo(currLocation) < MMITSS_CONSTANTS.FENCE_DISTANCE) {
			Log.d("isNearFence", "Near possibleApproach1.a");
			return true;
		}
		
		if (mapValid.approaches[possibleApproach2].b.distanceTo(currLocation) < MMITSS_CONSTANTS.FENCE_DISTANCE) {
			Log.d("isNearFence", "Near possibleApproach2.b");
			return true;
		}
		
		return false;
	}
}