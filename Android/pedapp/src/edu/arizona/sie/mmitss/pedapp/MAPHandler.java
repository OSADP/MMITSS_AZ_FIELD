package edu.arizona.sie.mmitss.pedapp;

import android.location.Location;
import android.util.Log;

public class MAPHandler {
	int phase = -1;
	double degree;
	MapReceiver mapReceiver;
	MapValidate mapValid;
	GeoFence geoFence;
	Approach crossingApproach;

	public MAPHandler () {
		mapReceiver = new MapReceiver();
	}

	public int getPhase (Location currLocation) {
		if (mapReceiver.getMapBufferLen() <= 0) {
			return MMITSS_CONSTANTS.NO_MAP_RECEIVED;
		}
		
		mapValid = new MapValidate (mapReceiver.getMapBuffer(), mapReceiver.getMapBufferLen());
		
//		The below two lines are to check point in rectangle only.
//		crossingApproach = mapValid.approaches[0];
//		testIsInsideCrosswalk();

		if (mapValid.isValid() == false) {
			return MMITSS_CONSTANTS.INVALID_MAP_RECEIVED;
		}
		
		geoFence = new GeoFence (mapValid);
		geoFence.setUpNearestFence (currLocation);
		
		if (geoFence.isNearFence(currLocation) == false) {
//			return MMITSS_CONSTANTS.NOT_NEAR_CROSSWALK;
		}
		
		Approach possibleApproach1 = mapValid.approaches[geoFence.possibleApproach1];
		Approach possibleApproach2 = mapValid.approaches[geoFence.possibleApproach2];
		
		if (possibleApproach1.matchHeading(currLocation) == true) {
			crossingApproach = possibleApproach1;
			return possibleApproach1.ApproachNum;
		}
		
		if (possibleApproach2.matchHeading(currLocation) == true) {
			crossingApproach = possibleApproach2;
			return possibleApproach2.ApproachNum;
		}
		
		return MMITSS_CONSTANTS.NOT_FACING_CROSSWALK;
	}
	
	public boolean isInsideCrosswalk(Location location){
		int i, j;
		boolean result = false;
		double vertx[] = {crossingApproach.A.getLatitude(),
		                  crossingApproach.B.getLatitude(),
		                  crossingApproach.C.getLatitude(),
		                  crossingApproach.D.getLatitude()};
		
		double verty[] = {crossingApproach.A.getLongitude(),
                		  crossingApproach.B.getLongitude(),
                		  crossingApproach.C.getLongitude(),
                		  crossingApproach.D.getLongitude()};
		
		double testx = location.getLatitude();
		double testy = location.getLongitude();
		int nvert = 4;
		
		for (i = 0, j = nvert-1; i < nvert; j = i++) {
			if (((verty[i]>testy) != (verty[j]>testy)) &&
				(testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
				result = !result;
		}
		return result;
	}
	
	protected void testIsInsideCrosswalk () {
		Approach oldCrossingApproach = crossingApproach;
		crossingApproach.A.setLatitude(32.235780);
		crossingApproach.A.setLongitude(-110.952515);
		
		crossingApproach.B.setLatitude(32.235743);
		crossingApproach.B.setLongitude(-110.952482);
		
		crossingApproach.C.setLatitude(32.235747);
		crossingApproach.C.setLongitude(-110.952233);
		
		crossingApproach.D.setLatitude(32.235782);
		crossingApproach.D.setLongitude(-110.952244);
		
		Location test = new Location("testing isInsideCrosswalk");
		
		test.setLatitude(32.235767);
		test.setLongitude(-110.952342);
		Log.d("testIsInsideCrosswalk", "Result 1 : " + isInsideCrosswalk(test));
		
		test.setLatitude(32.235824);
		test.setLongitude(-110.952322);
		Log.d("testIsInsideCrosswalk", "Result 2 : " + isInsideCrosswalk(test));
		
		test.setLatitude(32.235769);
		test.setLongitude(-110.952438);
		Log.d("testIsInsideCrosswalk", "Result 3 : " + isInsideCrosswalk(test));
		
		test.setLatitude(32.236013);
		test.setLongitude(-110.952396);
		Log.d("testIsInsideCrosswalk", "Result 4 : " + isInsideCrosswalk(test));
		crossingApproach = oldCrossingApproach;
	}
}
