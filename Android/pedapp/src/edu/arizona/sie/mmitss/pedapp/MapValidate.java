package edu.arizona.sie.mmitss.pedapp;

//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

import java.util.ArrayList;
import java.util.Arrays;

import android.util.Log;

public class MapValidate {
	int num_approaches;
	static int num_bytes_per_approach = 40;
	
	Approach[] approaches;
	
	public MapValidate (byte[] MapBuffer, int MapBufferLength) {
		ArrayList<Approach> approachList = new ArrayList<Approach>();
		int i;
		num_approaches = MapBufferLength / num_bytes_per_approach;
				
		for (i = 0; i < num_approaches; i++) {
			approachList.add(new Approach(
					Arrays.copyOfRange(MapBuffer,						// copy a sub array of num_bytes_per_approach size 
							i * num_bytes_per_approach, 				// starting point is at every 'num_bytes_per_approach'th byte
							(i + 1) * num_bytes_per_approach			// end point it num_bytes_per_approach bytes away from that
							)));										// Pass this sub buffer to be decoded for every approach
		}
		
		approaches = new Approach[approachList.size()];
		approaches = (Approach[]) approachList.toArray(approaches);
		
		for (i = 0; i < num_approaches; i++) {
			Log.d("ApproachDecode", "Approach Number:" + approaches[i].ApproachNum);
			Log.d("ApproachDecode", "Point A: " + approaches[i].A.getLatitude() + ", " + approaches[i].A.getLongitude());
			Log.d("ApproachDecode", "Point B: " + approaches[i].B.getLatitude() + ", " + approaches[i].B.getLongitude());
			Log.d("ApproachDecode", "Point C: " + approaches[i].C.getLatitude() + ", " + approaches[i].C.getLongitude());
			Log.d("ApproachDecode", "Point D: " + approaches[i].D.getLatitude() + ", " + approaches[i].D.getLongitude());
			Log.d("ApproachDecode", "Headings " + approaches[i].heading1 + ", " + approaches[i].heading2);
		}
	}

	public boolean isValid() {
		// TODO provide validation code for map
		return true;
	}
}
