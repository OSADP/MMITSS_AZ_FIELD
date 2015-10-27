package edu.arizona.sie.mmitss.pedapp;

//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

import java.util.Arrays;

import android.location.Location;
import android.util.Log;

public class Approach  {
	public int ApproachNum;
	public Location A, B, C, D, a, b;
	public double heading1, heading2;
	
	public Approach(byte[] approachBuf) {
		//Log.d("ApproachBuf", Arrays.toString(approachBuf));
		ApproachNum = decode (approachBuf, 0, 2);
		
		A = new Location ("A");
		A.setLatitude((double)decode (approachBuf, 2, 4) / 1000000);
		A.setLongitude((double)decode (approachBuf, 6, 4) / 1000000);
		
		B = new Location ("B");
		B.setLatitude((double)decode (approachBuf, 12, 4) / 1000000);
		B.setLongitude((double)decode (approachBuf, 16, 4) / 1000000);
		
		C = new Location ("C");
		C.setLatitude((double)decode (approachBuf, 22, 4) / 1000000);
		C.setLongitude((double)decode (approachBuf, 26, 4) / 1000000);

		D = new Location ("D");
		D.setLatitude((double)decode (approachBuf, 32, 4) / 1000000);
		D.setLongitude((double)decode (approachBuf, 36, 4) / 1000000);

		a = new Location ("Mid Point of A and B");
		a.setLatitude((A.getLatitude() + B.getLatitude()) / 2);
		a.setLongitude((A.getLongitude() + B.getLongitude()) / 2);

		b = new Location ("Mid Point of C and D");
		b.setLatitude((C.getLatitude() + D.getLatitude()) / 2);
		b.setLongitude((C.getLongitude() + D.getLongitude()) / 2);

		heading1 = a.bearingTo(b);
		if (heading1 < 0) heading1 = b.bearingTo(a);
		heading2 = 180 + heading1;
	}
	
	private int decode (byte[] buf, int offset, int size) {
		byte[] decodeBuf = Arrays.copyOfRange(buf, offset, offset+size);
		int i = 0, num = 0, temp = 0;
		for (i = 0; i < size; i++) {
			num = num << 8;
			temp = decodeBuf[i];
			if (decodeBuf[i] < 0) {
				temp = decodeBuf[i] + 256;
			}
			num = num | temp;
		}
		return num;			
	}

	public boolean matchHeading(Location currLocation) {
		double currHeading = currLocation.getBearing();
		
		if ((currHeading > (heading1 - MMITSS_CONSTANTS.HEADING_TOLERANCE)) && 
				(currHeading < (heading1 + MMITSS_CONSTANTS.HEADING_TOLERANCE))) {
			return true;
		}
		
		if ((currHeading > (heading2 - MMITSS_CONSTANTS.HEADING_TOLERANCE)) && 
				(currHeading < (heading2 + MMITSS_CONSTANTS.HEADING_TOLERANCE))) {
			return true;
		}
		
		return false;
	}
}