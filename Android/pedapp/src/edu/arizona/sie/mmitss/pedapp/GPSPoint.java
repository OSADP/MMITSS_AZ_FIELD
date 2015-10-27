package edu.arizona.sie.mmitss.pedapp;

//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

public class GPSPoint {
	private int lattitude, longitude;
	
	public GPSPoint (int a, int b) {
		this.lattitude = a;
		this.longitude = b;
	}
	
	public int getLattitude () {
		return lattitude;
	}
	
	public int getLongitude () {
		return longitude;
	}
}