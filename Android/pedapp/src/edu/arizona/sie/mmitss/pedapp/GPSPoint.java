package edu.arizona.sie.mmitss.pedapp;


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