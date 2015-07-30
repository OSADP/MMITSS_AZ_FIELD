package edu.arizona.sie.mmitss.pedapp;

public class MMITSS_CONSTANTS {
	public static final double HEADING_TOLERANCE = 20; //degrees
	public static final double FENCE_DISTANCE = 10.0; //metres
	public static final int GPS_UPDT_MIN_DIST_CHANGE = 0; //meters
	public static final int GPS_UPDT_MIN_TIME_CHANGE = 0; //milliseconds
	public static final int NUM_PHASES = 8; // Number of phases in the SPAT message
	
	public static final int NORMAL = 0;
	public static final int NO_MAP_RECEIVED = -1;
	public static final int INVALID_MAP_RECEIVED = -2;
	public static final int NOT_NEAR_CROSSWALK = -3;
	public static final int NOT_FACING_CROSSWALK = -4;
	
	public static final int MAP_RECV_PORT = 15030;
	public static final int SPAT_RECV_PORT = 7890;
	public static final int SPAT_SEND_PORT = 5678;
	
	public static final int SPAT_STATE_UNAVAILABLE = 0;
	public static final int SPAT_STATE_WALK = 1;
	public static final int SPAT_STATE_FLASH = 2;
	public static final int SPAT_STATE_DONTWALK = 3;
}