package edu.arizona.sie.mmitss.pedapp;

//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

import android.content.Context;
import android.os.Vibrator;

public class vibration {

	Vibrator v;
	
	public vibration(Context context){
		 v = (Vibrator) context.getSystemService(context.VIBRATOR_SERVICE);
		
	}
	public void changephase(){
		v.vibrate(1000);
	}
	
	public void notaligned(){
		// This example will cause the phone to vibrate "SOS" in Morse Code
		// In Morse Code, "s" = "dot-dot-dot", "o" = "dash-dash-dash"
		// There are pauses to separate dots/dashes, letters, and words
		// The following numbers represent millisecond lengths
		int dot = 200;      // Length of a Morse Code "dot" in milliseconds
		int dash = 500;     // Length of a Morse Code "dash" in milliseconds
		int short_gap = 200;    // Length of Gap Between dots/dashes
		int medium_gap = 500;   // Length of Gap Between Letters
		int long_gap = 1000;    // Length of Gap Between Words
		long[] pattern = {
		    0,  // Start immediately
		    dot, short_gap, dot, short_gap, dot,    // s
		    medium_gap,
		    dash, short_gap, dash, short_gap, dash, // o
		    medium_gap,
		    dot, short_gap, dot, short_gap, dot,    // s
		    long_gap
		};
		v.vibrate(pattern, -1);
	}
}

