package edu.arizona.sie.mmitss.pedapp;

//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

import java.util.Arrays;

public class SpatValidate {
	private int[] phaseID;
	private int[] phaseState;
	private int[] phaseTimes;

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
	
	public SpatValidate() {

	}
	
	public int getPhaseState (int phase) {
//		Log.d("getPhaseState", "Phase " + phase + " is in state " + phaseState[phase - 1]);
		return phaseState[phase - 1];
	}
	
	public int getPhaseTimes (int phase) {
//		Log.d("getPhaseTimes", "Phase " + phase + " has " + phaseTimes[phase - 1] + " remaining.");
		return phaseTimes[phase - 1];
	}

	public void setPhaseInfo(byte[] spatBuffer, int spatBufferLen) {
		phaseID = new int[MMITSS_CONSTANTS.NUM_PHASES];
		phaseState = new int[MMITSS_CONSTANTS.NUM_PHASES];
		phaseTimes = new int[MMITSS_CONSTANTS.NUM_PHASES];
		int i;
		int offset = 0;
		
		offset = 8; // Number of bytes to store GPS time in SPAT
		
		for (i = 0; i < MMITSS_CONSTANTS.NUM_PHASES; i++) {
			phaseID[i] = decode (spatBuffer, offset, 4);
			offset += 4;
			phaseState[i] = decode (spatBuffer, offset, 4);
			offset += 4;
			phaseTimes[i] = decode (spatBuffer, offset, 4);
			offset += 4;
		}
		
//		Log.d ("SpatValidate", "phaseIDs   : " + Arrays.toString(phaseID));
//		Log.d ("SpatValidate", "phaseState : " + Arrays.toString(phaseState));
//		Log.d ("SpatValidate", "phaseTimes : " + Arrays.toString(phaseTimes));
	}
}
