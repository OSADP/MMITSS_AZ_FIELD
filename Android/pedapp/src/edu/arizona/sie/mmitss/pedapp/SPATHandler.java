package edu.arizona.sie.mmitss.pedapp;

import android.util.Log;

public class SPATHandler {
	private SpatReceiver spatReceiver;
	private SpatValidate spatValidate;
	int phase;
	int currentState = -1;

	public SPATHandler () {
		spatReceiver = new SpatReceiver ();
		spatValidate = new SpatValidate ();
	}

	public void setPhase (int phase) {
		this.phase = phase;
		
		spatUpdate();
		
		Log.d ("setPhase","Requesting phase " + phase);
		
		// Logic to compare available time for the requested phase and send a request if necessary
		switch (currentState) {
			case MMITSS_CONSTANTS.SPAT_STATE_WALK:
				Log.d("setPhase", "SPAT_STATE_WALK");
				break;
			case MMITSS_CONSTANTS.SPAT_STATE_DONTWALK:
				Log.d("setPhase", "SPAT_STATE_DONTWALK");
				spatReceiver.send(phase);
				break;
			case MMITSS_CONSTANTS.SPAT_STATE_FLASH:
				Log.d("setPhase", "SPAT_STATE_FLASH");
				spatReceiver.send(phase);
				break;
			case MMITSS_CONSTANTS.SPAT_STATE_UNAVAILABLE:
				Log.d("setPhase", "SPAT_STATE_UNAVAILABLE");
				break;
		}
	}
	
	public void spatUpdate () {
		spatValidate.setPhaseInfo(spatReceiver.getSPATBuffer(), spatReceiver.getSPATBufferLen());
		currentState  = spatValidate.getPhaseState(phase);
	}
	
	public SpatValidate getSpatObject () {
		spatUpdate();
		return spatValidate;
	}
}