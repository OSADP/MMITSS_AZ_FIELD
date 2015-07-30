package edu.arizona.sie.mmitss.pedapp;

import java.util.Locale;

import edu.arizona.sie.mmitss.pedapp.R;
import android.content.Context;
import android.speech.tts.TextToSpeech;
import android.util.Log;
import android.widget.ImageView;

public class ImageHandler {

	private static final int[] cdArray = {
		R.drawable.countdown00, R.drawable.countdown01, R.drawable.countdown02, R.drawable.countdown03, R.drawable.countdown04,
		R.drawable.countdown05, R.drawable.countdown06, R.drawable.countdown07, R.drawable.countdown08, R.drawable.countdown09,
		R.drawable.countdown10, R.drawable.countdown11, R.drawable.countdown12, R.drawable.countdown13, R.drawable.countdown14,
		R.drawable.countdown15, R.drawable.countdown16, R.drawable.countdown17, R.drawable.countdown18, R.drawable.countdown19,
		R.drawable.countdown20, R.drawable.countdown21, R.drawable.countdown22, R.drawable.countdown23, R.drawable.countdown24,
		R.drawable.countdown25, R.drawable.countdown26, R.drawable.countdown27, R.drawable.countdown28, R.drawable.countdown29,
		R.drawable.countdown30, R.drawable.countdown31, R.drawable.countdown32, R.drawable.countdown33, R.drawable.countdown34,
		R.drawable.countdown35, R.drawable.countdown36, R.drawable.countdown37, R.drawable.countdown38, R.drawable.countdown39,
		R.drawable.countdown40, R.drawable.countdown41, R.drawable.countdown42, R.drawable.countdown43, R.drawable.countdown44,
		R.drawable.countdown45, R.drawable.countdown46, R.drawable.countdown47, R.drawable.countdown48, R.drawable.countdown49,
		R.drawable.countdown50, R.drawable.countdown51, R.drawable.countdown52, R.drawable.countdown53, R.drawable.countdown54,
		R.drawable.countdown55, R.drawable.countdown56, R.drawable.countdown57, R.drawable.countdown58, R.drawable.countdown59
	};
	
	ImageView cdView;
	ImageView iw;
	vibration vibrator;
	TextToSpeech ttsHandler;
	Context context;

	public ImageHandler (Context context, ImageView imgView) {
		cdView = imgView;
		this.context = context;
		cdView.setImageResource(R.drawable.welcome);
		vibrator= new vibration (context);
		ttsHandler= new TextToSpeech(context, null);
		ttsHandler.setLanguage(Locale.US);
	}

	void speak (String string) {
		Log.d("getPhaseResponse", string);
//		Toast.makeText(context, string, Toast.LENGTH_SHORT).show();
		ttsHandler.speak(string, TextToSpeech.QUEUE_FLUSH, null);
	}

	public void notaligned(){
		vibrator.notaligned();
	}

	public void dontWalk() {
		cdView.setImageResource(R.drawable.dontwalk);
		speak("Don't walk");
	}
	
	public void walkSignOn () {
		cdView.setImageResource(R.drawable.walk);
		speak("Walk Sign is ON");
	}
	
	public void setHomeScreen () {
		cdView.setImageResource(R.drawable.welcome);
	}
	
	public void talkNumber(int i) {
		speak (Integer.toString(i));
		cdView.setImageResource(cdArray[i]);
	}
	
	public void notFacing()			{speak ("You are not facing the Crosswalk");}
	public void request()			{speak ("Ped request has been sent");}
	public void notNearCrosswalk()	{speak ("You are not near any crosswalk");}
	public void invalidMap()		{speak ("Invalid map received");}
	public void unknownError()		{speak ("Unknown Error");}
	public void noMapReceived()		{speak ("No map received");}
	public void timeUp()			{speak ("Ped clear is over");}
}