package edu.arizona.sie.mmitss.pedapp;

import java.io.*;
import java.net.*;
import java.util.Arrays;

import android.util.Log;

public class MapReceiver {
	DatagramSocket receiverSocket = null;
	Thread receiver;
	byte[] rx_buf;
	int rx_len = 0;

	public MapReceiver()
	{
		rx_buf = new byte[1024];
		receiver = new Thread (new ReceiverThread());
		receiver.start();
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	class ReceiverThread implements Runnable {

		@Override
		public void run() {
			try
			{
				//1. creating a server socket, parameter is local port number
				receiverSocket = new DatagramSocket(MMITSS_CONSTANTS.MAP_RECV_PORT);

				//buffer to receive incoming data
				DatagramPacket incoming = new DatagramPacket(rx_buf, rx_buf.length);

				//2. Wait for an incoming data
				Log.d("MAP Receiver", "Receiver socket created on port " + MMITSS_CONSTANTS.MAP_RECV_PORT + ". Waiting for incoming data...");

				//communication loop
				while(true)
				{
					receiverSocket.receive(incoming);
					rx_buf = incoming.getData();
					rx_len = incoming.getLength();
//					Log.d("UDPReceiver", "Receiving data : " + Arrays.toString(rx_buf));

					try {
						Thread.sleep(1000);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
			catch(IOException e)
			{
				e.printStackTrace();
			}
		}
	}
	
	public byte[] getMapBuffer () {
		return rx_buf;
	}
	
	public int getMapBufferLen () {
		return rx_len;
	}
}