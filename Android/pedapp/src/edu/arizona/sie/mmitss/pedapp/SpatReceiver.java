package edu.arizona.sie.mmitss.pedapp;
import java.io.*;
import java.net.*;
import android.util.Log;

public class SpatReceiver {
	DatagramSocket receiverSocket = null;
	DatagramSocket senderSocket = null;
	Thread receiver;
	Thread sender;
	InetAddress hostAddress;
	byte[] rx_buf;
	int rx_buf_len;
	byte[] tx_buf;
	SPATHandler spathandle;
	boolean sendFlag = false;

	public SpatReceiver()
	{
		tx_buf = new byte[8];
		sender = new Thread (new SenderThread());
		sender.start();
		
		rx_buf = new byte[200];
		receiver = new Thread (new ReceiverThread());
		receiver.start();
	}

	class SenderThread implements Runnable {
		@Override
		public void run(){
			Log.d("SPAT Sender", "Sender socket created. Waiting to send data...");

			try
			{
				senderSocket = new DatagramSocket();
				InetAddress host = InetAddress.getByName("10.254.56.255");
				DatagramPacket dp = new DatagramPacket(tx_buf, tx_buf.length, host , MMITSS_CONSTANTS.SPAT_SEND_PORT);

				while(true)
				{
					try {Thread.sleep(1000);} catch (InterruptedException e) {e.printStackTrace();}
					if (sendFlag == false) continue;
					senderSocket.send(dp);
					try {Thread.sleep(100);} catch (InterruptedException e) {e.printStackTrace();}
					senderSocket.send(dp);
					try {Thread.sleep(100);} catch (InterruptedException e) {e.printStackTrace();}
					senderSocket.send(dp);
					try {Thread.sleep(100);} catch (InterruptedException e) {e.printStackTrace();}
					senderSocket.send(dp);
					stopSending();
//					Log.d("SPAT Sender", "Sending data : " + Arrays.toString(tx_buf));
				}
			}
			catch(IOException e)
			{
				e.printStackTrace();
			}
		}
	}

	class ReceiverThread implements Runnable {
		@Override
		public void run() {
			try
			{
				//1. creating a server socket, parameter is local port number
				receiverSocket = new DatagramSocket(MMITSS_CONSTANTS.SPAT_RECV_PORT);

				//buffer to receive incoming data
				DatagramPacket incoming = new DatagramPacket(rx_buf, rx_buf.length);

				//2. Wait for an incoming data
				Log.d("SPAT Receiver", "SPAT Receiver socket created. Waiting for incoming data...");

				//communication loop
				while(true)
				{
					receiverSocket.receive(incoming);
					rx_buf = incoming.getData();
					rx_buf_len = incoming.getLength();
//					Log.d("SPAT Receiver", "Receiving data : " + Arrays.toString(rx_buf));

					try {
						Thread.sleep(200);
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

	public byte[] getSPATBuffer () {
		return rx_buf;
	}

	public int getSPATBufferLen () {
		return rx_buf_len;
	}

	public void send(int phase) {
		tx_buf[0] = (byte)phase;
		sendFlag = true;
	}
	
	public void stopSending () {
		sendFlag = false;
	}
}
