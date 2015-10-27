package edu.arizona.sie.mmitss.pedapp;

//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

import java.io.*;
import java.net.*;
import java.util.Arrays;

import android.util.Log;

public class SocketHandler {
	DatagramSocket receiverSocket = null;
	DatagramSocket senderSocket = null;
	Thread receiver;
	Thread sender;
	int sendingPort;
	int receivingPort;
	InetAddress hostAddress;
	byte[] rx_buf;
	byte[] tx_buf;
	SPATHandler spathandle;
	boolean sendFlag = false;
	boolean receiveFlag = false;

	public SocketHandler()
	{
		tx_buf = new byte[8];
		sendingPort = 5678;
		sender = new Thread (new SenderThread());
		sender.start();
		
		rx_buf = new byte[8];
		receivingPort = 6789;
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
				DatagramPacket  dp = new DatagramPacket(tx_buf, tx_buf.length, host , sendingPort);

				while(true)
				{
					if (sendFlag == false) continue;
					senderSocket.send(dp);
					Log.d("SPAT Sender", "Sending data : " + Arrays.toString(tx_buf));
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

	class ReceiverThread implements Runnable {
		@Override
		public void run() {
			try
			{
				//1. creating a server socket, parameter is local port number
				receiverSocket = new DatagramSocket(receivingPort);

				//buffer to receive incoming data
				DatagramPacket incoming = new DatagramPacket(rx_buf, rx_buf.length);

				//2. Wait for an incoming data
				Log.d("SPAT Receiver", "Receiver socket created. Waiting for incoming data...");

				//communication loop
				while(true)
				{
					if (receiveFlag == false) continue;
					receiverSocket.receive(incoming);
					rx_buf = incoming.getData();
					Log.d("SPAT Receiver", "Receiving data : " + Arrays.toString(rx_buf));

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

	public int receive () {
		return (int)rx_buf[0];
	}

	public void send(int phase) {
		tx_buf[0] = (byte)phase;
		sendFlag = true;
		receiveFlag = true;
	}
	
	public void stopSending () {
		sendFlag = false;
		receiveFlag = false;
	}
}