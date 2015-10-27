package edu.arizona.sie.mmitss.pedapp;

//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class PedCrosswalk {
	double [] pointA = new double[2];
	double [] pointB = new double[2];
	double [] pointC = new double[2];
	double [] pointD = new double[2];
	double intersectioncorner1slope=0;
	double intersectioncorner2slope=0;
	double intersectioncenterslope=0;
	double headcenter;

	public PedCrosswalk(){

	}
	public void Crosswalkindicator(double[] data){
		
		// double[] data=toDoubleArray(arg);
		 this.pointA[0]=data[0];
			this.pointA[1]=data[1];

			this.pointB[0]=data[2];
			this.pointB[1]=data[3];
			this.pointC[0]=data[4];
			this.pointC[1]=data[5];
			this.pointD[0]=data[6];
			this.pointD[1]=data[7];	
	}
		/*public static double[] toDoubleArray(byte[] byteArr){
		    double[] arr=new double[byteArr.length];
		    for (int i=0;i<arr.length;i++){
		        arr[i]=byteArr[i];
		    }
		    return arr;
		}*/
		//double data= ByteBuffer.wrap(arg).order(ByteOrder.LITTLE_ENDIAN).getDouble();

		

	

	public double crosswalkheading(){
		this.intersectioncorner1slope= (this.pointA[0]-this.pointB[0])/(this.pointA[1]-this.pointB[1]);
		this.intersectioncorner2slope= (this.pointC[0]-this.pointD[0])/(this.pointC[1]-this.pointD[1]);
		this.intersectioncenterslope= (intersectioncorner1slope + intersectioncorner2slope)/2;
		this.headcenter= Math.atan(intersectioncenterslope);

		return headcenter;

	}





}
