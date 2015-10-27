//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  ReqEntry.h  
 *  Created by Mehdi Zamanipour on 9/27/14.
 *  University of Arizona
 *  ATLAS Research Center
 *  College of Engineering
 *
 *  This code was develop under the supervision of Professor Larry Head
 *  in the ATLAS Research Center.
 *
 *  Revision History:
 * 
 *  
 */
#pragma once

#include <iostream>
#include <fstream>
#include <time.h>
using namespace std;

class ReqEntry
{
public:
	long VehID;			//  The ID of the vehicle
	int VehClass;		//	The Class of the vehicle
	int ETA;			//	The Estimation Time of Arrival
	int Phase;			//	The phase of the Intersection
	float MinGreen;		// Mini Green time to clear the queue in front of vehicle, only for 0 speed: Queue clear time
    time_t AbsTime;
	int Split_Phase;    // If it is =0, then no split phase; >0  represents the split phase; =-1, means will not call SplitPhase
  	int iInLane,iOutLane,iStrHour,iStrMinute,iStrSecond,iEndHour,iEndMinute,iEndSecond,iVehState, iMsgCnt;	
  	double iSendingTimeofReq;
  	int iSendingMsgCnt;
  	double dSpeed; 
  	double dInitialSpeed;
  	double dDistanceToStpBar;
  
	
public:
	ReqEntry();
    void ReqEntryFromFile(long vehID, int vehClass, int eta, int phase,float mgreen, int absTime, int split_phase, int iinLane,int ioutLane,int istrHour,int istrMinute,int istrSecond,int iendHour,int iendMinute,int iendSecond,int ivehState, int imsgcnt);
	void SendingReqEntry(long vehID, double sendingTime, double dspeed, int MsgCnt, int inlane, double distant);
	ReqEntry(ReqEntry& Req);
	ReqEntry& operator=(ReqEntry& Req);
public:
	~ReqEntry();
	friend ostream &operator <<(ostream &stream, ReqEntry e)
        {
            stream<<"VehID: "<<e.VehID<<"\tClass: "<<e.VehClass<<"\tETA: "<<e.ETA<<"\tPhase: "<<e.Phase<<"\tSplitPhase: "
                <<e.Split_Phase<<endl;
        return stream;
        }
};
