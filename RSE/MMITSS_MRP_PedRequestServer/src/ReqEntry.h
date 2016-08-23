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
	float ETA;			//	The Estimation Time of Arrival
	int Phase;			//	The phase of the Intersection
	float MinGreen;		// Mini Green time to clear the queue in front of vehicle, only for 0 speed: Queue clear time
    time_t AbsTime;
	int Split_Phase;    // If it is =0, then no split phase; >0  represents the split phase; =-1, means will not call SplitPhase
	int iInLane,iOutLane,iStrHour,iStrMinute,iStrSecond,iEndHour,iEndMinute,iEndSecond,iVehState, iMsgCnt;	 
	
	
public:
	ReqEntry();
	ReqEntry(long vehID, int VehClass, float eta, int phase,float mgreen, int absTime, int split_phase,int iInLane,int iOutLane,int iStrHour,int iStrMinute,int iStrSecond,int iEndHour,int iEndMinute,int iEndSecond,int iVehState	, int iMsgCnt);
	ReqEntry(ReqEntry& Req);
	ReqEntry& operator=(ReqEntry& Req);
	
	~ReqEntry();
 friend ostream &operator <<(ostream &stream, ReqEntry e)
        {
            stream<<"VehID: "<<e.VehID<<"\tClass: "<<e.VehClass<<"\tETA: "<<e.ETA<<"\tPhase: "<<e.Phase<<"\tSplitPhase: "
                <<e.Split_Phase<<endl;
        return stream;
        }
};
