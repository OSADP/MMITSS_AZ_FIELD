//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  ReqEntry.cpp  
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
#include "ReqEntry.h"

ReqEntry::ReqEntry(void)
{
	VehID=0;
	VehClass=10;  // very low priority
	ETA=0;
	Phase=0;
	MinGreen =0;
	Split_Phase=-10; // means not a split phase
	iInLane=0;iOutLane=0; iStrHour=0; iStrMinute=0; iStrSecond=0; iEndHour=0;iEndMinute=0; iEndSecond=0; iVehState=0; iMsgCnt=0;
    dSpeed=0.0;
    iSendingTimeofReq=0.0;
    iSendingMsgCnt=0;
    dInitialSpeed=15.0;
    dDistanceToStpBar=0.0;
   
}

void ReqEntry::ReqEntryFromFile(long vehID, int vehClass, int eta, int phase,float mgreen, int absTime, int split_phase, int iinLane,int ioutLane,int istrHour,int istrMinute,int istrSecond,int iendHour,int iendMinute,int iendSecond,int ivehState, int imsgcnt)  
{
	VehID=vehID;
	VehClass=vehClass;
	ETA=eta;
	Phase=phase;
	MinGreen = mgreen;
    AbsTime=absTime;
	Split_Phase=split_phase;
	iInLane=iinLane;
	iOutLane=ioutLane;
	iStrHour=istrHour; 
	iStrMinute=istrMinute;
	iStrSecond=istrSecond;
	iEndHour=iendHour;
	iEndMinute=iendMinute;
	iEndSecond=iendSecond;
	iVehState=ivehState;	
	iMsgCnt=imsgcnt;
}

void ReqEntry::SendingReqEntry(long vehID, double sendingTime, double dspeed, int MsgCnt, int inlane,double distant)
{
	VehClass=0;
	ETA=0;
	Phase=0;
	MinGreen =0.0;
    AbsTime=0;
	Split_Phase=0;
	iOutLane=1;
	iStrHour=0; 
	iStrMinute=0;
	iStrSecond=0;
	iEndHour=0;
	iEndMinute=0;
	iEndSecond=0;
	iVehState=0;
	
	VehID=vehID;
	iSendingTimeofReq=sendingTime;
  	iSendingMsgCnt=MsgCnt;
    iInLane=inlane;
    dSpeed=dspeed; 
    dDistanceToStpBar=distant;	
    if (MsgCnt==0) // recorde the initial speed of the vehicle when it arrives to the range
		dInitialSpeed=dspeed;
	
}



ReqEntry::ReqEntry(ReqEntry& Req)
{
	VehID=Req.VehID;
	VehClass=Req.VehClass;
	ETA=Req.ETA;
	Phase=Req.Phase;
	MinGreen = Req.MinGreen;
    AbsTime=Req.AbsTime;
	Split_Phase=Req.Split_Phase;
	iInLane=Req.iInLane;
	iOutLane=Req.iOutLane;
	iStrHour=Req.iStrHour; 
	iStrMinute=Req.iStrMinute;
	iStrSecond=Req.iStrSecond;
	iEndHour=Req.iEndHour;
	iEndMinute=Req.iEndMinute;
	iEndSecond=Req.iEndSecond;
	iVehState=Req.iVehState;	
	iMsgCnt=Req.iMsgCnt;
	dSpeed=Req.dSpeed;
	iSendingTimeofReq=Req.iSendingTimeofReq;
	iSendingMsgCnt=Req.iSendingMsgCnt;
	dInitialSpeed=Req.dInitialSpeed;
	dDistanceToStpBar=Req.dDistanceToStpBar;
}

ReqEntry& ReqEntry::operator=(ReqEntry& Req)
{
	VehID=Req.VehID;
	VehClass=Req.VehClass;
	ETA=Req.ETA;
	Phase=Req.Phase;
	MinGreen = Req.MinGreen;
    AbsTime=Req.AbsTime;
	Split_Phase=Req.Split_Phase;
	iInLane=Req.iInLane;
	iOutLane=Req.iOutLane;
	iStrHour=Req.iStrHour; 
	iStrMinute=Req.iStrMinute;
	iStrSecond=Req.iStrSecond;
	iEndHour=Req.iEndHour;
	iEndMinute=Req.iEndMinute;
	iEndSecond=Req.iEndSecond;
	iVehState=Req.iVehState;	
	iMsgCnt=Req.iMsgCnt;
	dSpeed=Req.dSpeed;
	iSendingTimeofReq=Req.iSendingTimeofReq;
	iSendingMsgCnt=Req.iSendingMsgCnt;
	dInitialSpeed=Req.dInitialSpeed;
	dDistanceToStpBar=Req.dDistanceToStpBar;
	return *this;
}

ReqEntry::~ReqEntry(void)
{}
