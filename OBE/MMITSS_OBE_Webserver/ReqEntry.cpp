//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  ReqEntry.cpp
*  Modified by Mehdi Zamanipour for OBU webpage display.
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.
*
*  Revision History:
*  
*/
#include "ReqEntry.h"

ReqEntry::ReqEntry(void)
{
	VehID=0;
	VehClass=0;
	ETA=0;
	Phase=0;
	MinGreen =0;
	Split_Phase=0;
	iInLane=0;iOutLane=0;iStartHour=0;iStartMin=0;iStartSec=0;iEndHour=0;iEndMin=0;iEndSec=0;
	iVehicleState=0;
	strcpy(cInLane,"");
	strcpy(cOutLane,"");
	iMsgCnt=0;
}


ReqEntry::ReqEntry(int vehID, int vehClass, float eta, int phase, float mgreen,int absTime,int split_phase, int inlane,int outlane, int strhour, int strmin, int strsec, int endhour, int endmin , int endsec, int vehstate, char* cinlane,char* coutlane,int imsgcnt)
{
	VehID=vehID;
	VehClass=vehClass;
	ETA=eta;
	Phase=phase;
	MinGreen = mgreen;
    AbsTime=absTime;
	Split_Phase=split_phase;
	iInLane=inlane;
	iOutLane=outlane;
	iStartHour=strhour;
	iStartMin=strmin;
	iStartSec=strsec;
	iEndHour=endhour;
	iEndMin=endmin;
	iEndSec=endsec;
	iVehicleState=vehstate;
	strcpy(cInLane,cinlane);
	strcpy(cOutLane,coutlane);
	iMsgCnt=imsgcnt;
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
	iStartHour=Req.iStartHour;
	iStartMin=Req.iStartMin;
	iStartSec=Req.iStartSec;
	iEndHour=Req.iEndHour;
	iEndMin=Req.iEndMin;
	iEndSec=Req.iEndSec;
	iVehicleState=Req.iVehicleState;
	strcpy(cInLane,Req.cInLane);
	strcpy(cOutLane,Req.cOutLane);
	iMsgCnt=Req.iMsgCnt;
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
	iStartHour=Req.iStartHour;
	iStartMin=Req.iStartMin;
	iStartSec=Req.iStartSec;
	iEndHour=Req.iEndHour;
	iEndMin=Req.iEndMin;
	iEndSec=Req.iEndSec;
	iVehicleState=Req.iVehicleState;
	strcpy(cInLane,Req.cInLane);
	strcpy(cOutLane,Req.cOutLane);
	iMsgCnt=Req.iMsgCnt;
	return *this;
}

ReqEntry::~ReqEntry(void)
{}
