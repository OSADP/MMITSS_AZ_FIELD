//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  PriorityRequest.cpp  
 *  Created by Mehdi Zamanipour on 2/19/15
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

#include "PriorityRequest.h"

PriorityRequest::PriorityRequest(void)
{
	iPhaseCycle=0;
	dRl=0.0;	
	dRu=0.0;
	iType=0;
	dReqDelay=0.0;
}

PriorityRequest::PriorityRequest(int phaseInRingNo,double rl,double ru,double delay,int type)
{
	iPhaseCycle=phaseInRingNo;
	dRl=rl;
	dRu=ru;
	iType=type;
	dReqDelay=delay;
}

PriorityRequest::PriorityRequest(const PriorityRequest& PR)
{
    iPhaseCycle=PR.iPhaseCycle;
	dRl=PR.dRl;
	dRu=PR.dRu;
	iType=PR.iType;
	dReqDelay=PR.dReqDelay;
}

PriorityRequest& PriorityRequest::operator=(PriorityRequest& PR)
{
    iPhaseCycle=PR.iPhaseCycle;
	dRl=PR.dRl;
	dRu=PR.dRu;
	iType=PR.iType;
	dReqDelay=PR.dReqDelay;
    return *this;
}

PriorityRequest::~PriorityRequest(void)
{
}
