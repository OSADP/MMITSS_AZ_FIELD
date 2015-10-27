//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  MAPEntry.cpp  
 *  Created by Mehdi Zamanipour on 7/9/14.
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

#include "MAPEntry.h"
MAPEntry::MAPEntry(void)
{
	ID=0;
	AtTime=0;
	//Flag=-999;
}
MAPEntry::MAPEntry(long id, time_t time)
{
	ID=id;
	AtTime=time;
}

MAPEntry::MAPEntry(MAPEntry& that)
{
	ID=that.ID;
	AtTime=that.AtTime;
}
MAPEntry& MAPEntry::operator=(MAPEntry& that)
{
	ID=that.ID;
	AtTime=that.AtTime;
	return *this;
}
MAPEntry::~MAPEntry(void)
{
}
