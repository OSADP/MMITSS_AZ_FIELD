//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  Brakes.cpp
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
#include "stdafx.h"
#include "Brakes.h"



Brakes::Brakes(void)
{
	brackAppPres=unavailable;                        // 4 bits
	wheelBrakesUnavialable=false ;
	spareBit              =false ;                 
	tractionCntrStat      =TracCntrlunavailable;     // 2 Bits
	antiLckBrkStat         =AntiLockunavailable;     //2 bits
	stabilityCtrlStat      =Stabilitycntlunavailable;
	brakeBstAppld          =BrakeBoostunavailable;
     ausciliaryBrkStat     =AuxiliaryBrkunavailable;
}


Brakes::~Brakes(void)
{
}
