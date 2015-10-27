//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  AccelerationSet4Way.cpp 
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

#include "AccelerationSet4Way.h"
AccelerationSet4Way::AccelerationSet4Way(void)
{
	longAcceleration=0.0; // -x- Along the Vehicle Longitudinal axis
	latAcceleration=0.0 ; //-x- Along the Vehicle Lateral axis
	verticalAcceleration=0.0 ; // -x- Along the Vehicle Vertical axis
	yawRate=0.0 ;
}

AccelerationSet4Way::~AccelerationSet4Way(void)
{
}
