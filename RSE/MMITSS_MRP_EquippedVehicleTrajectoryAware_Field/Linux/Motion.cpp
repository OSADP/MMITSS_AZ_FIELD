//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  Motion.cpp  
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
#include "Motion.h"

Motion::Motion(void)
{
    speed                 = 0.0; 
    heading               = 0.0; 
    angle                 = 0.0;  
    accel.longAcceleration= 0.0  ; // -x- Along the Vehicle Longitudinal axis
	accel.latAcceleration = 0.0; //-x- Along the Vehicle Lateral axis
	accel.verticalAcceleration= 0.0 ; // -x- Along the Vehicle Vertical axis
	accel.yawRate             = 0.0; 
}


Motion::~Motion(void)
{
}
