//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  PositionLocal3D.cpp  
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


#include "PositionLocal3D.h"

PositionLocal3D::PositionLocal3D(void)
{
	latitude         = 0.0; 
	longitude        = 0.0; 
	elevation        = 0.0;  
	positionAccuracy = 0.0; 
}


PositionLocal3D::~PositionLocal3D(void)
{
}
