//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  PositionLocal3D.h  
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

#ifndef _PositionLocal3D_h
#define _PositionLocal3D_h

class PositionLocal3D
{
	public:
		PositionLocal3D(void);
		~PositionLocal3D(void);

	public:
		double latitude ;         // 4 bytes in degrees
		double longitude ;        // 4 bytes in degrees
		double elevation ;        // 2 bytes in meters
		double positionAccuracy ; // 4 bytes 
};

#endif
