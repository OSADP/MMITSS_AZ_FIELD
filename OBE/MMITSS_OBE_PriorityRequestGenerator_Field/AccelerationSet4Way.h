//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  AccelerationSet4Way.h  
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


#ifndef _AccelerationSet4Way_h
#define _AccelerationSet4Way_h

class AccelerationSet4Way
{
	public:
		double longAcceleration;        // -x- Along the Vehicle Longitutal axis
		double latAcceleration ;        //-x- Along the Vehicle Lateral axis
		double verticalAcceleration ;  // -x- Along the Vehicle Vertical axis
		double yawRate ;
		AccelerationSet4Way(void);
		~AccelerationSet4Way(void);
};

#endif
