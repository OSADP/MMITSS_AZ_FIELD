
//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

#pragma once
class AccelerationSet4Way
{
public:
	double longAcceleration; // -x- Along the Vehicle Longitudinal axis
	double latAcceleration ; //-x- Along the Vehicle Lateral axis
	double verticalAcceleration ; // -x- Along the Vehicle Vertical axis
	double yawRate ;
	
	
	AccelerationSet4Way(void);
	~AccelerationSet4Way(void);
};

