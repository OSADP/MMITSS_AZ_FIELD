//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

#pragma once


class PositionLocal3D
{
public:
	PositionLocal3D(void);
	~PositionLocal3D(void);

	public:
	double latitude ; // 4 bytes measured in degrees
	double longitude ; // 4 bytes measured in degrees
	double elevation ;  // 2 bytes measured in meters
	double positionAccuracy ; //4 bytes measured in ?? is this J2735 position accurracy??

	void Init();
};

