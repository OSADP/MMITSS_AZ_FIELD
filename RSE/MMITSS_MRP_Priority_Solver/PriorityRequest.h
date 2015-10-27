//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  PriorityRequest.h  
 *  Created by Mehdi Zamanipour on 2/19/15
 *  University of Arizona/
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
#include <iostream>
using namespace std;

class PriorityRequest
{
	public:
		int iPhaseCycle; // the phase number in the ring that can be {0,1,2,3,10,11,12,13} for both rings
		double dRl;
		double dRu;
		int iType;
		double dReqDelay;
	public:
		PriorityRequest(void);
		PriorityRequest(const PriorityRequest& PR);
		PriorityRequest& operator=(PriorityRequest& PR);
		PriorityRequest(int phaseInRing,double rl,double ru,double delay,int type);
		~PriorityRequest(void);
};
