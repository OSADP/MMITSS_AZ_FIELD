/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   

/*  IntLanePhase.h
*  Created by Mehdi Zamanipour
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.
*
*  Revision History:
*/
#pragma once
#include <sstream>
#include <iostream>
#include <fstream>
#include <string.h>
#include "stdio.h"
#include "stdlib.h"

#ifndef MAX_APRROACH
    #define MAX_APRROACH 10
#endif
#ifndef MAX_LANE
    #define MAX_LANE 10
#endif

using namespace std;
class IntLanePhase
{
public:
	int iIntersectionID;
	int iTotalApproaches;
	int iTotalPhases;
	int iTotalIngress;
	int iApproach;
	int iNoRow;
	int iInLane;
	int iOutLane;
	int iPhase;
	int iOutApproach;
	int iInLaneOutLanePhase[MAX_APRROACH][MAX_LANE][MAX_APRROACH][MAX_LANE];

	//Methods
	int ReadLanePhaseMap(char* filename);
	int findThePhaseOfInLine(int ilane);

};
