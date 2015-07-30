/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   



/* NMAP.h
*  Created by :Yiheng Feng
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.

*/

#pragma once

#include <vector>
#include <string>
#include "stdio.h"
#include <iostream>
#include <fstream>
#include "stdlib.h"
#include "math.h"

#ifndef M_PI
    #define M_PI           3.14159265358979323846
#endif

using namespace std;

class ElementID
{
 public:
   int Approach;
   int Node;
};

class LaneNodes			// lane nodes
{
public:
    ElementID index;
	int ID;
	double E_Offset;
	double N_Offset;
	double Latitude;
	double Longitude;
};


class Approach
{
public:
	int ID;
	//int Direction; //not use yet
	int type;      // 1: approach; 2:phase
	int Node_No;
	vector<LaneNodes> Nodes;
};

class Intersection
{
public:
	int ID;
	double Ref_Lat;
	double Ref_Long;
	double Ref_Ele;
	int Appro_No;
	vector<Approach> Approaches;
	string Map_Name;
	string Rsu_ID;
	//method
};

class MAP
{
public:
	Intersection intersection;
	int ID;
	int Version;
	//method
	int ParseIntersection(char* filename);
};
