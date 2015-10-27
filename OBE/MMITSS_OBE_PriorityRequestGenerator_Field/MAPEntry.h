//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  MAPEntry.cpp  
 *  Created by Mehdi Zamanipour
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
#pragma  once
#include <iostream>
#include <string>
using namespace std;
class MAPEntry
{
	public:
		long ID;	 // ID of the parsed MAP
		time_t AtTime;              //  Time stamp recording when the entry added 
	public:
		MAPEntry();
		MAPEntry(long id, time_t time);
		MAPEntry(MAPEntry& that);
		MAPEntry& operator=(MAPEntry& that);
	public:
		~MAPEntry();
		friend ostream &operator <<(ostream &stream, MAPEntry e)
		{
			stream<<"MAP ID is: "<<e.ID<<", at time: "<<e.AtTime<<endl;
			return stream;
		}
};
