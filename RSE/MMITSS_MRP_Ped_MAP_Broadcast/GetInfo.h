/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   



/* GetInfo.h
*  Created by :Jun Ding
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.

*/

#pragma once

#include <iostream>
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <istream>
#include <math.h>



using namespace std;

#define NumPhases 8

//---Store the file name of the Config file.
//--- For Example: ConfigInfo_Hastings.txt, which has the minGrn and maxGrn etc.

extern char INTip[64];// = "150.135.152.23";
extern char INTport[16];
extern char IPInfo[64]	;

extern char tmp_log[512];

extern int outputlog(char *output);


void get_ip_address();


void get_ip_address()
{
	  char tmp_log[256];
    fstream fs;
    fs.open(IPInfo);

    string lineread;
    getline(fs,lineread);
    outputlog("Controller IP Address is:\t");
    if(lineread.size()!=0)
    {
        //std::cout<< "Current Vehicle ID:" << RSUID <<std::endl;
        sprintf(INTip,"%s",lineread.c_str());
        cout<<INTip<<endl;		outputlog(INTip);
    }
    else
    {
        sprintf(tmp_log,"Reading IPinfo file %s problem",IPInfo);
        cout<<tmp_log<<endl;		outputlog(tmp_log);
        exit(0);
    }

    getline(fs,lineread);
	    outputlog("Controller UDP port is:\t");
	    if(lineread.size()!=0)
	    {

	        sprintf(INTport,"%s",lineread.c_str());
	        cout<<INTport<<endl;		outputlog(INTport);
	    }
	    else
	    {
	        sprintf(tmp_log,"Reading IPinfo file %s problem",IPInfo);
	        cout<<tmp_log<<endl;		outputlog(tmp_log);
	        exit(0);
    }



    outputlog("\n");

    fs.close();
}
