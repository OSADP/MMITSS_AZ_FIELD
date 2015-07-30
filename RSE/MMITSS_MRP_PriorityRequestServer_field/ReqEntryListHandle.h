/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   

/*  ReqEntryListHandle.h
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


#include <stdio.h>
#include <vector>
#include <iostream>
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <istream>
#include <math.h>
#include "LinkedList.h"
#include "ReqEntry.h"
#include <math.h>


using namespace std;


//*** When the EV is in the list, the lower priority will be ignored by the Solver now. BY DJ 2012.2.10

#define EV 1      ////*Emergency vehicle: will have two split phases requests and priority as 1
#define TRANSIT 2 ////*Transit bus: will have one request and priority as 2
#define TRUCK 3
#define PEDESTRIAN 4
#define COORDINATION 6

extern double dCoordPhaseSplit;
extern double dCoordinationCycle; 
extern double fcurrentCycleTime;
extern int req_interval;
extern double dRollingHorInterval;
extern int flagForClearingInterfaceCmd;
extern int ReqListUpdateFlag;    // The Flag to identify the ReqList update
extern string RSUID;    // will get from "rsuid.txt"
extern int outputlog(char *output);
extern char logfilename[256];
extern char temp_log[256];
extern int CombinedPhase[8];
extern LinkedList<ReqEntry> Stop_List;  // Linked list to record the STOPPED vehicles
//----------------------------------------------------------------------------------------------//
int FindInReqList(LinkedList<ReqEntry> ReqList, ReqEntry TestEntry);
int FindInStopList(ReqEntry TestEntry);
void UpdateList(LinkedList<ReqEntry> &Req_List,char *RcvMsg,int phaseStatus[8]);   
void PrintList2File(char *Filename,LinkedList<ReqEntry> &ReqList,int IsCombined=0); 
void PrintList(LinkedList<ReqEntry> &ReqList);
int ReqListFromFile(char *filename,LinkedList<ReqEntry>& Req_List);    
int FindSplitPhase(int phase,int phaseStatus[8]);
int FindListHighestPriority(LinkedList<ReqEntry> Req_List);
int FindTimesInList(LinkedList<ReqEntry> Req_List,int Veh_Class);
void UpdateCurrentList( LinkedList<ReqEntry> &Req_List);
void UpdateListForPed(LinkedList<ReqEntry> &Req_List,int pedPhaseStatus[8]);
int isThePedCallInList(LinkedList<ReqEntry> Req_List,int phaseID);
int checkIfThereIsaPedInTable(LinkedList<ReqEntry>& Req_List);
