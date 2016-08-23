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


#define EV 1      ////*Emergency vehicle: will have two split phases requests and priority as 1
#define TRANSIT 2 ////*Transit bus: will have one request and priority as 2
#define TRUCK 3
#define COORDINATION 6

extern int req_interval;
extern double dRollingHorInterval;
extern int ReqListUpdateFlag;    // The Flag to identify the ReqList update
extern int flagForClearingInterfaceCmd;
extern string RSUID;    // will get from "rsuid.txt"
extern int outputlog(char *output);
extern char logfilename[256];
extern char temp_log[256];
extern int CombinedPhase[8];
extern LinkedList<ReqEntry> Stop_List;  // Linked list to record the STOPPED vehicles
extern double dCoordinationCycle;
extern double dCoordPhaseSplit;
extern float fcurrentCycleTime;

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
void UpdateCurrentList( LinkedList<ReqEntry> &Req_List, int coordphase1,int coordphase2,double coordphase1split,double coordphase2split);
