//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  ListHandle.h
*  Modified by Mehdi Zamanipour for OBU webpage display.
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.
*
*  Revision History:
*  
*/

#pragma once
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <sstream>
#include <assert.h>
#include "LinkedList.h"
#include "ReqEntry.h"
#include "ReqEntry.h"

using namespace std;

#define EV 1
#define TRANSIT 2
#define TRUCK 3
#define PEDESTRIAN 4
int  FindCombinedPhase(int phase);
void FindCombinedReq(LinkedList<ReqEntry> &ReqList, ReqEntry &TestEntry);
void FindCombinedReqList(LinkedList<ReqEntry> &ReqList, LinkedList<ReqEntry> &ReqList_Com);
void PrintReqList(LinkedList<ReqEntry> ReqList);
void PrintReqListCombined(LinkedList<ReqEntry> ReqList);
int  ReqListFromFile(char *filename,LinkedList<ReqEntry>& Req_List);
void fillUpPedStatus(LinkedList<ReqEntry> ReqList,int ped[8][2]);
int isTherePedCallInPSM(LinkedList<ReqEntry> ReqList);
