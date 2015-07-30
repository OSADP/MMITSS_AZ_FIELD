/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   

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
