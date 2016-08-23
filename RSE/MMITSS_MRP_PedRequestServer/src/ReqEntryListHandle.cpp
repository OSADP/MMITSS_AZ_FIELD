#include <ReqEntryListHandle.h>
	//----- ReqListUpdateFlag=1: ADD a new request
    //----- ReqListUpdateFlag=2: UPDATED request (changing the speed, joining the queue, leaving the queue)
    //----- ReqListUpdateFlag=3: DELETE an obsolete request
    //----- ReqListUpdateFlag=4: CANCEL a request due to leaving intersection
 
int FindTimesInList(LinkedList<ReqEntry> Req_List,int Veh_Class)
{
    Req_List.Reset();
    int times=0;

    while(!Req_List.EndOfList())
    {
        if(Req_List.Data().VehClass==Veh_Class)
        {
            times++;
        }

        Req_List.Next();
    }

    return times;
}
//IF there is EV, the change of lower priority will not change the status
void UpdateList(LinkedList<ReqEntry> &Req_List,char *RcvMsg,int phaseStatus[8])
{
	flagForClearingInterfaceCmd=0;
	int iNewReqDiviser=0;
	int iRecvReqListDiviser=0;
    ReqEntry NewReq; // default NewReq.Split_Phase=-10;
    char RSU_ID[16],msg[16];
    sscanf(RcvMsg,"%s %s %ld %d %f %d %f %ld %d %d %d %d %d %d %d %d %d %d ",msg,RSU_ID,&NewReq.VehID,&NewReq.VehClass,
      &NewReq.ETA,&NewReq.Phase,&NewReq.MinGreen,&NewReq.AbsTime,&NewReq.iInLane,&NewReq.iOutLane,&NewReq.iStrHour,&NewReq.iStrMinute,&NewReq.iStrSecond,
      &NewReq.iEndHour,&NewReq.iEndMinute,&NewReq.iEndSecond,&NewReq.iVehState,&NewReq.iMsgCnt);
   	int HighestP=FindListHighestPriority(Req_List);
    int SplitPhase=0;
    if(NewReq.VehClass==EV)
    {
        SplitPhase=FindSplitPhase(NewReq.Phase,phaseStatus);
        sprintf(temp_log,"split phase is: %d\n",SplitPhase);
        outputlog(temp_log);
    }
   //---------------Beginning of Handling the Received Requests.----------------//
 
	int pos=FindInReqList(Req_List,NewReq);
    if( ( strcmp(msg,"request")==0 || strcmp(msg,"coord_request")==0 ) && NewReq.Phase > 0)  // the vehicle is approaching the intersection or in it is in the queue and requesting a phase
    {
        if(pos<0)  // Request is NOT in the List. It includes the ReqNo<=0 case as well
        {
            NewReq.Split_Phase=SplitPhase; //UPDATE the split phase when first time occurs.
			
			//new request, must solve, save first time solve ETA and Abstime
		    Req_List.InsertRear(NewReq);
			Req_List.Reset(0);
			sprintf(temp_log,"%ld %d %f %d %f %ld %d %d %d %d %d %d %d %d %d %d \n",Req_List.Data().VehID,Req_List.Data().VehClass,Req_List.Data().ETA,Req_List.Data().Phase,Req_List.Data().MinGreen,
			Req_List.Data().AbsTime, Req_List.Data().iInLane, Req_List.Data().iOutLane, Req_List.Data().iStrHour,Req_List.Data().iStrMinute,Req_List.Data().iStrSecond,
			Req_List.Data().iEndHour,Req_List.Data().iEndMinute,Req_List.Data().iEndSecond,Req_List.Data().iVehState, Req_List.Data().iMsgCnt);
			outputlog(temp_log);
			ReqListUpdateFlag=1;
		    sprintf(temp_log,"***Add Request**** { %s }\t \tFLAG {%d} at time (%ld).\n",RcvMsg,ReqListUpdateFlag,time(NULL));
            outputlog(temp_log);
        }
        else  // The request is already in the list. 
        {
            Req_List.Reset(pos);
            iRecvReqListDiviser = (int) Req_List.Data().iMsgCnt /10;
            iNewReqDiviser = (int) NewReq.iMsgCnt /10;
            outputlog("All the data in the request list is:\n");
            if (iRecvReqListDiviser==iNewReqDiviser) // the recived SRM with identical MsgCnt type is already in the list, so we should not add the new req to the list. We should just update the ETA of the request in the list 
            {
				//~ if (Req_List.Data().VehID==99998 || Req_List.Data().VehID==99999)
				//~ {
						//~ if (Req_List.Data().ETA==0 && Req_List.Data().MinGreen > 1)
							//~ Req_List.Data().MinGreen = dCoordPhaseSplit-fcurrentCycleTime;
						//~ if (Req_List.Data().ETA>0 && Req_List.Data().MinGreen ==0 )
							//~ Req_List.Data().ETA = dCoordinationCycle-fcurrentCycleTime;
				//~ }else
				//~ {
					ReqListUpdateFlag=2;  
					Req_List.Data().ETA=Req_List.Data().ETA-dRollingHorInterval;
					if (Req_List.Data().ETA<0)
						Req_List.Data().ETA=0;
					sprintf(temp_log,"%ld %d %f %d %f %ld %d %d %d %d %d %d %d %d %d %d\n",Req_List.Data().VehID,Req_List.Data().VehClass,Req_List.Data().ETA,Req_List.Data().Phase,Req_List.Data().MinGreen,
					Req_List.Data().AbsTime, Req_List.Data().iInLane, Req_List.Data().iOutLane, Req_List.Data().iStrHour,Req_List.Data().iStrMinute,Req_List.Data().iStrSecond,
					Req_List.Data().iEndHour,Req_List.Data().iEndMinute,Req_List.Data().iEndSecond,Req_List.Data().iVehState,Req_List.Data().iMsgCnt);
				//~ }
			}
			else
			{
				if(NewReq.VehClass==EV)  //Modified by YF: Solve the problem that if duing the first request the split phase is -1, it will never change!!!
				{
					NewReq.Split_Phase=SplitPhase;
				}
				else
				{
					NewReq.Split_Phase=Req_List.Data().Split_Phase;
				}
				if( (NewReq.VehClass==1) || (HighestP!=1) ) // if the new request is an EV or if there is no EV in the request list
				{
					ReqListUpdateFlag=2;  
				}
			
				Req_List.Data()=NewReq; // Update the existed entry.
				Req_List.Data().AbsTime=NewReq.AbsTime;
				sprintf(temp_log,"%ld %d %f %d %f %ld %d %d %d %d %d %d %d %d %d %d\n",Req_List.Data().VehID,Req_List.Data().VehClass,Req_List.Data().ETA,Req_List.Data().Phase,Req_List.Data().MinGreen,
				Req_List.Data().AbsTime, Req_List.Data().iInLane, Req_List.Data().iOutLane, Req_List.Data().iStrHour,Req_List.Data().iStrMinute,Req_List.Data().iStrSecond,
				Req_List.Data().iEndHour,Req_List.Data().iEndMinute,Req_List.Data().iEndSecond,Req_List.Data().iVehState,Req_List.Data().iMsgCnt);
				outputlog(temp_log);
			}
        }

        //----------------End Update the Requests list according to the received time.--------//
    }
    else if(strcmp(msg,"request_clear")==0) 
    {
		if(pos>=0) // this is the first time ewe receive the clear request 
        {
			Req_List.Reset(pos);
            Req_List.DeleteAt();
            sprintf(temp_log,"***CLEAR**** %s at time (%ld).\n",RcvMsg,time(NULL));
            outputlog(temp_log);
            cout<<temp_log<<endl;
            if( Req_List.ListSize()>0 ) // if there is another request in the table we should colve the problem again 
				ReqListUpdateFlag=4;  
			else 
				flagForClearingInterfaceCmd=1;
	    }
	    else
	    {
			if( Req_List.ListSize()>0 ) // if there is another request in the table we should colve the problem again 
				ReqListUpdateFlag=4;  
			else 
				flagForClearingInterfaceCmd=1;
		}
    }
    //---------------End of Handling the Received Requests.----------------//
   
    if (Req_List.ListSize()==0 && ReqListUpdateFlag!=4 )
    {
        sprintf(temp_log,"*************Empty List at time (%ld).\n",time(NULL));
        outputlog(temp_log);
        cout<<temp_log<<endl;
        ReqListUpdateFlag=0;
    }
}

int FindListHighestPriority(LinkedList<ReqEntry> ReqList)
{
	// The smaller number, the higher priority
    ReqList.Reset();

    int priority=10;  // TRANSIT(2) is lower than EV(1): a large number

    if(ReqList.ListEmpty())
    {
        return priority;
    }

    while(!ReqList.EndOfList())
    {
        if(ReqList.Data().VehClass<priority)
        {
            priority=ReqList.Data().VehClass;
        }
        else
        {
            ReqList.Next();
        }
    }

    return priority;
}

int FindSplitPhase(int phase,int phaseStatus[8])
{
    //*** From global array CombinedPhase[] to get the combined phase :get_configfile() generates CombinedPhase[]
    //*** If the phase exsits, the value is not 0; if not exists, the default value is '0'.; "-1"  means will not change later
    //*** The argument phase should be among {1..8}
    //int Phase_Seq[8]={1,2,3,4,5,6,7,8};
    //*** NOW also consider the case that if phase 6 is current or next, will only call 2, and 6 will be in the solver as well

    int combined_phase=0;

    switch (phase)
    {
    case 1:
        combined_phase= CombinedPhase[6-1];
        break;
    case 2: // if current phase or next phase is 6: we should not call phase 5, because cannot reverse from 6 to 5;
        {
            if(phaseStatus[6-1]==2 || phaseStatus[6-1]==7) // do not consider not enable case: phaseStatus[6-1]==3
                combined_phase=-1;
            else
                combined_phase= CombinedPhase[5-1];
            break;
        }
    case 3:
        combined_phase= CombinedPhase[8-1];
        break;
    case 4:
        {
            if(phaseStatus[8-1]==2 || phaseStatus[8-1]==7)
                combined_phase=-1;
            else
                combined_phase= CombinedPhase[7-1];
            break;
        }
    case 5:
        combined_phase= CombinedPhase[2-1];
        break;
    case 6:
        {
            if(phaseStatus[2-1]==2 || phaseStatus[2-1]==7)
                combined_phase=-1;
            else
                combined_phase= CombinedPhase[1-1];
            break;
        }
    case 7:
        combined_phase= CombinedPhase[4-1];
        break;
    case 8:
        {
            if(phaseStatus[4-1]==2 || phaseStatus[4-1]==7)
                combined_phase=-1;
            else
                combined_phase= CombinedPhase[3-1];
            break;
        }
    default:
        	cout<< "*** Wrong Phase Information***"<<endl;
			system("pause");
			break;
		
    }

    return combined_phase;
}

// Find if the TestEntry is in the STOP list, return value is the position. //
int FindInStopList(ReqEntry TestEntry)
{
    Stop_List.Reset();
    int temp=-1;
    if(Stop_List.ListEmpty())
    {
        return temp;
    }
    else
    {
        while(!Stop_List.EndOfList())
        {
            if (Stop_List.Data().VehID== TestEntry.VehID)
            {
                return Stop_List.CurrentPosition();
            }
            Stop_List.Next();
        }
    }
    return temp;
}

 // Find if the TestEntry is in the list, return value is the position.
int FindInReqList(LinkedList<ReqEntry> ReqList, ReqEntry TestEntry)
{
    ReqList.Reset();
    int temp=-1;

    if(ReqList.ListEmpty())
    {
        return temp;
    }
    else
    {
        while(!ReqList.EndOfList())
        {
            if(ReqList.Data().VehID== TestEntry.VehID)
            {
		
                return ReqList.CurrentPosition();
            }
            ReqList.Next();
        }
    }
    return temp;
}


int ReqListFromFile(char *filename,LinkedList<ReqEntry>& Req_List)
{
    fstream fss;
    fss.open(filename,fstream::in);
    ReqEntry req_temp;

    int ReqNo=-1;
    char RSU_ID[20];
    long vehicle_ID;
    int Veh_Class,Req_Phase,AbsTime,Req_SplitPhase;
    float ETA,MinGrn;
	
	
	//Added by MZ
	int iinlane,ioutlane,istrhour,istrminute,istrsecond,iendhour,iendminute,iendsecond,ivehstate,imsgcnt; 
	string lineread;
    Req_List.ClearList();

    getline(fss,lineread); // Read first line: should be [Num_req] [No] [UpdateFlag]
    sscanf(lineread.c_str(),"%*s %d %d",&ReqNo,&ReqListUpdateFlag);
    cout<<"The total Requests is:"<<ReqNo<<endl;

    while(!fss.eof() && ReqNo>0)
    {
        getline(fss,lineread);
        if(lineread.size()!=0)
        {
			sscanf(lineread.c_str(),"%s %ld %d %f %d %f %d %d %d %d %d %d %d %d %d %d %d %d",RSU_ID,&vehicle_ID,&Veh_Class,&ETA, 
			&Req_Phase,&MinGrn,&AbsTime,&Req_SplitPhase,&iinlane,&ioutlane,&istrhour,&istrminute,&istrsecond,&iendhour,&iendminute,&iendsecond,&ivehstate,&imsgcnt); 
            ReqEntry req_temp(vehicle_ID,Veh_Class,ETA,Req_Phase,MinGrn,AbsTime,Req_SplitPhase,iinlane,ioutlane,istrhour,istrminute,istrsecond,iendhour,iendminute,iendsecond,ivehstate,imsgcnt);  
            Req_List.InsertAfter(req_temp);
            cout<<lineread<<endl;
        }
        else
        {
            cout<<"Blank Line in requests.txt file!\n";
        }
    }
    fss.close();
    return ReqNo;

}

void PrintList(LinkedList<ReqEntry> &ReqList)
{
    if (ReqList.ListEmpty())
    {
        cout<<"NO entry in the list!\n";
    }
    else
    {
        ReqList.Reset();
        while(!ReqList.EndOfList())
        {
            cout<<ReqList.Data();
            ReqList.Next();
        }
        cout<<"!!!!List print OVER!!!"<<endl;
    }
}


void PrintList2File(char *Filename,LinkedList<ReqEntry> &ReqList,int IsCombined)
{
    // If IsCombined=1 (There is no EV) print combined phase information into "requests.txt". // BY DJ 2012.3.27
    // The argument of IsCombined is optional, default value is 0, means no EV 
    // phase_status is from the ASC controller
    // NOW also consider the case that if phase 6 is current or next, will only call 2, and 6 will be in the solver as well // BY DJ 2012.3.29

    FILE *pFile=fopen(Filename,"w");
    int TotalReqNum=0;
    int CurPhase;
    int SplitPhase;
    int times;
    if (!ReqList.ListEmpty() && pFile!=NULL) 
    {
		if(IsCombined==0)  // output to "requests_combined.txt"
		{
			times=FindTimesInList(ReqList,EV);
			ReqList.Reset();
			if(times==1)  //ONLY have one EV will possiblly call split phase
			{
				if (!ReqList.ListEmpty())
				{
					while(!ReqList.EndOfList())
					{
						if ((ReqList.Data().iInLane==0)&&(ReqList.Data().iOutLane==0)&&(ReqList.Data().iStrHour==0)&&(ReqList.Data().iStrMinute==0)&&(ReqList.Data().iStrSecond==0)&& (ReqList.Data().VehClass!=COORDINATION) ) // some times there is a residual request in the table with all of these element equal zero, this if prevents it
						{	
							sprintf(temp_log,"\n******************  A residual Resuests Detected ****************** At time %ld. \n",time(NULL));
							outputlog(temp_log); 
							cout<< temp_log<< endl;
						}
						else
						{
							CurPhase=ReqList.Data().Phase;
							if((ReqList.Data().VehClass==EV && ReqList.Data().Split_Phase<=0) || ReqList.Data().VehClass==TRANSIT || ReqList.Data().VehClass==TRUCK)
								TotalReqNum+=1;
							if(ReqList.Data().VehClass==EV && ReqList.Data().Split_Phase>0)
								TotalReqNum+=2;   // We need send two requests.
							ReqList.Next();
						}
					}
				}
				fprintf(pFile,"Num_req %d %d\n",TotalReqNum,ReqListUpdateFlag);
				if (!ReqList.ListEmpty())
				{
					ReqList.Reset();
					while(!ReqList.EndOfList())
					{
						
						if ((ReqList.Data().iInLane==0)&&(ReqList.Data().iOutLane==0)&&(ReqList.Data().iStrHour==0)&&(ReqList.Data().iStrMinute==0)&&(ReqList.Data().iStrSecond==0)&& (ReqList.Data().VehClass!=COORDINATION) ) // some times there is a residual request in the table with all of these element equal zero, this if prevents it
						{	
							sprintf(temp_log,"\n******************  A residual Resuests Detected ****************** At time %ld. \n",time(NULL));
							outputlog(temp_log); 
							cout<< temp_log<< endl;
						}
						else
						{
							CurPhase=ReqList.Data().Phase;              // Current request phase
							SplitPhase=ReqList.Data().Split_Phase;
							if(SplitPhase<=0 || ReqList.Data().VehClass==TRANSIT || ReqList.Data().VehClass==TRUCK)
							{
								fprintf(pFile,"%s %ld %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d\n",RSUID.c_str(),ReqList.Data().VehID,ReqList.Data().VehClass,
								ReqList.Data().ETA,CurPhase,ReqList.Data().MinGreen,ReqList.Data().AbsTime, ReqList.Data().iInLane, ReqList.Data().iOutLane, ReqList.Data().iStrHour,ReqList.Data().iStrMinute,ReqList.Data().iStrSecond,
								ReqList.Data().iEndHour,ReqList.Data().iEndMinute,ReqList.Data().iEndSecond,ReqList.Data().iVehState, ReqList.Data().iMsgCnt);
							}
							else
							{
								fprintf(pFile,"%s %ld %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d\n",RSUID.c_str(),ReqList.Data().VehID,ReqList.Data().VehClass,
									ReqList.Data().ETA,CurPhase,ReqList.Data().MinGreen,ReqList.Data().AbsTime, ReqList.Data().iInLane, ReqList.Data().iOutLane, ReqList.Data().iStrHour,ReqList.Data().iStrMinute,ReqList.Data().iStrSecond,
									ReqList.Data().iEndHour,ReqList.Data().iEndMinute,ReqList.Data().iEndSecond,ReqList.Data().iVehState, ReqList.Data().iMsgCnt);
								fprintf(pFile,"%s %ld %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d\n",RSUID.c_str(),ReqList.Data().VehID,ReqList.Data().VehClass,
									ReqList.Data().ETA,SplitPhase,ReqList.Data().MinGreen,ReqList.Data().AbsTime, ReqList.Data().iInLane, ReqList.Data().iOutLane, ReqList.Data().iStrHour,ReqList.Data().iStrMinute,ReqList.Data().iStrSecond,
									ReqList.Data().iEndHour,ReqList.Data().iEndMinute,ReqList.Data().iEndSecond,ReqList.Data().iVehState, ReqList.Data().iMsgCnt);
		   
							}
						}
						ReqList.Next();
					}
				}
			}
			else  //if(times!=1): will only call phases requested without split_phase
			{
			
				TotalReqNum=ReqList.ListSize();
					fprintf(pFile,"Num_req %d %d\n",TotalReqNum,ReqListUpdateFlag);
				//	if (!ReqList.ListEmpty())
			//	{
					ReqList.Reset();
					while(!ReqList.EndOfList())
					{
						if ((ReqList.Data().iInLane==0)&&(ReqList.Data().iOutLane==0)&&(ReqList.Data().iStrHour==0)&&(ReqList.Data().iStrMinute==0)&&(ReqList.Data().iStrSecond==0)&& (ReqList.Data().VehClass!=COORDINATION) ) // some times there is a residual request in the table with all of these element equal zero, this if prevents it
						{	cout<<"Here2"<<endl;	cout<<ReqList.Data().VehID<<endl;
							sprintf(temp_log,"\n******************  A residual Resuests Detected ****************** At time %ld. \n",time(NULL));
							outputlog(temp_log); 
							cout<< temp_log<< endl;
						}
						else
						{
							fprintf(pFile,"%s %ld %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d\n",RSUID.c_str(),ReqList.Data().VehID,ReqList.Data().VehClass,
							ReqList.Data().ETA,ReqList.Data().Phase,ReqList.Data().MinGreen,ReqList.Data().AbsTime, ReqList.Data().iInLane, ReqList.Data().iOutLane, ReqList.Data().iStrHour,ReqList.Data().iStrMinute,ReqList.Data().iStrSecond,
							ReqList.Data().iEndHour,ReqList.Data().iEndMinute,ReqList.Data().iEndSecond,ReqList.Data().iVehState, ReqList.Data().iMsgCnt);
						}
						ReqList.Next();
					}
				//}
			}
		}
		else if(IsCombined==1)  //----- EV will have split phase requests: output "requests.txt": will add split_phase in the end
		{
			TotalReqNum=ReqList.ListSize();
			fprintf(pFile,"Num_req %d %d\n",TotalReqNum,0);
			if (!ReqList.ListEmpty())
			{
				ReqList.Reset();
				while(!ReqList.EndOfList())
				{
				   if ((ReqList.Data().iInLane==0)&&(ReqList.Data().iOutLane==0)&&(ReqList.Data().iStrHour==0)&&(ReqList.Data().iStrMinute==0)&&(ReqList.Data().iStrSecond==0)&& (ReqList.Data().VehClass!=COORDINATION)) // some times there is a residual request in the table with all of these element equal zero, this if prevents it
					{	cout<<"Here3"<<endl; cout<<ReqList.Data().VehID<<endl;
						sprintf(temp_log,"\n******************  A residual Resuests Detected ****************** At time %ld. \n",time(NULL));
						outputlog(temp_log);
						cout<< temp_log<< endl;
					}
					else
					{
						fprintf(pFile,"%s %ld %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d %d \n",RSUID.c_str(),ReqList.Data().VehID,ReqList.Data().VehClass,
						ReqList.Data().ETA,ReqList.Data().Phase,ReqList.Data().MinGreen,ReqList.Data().AbsTime,ReqList.Data().Split_Phase,
						ReqList.Data().iInLane, ReqList.Data().iOutLane, ReqList.Data().iStrHour,ReqList.Data().iStrMinute,ReqList.Data().iStrSecond,
						ReqList.Data().iEndHour,ReqList.Data().iEndMinute,ReqList.Data().iEndSecond,ReqList.Data().iVehState, ReqList.Data().iMsgCnt);
					}
						
					ReqList.Next();
				}
			}
		} //*** else if(IsCombined==1) ***//
	} // endl of if (!Req_List.ListEmpty()) 

    fclose(pFile);
}



void UpdateCurrentList( LinkedList<ReqEntry> &Req_List,int coordphase1,int coordphase2,double coordphase1split,double coordphase2split)
{
	int HighestP=FindListHighestPriority(Req_List);
    Req_List.Reset();
    while (!Req_List.EndOfList())
    {
		if ( (time(NULL) - Req_List.Data().AbsTime> req_interval ) && (Req_List.Data().VehClass!=COORDINATION) )// if the time that we received the last SRM is (req_interval second) ago and the SRM has not been updated during this interval, this request is a residual request and should be deleted!
		{
			Req_List.Reset(Req_List.CurrentPosition());
			sprintf(temp_log," ************Residual request with ID %ld DELETED from the requests list  at time %ld ************\n",Req_List.Data().VehID,time(NULL));
		    if( (Req_List.Data().VehClass==1) || (HighestP!=1) ) // if the new request is an EV or if there is no EV in the request list
			{
				ReqListUpdateFlag=3;  
			}	
		    Req_List.DeleteAt();
            outputlog(temp_log);cout<<temp_log<<endl;
			continue;
		}else
		{
			if ( (Req_List.Data().VehClass==COORDINATION) )
			{
				if (coordphase1split==coordphase2split) // if the two coordinated phse have the same split time
				{
					if (Req_List.Data().ETA==0 && Req_List.Data().MinGreen > 1)
							Req_List.Data().MinGreen = dCoordPhaseSplit-fcurrentCycleTime;
					if (Req_List.Data().ETA>0 && Req_List.Data().MinGreen ==0 )
							Req_List.Data().ETA = max(0.0,dCoordinationCycle-fcurrentCycleTime);
				}else
				{
					if (coordphase1==Req_List.Data().Phase)
					{
						if (Req_List.Data().ETA==0 && Req_List.Data().MinGreen > 1)
							Req_List.Data().MinGreen = max(coordphase2split,coordphase1split)-fcurrentCycleTime;
						if (Req_List.Data().ETA>0 && Req_List.Data().MinGreen ==0 )
						{
							if (fcurrentCycleTime<=max(coordphase2split-coordphase1split ,0.0 )) // if signal is on the coordinated split time , one coordinated phase has ETA and the other has MinGreen 
								Req_List.Data().ETA = max(coordphase2split-coordphase1split ,0.0 )-fcurrentCycleTime;
							else
								Req_List.Data().ETA = dCoordinationCycle-fcurrentCycleTime + max(coordphase2split-coordphase1split ,0.0 );
						}
					}
					if (coordphase2==Req_List.Data().Phase)
					{
						if (Req_List.Data().ETA==0 && Req_List.Data().MinGreen > 1)
							Req_List.Data().MinGreen = max(coordphase2split,coordphase1split)-fcurrentCycleTime;
						if (Req_List.Data().ETA>0 && Req_List.Data().MinGreen ==0 )
						{
							if (fcurrentCycleTime<=max(coordphase1split-coordphase2split ,0.0 ))
								Req_List.Data().ETA = max(coordphase1split-coordphase2split ,0.0 )-fcurrentCycleTime;
							else
								Req_List.Data().ETA = dCoordinationCycle-fcurrentCycleTime + max(coordphase1split-coordphase2split ,0.0 );
						}
					}		
				}
				
			}else
			{
				if (Req_List.Data().ETA>0)
					Req_List.Data().ETA=Req_List.Data().ETA-dRollingHorInterval;
				else
					Req_List.Data().ETA=0;	
			}
		}
	    Req_List.Next();
    }
}	
