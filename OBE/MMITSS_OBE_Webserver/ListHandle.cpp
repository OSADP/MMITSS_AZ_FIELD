/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   

/*  ListHandle.cpp
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

#include "ListHandle.h"
void FindCombinedReq(LinkedList<ReqEntry> &ReqList, ReqEntry &TestEntry)
{
	ReqList.Reset();

	int SplitPhase=FindCombinedPhase(TestEntry.Phase);

	if(!ReqList.ListEmpty())
	{
		while(!ReqList.EndOfList())
		{
			if( (ReqList.Data().VehID==TestEntry.VehID) && ReqList.Data().Phase == TestEntry.Phase)
			{
				ReqList.DeleteAt();
				if(TestEntry.VehClass==TRANSIT) break;
			}
			else if( (ReqList.Data().VehID==TestEntry.VehID) && ReqList.Data().Phase == SplitPhase)
			{
				ReqList.DeleteAt();
				TestEntry.Split_Phase=SplitPhase;
				break;
			}
			else
			{
				ReqList.Next();
			}
		}
	}
}

void FindCombinedReqList(LinkedList<ReqEntry> &ReqList, LinkedList<ReqEntry> &ReqList_Com)
{
	ReqList_Com.Reset();
	while(!ReqList.ListEmpty())
	{
		ReqList.Reset();

		ReqEntry TempEntry=ReqList.Data();

		FindCombinedReq(ReqList,TempEntry);

		ReqList_Com.InsertAfter(TempEntry);
	}
}

void PrintReqList(LinkedList<ReqEntry> ReqList)
{
	if(ReqList.ListEmpty()==0)
	{
		ReqList.Reset();
		while(!ReqList.EndOfList())
		{
			ReqEntry R(ReqList.Data());
			cout<<R.VehID<<"\t"<<R.VehClass<<" \t"<<R.ETA<<"\t"<<R.Phase<<"\t"<<R.MinGreen<<endl;
			ReqList.Next();
		}
	}
	else
	{
		cout<<"Empty Req List!\n";
	}
}




void PrintReqListCombined(LinkedList<ReqEntry> ReqList)
{
	if(ReqList.ListEmpty()==0)
	{
		ReqList.Reset();
		while(!ReqList.EndOfList())
		{
			ReqEntry R(ReqList.Data());
			cout<<R.VehID<<"\t"<<R.VehClass<<" \t"<<R.ETA<<"\t"<<R.Phase<<"\t"<<R.Split_Phase<<"\t"<<R.MinGreen<<endl;
			ReqList.Next();
		}
	}
	else
	{
		cout<<"Empty Req List!\n";
	}
}


int FindCombinedPhase(int phase)
{
    //*** From global array CombinedPhase[] to get the combined phase :get_configfile() generates CombinedPhase[]***//
    //*** If the phase exsits, the value is not 0; if not exists, the default value is '0'. ***//
    //*** The argument phase should be among {1..8}
    //int Phase_Seq[8]={1,2,3,4,5,6,7,8};
	int CombinedPhase[8]={1,2,3,4,5,6,7,8};
    int combined_phase=0;

    switch (phase)
    {
    case 1:
        combined_phase= CombinedPhase[6-1];
        break;
    case 2:
        combined_phase= CombinedPhase[5-1];
        break;
    case 3:
        combined_phase= CombinedPhase[8-1];
        break;
    case 4:
        combined_phase= CombinedPhase[7-1];
        break;
    case 5:
        combined_phase= CombinedPhase[2-1];
        break;
    case 6:
        combined_phase= CombinedPhase[1-1];
        break;
    case 7:
        combined_phase= CombinedPhase[4-1];
        break;
    case 8:
        combined_phase= CombinedPhase[3-1];
        break;
    default:
         system("pause");
        break;
    }
    return combined_phase;
}

int ReqListFromFile(char *filename,LinkedList<ReqEntry>& Req_List)
{
	fstream fss;
	fss.open(filename,fstream::in);
	ReqEntry req_temp;
	int ReqNo=0;
	int OBU_ID;
	int Veh_Class,Req_Phase;
	int abs_time;
	float ETA,MinGrn;
	int inlane, outlane, strhour, strmin, strsec, endhour, endmin, endsec;
	int vehstate;
	char cinlane[64];
	char coutlane[64];
	int tmpinlane,tmpoutlane;
	int imsgcnt;
	string lineread;
	Req_List.ClearList();
	getline(fss,lineread);
	sscanf(lineread.c_str(),"%*s %d",&ReqNo); 
	while(!fss.eof())
	{
		getline(fss,lineread);

		if(lineread.size()!=0)    
		{
			sscanf(lineread.c_str(),"%d %d %f %d %f %d %d %d %d %d %d %d %d %d %d %d",&OBU_ID,&Veh_Class,
				&ETA,&Req_Phase,&MinGrn,&abs_time, &inlane, &outlane, &strhour, &strmin, &strsec, &endhour, &endmin, &endsec, &vehstate, &imsgcnt);
				
			if ((-300>inlane) || (300<inlane))
			   inlane=0;
			if ((-300>outlane) || (300<outlane))
			   outlane=0;
			if ((-300>imsgcnt) || (300<imsgcnt))
			   imsgcnt=0;
			tmpinlane=inlane%10;
			inlane= (int) (inlane-tmpinlane)/10;         // calculating the in comming approach  and put it in inlane!
			sprintf(cinlane,"%d . %d",inlane,tmpinlane); //   cinlane would be :  approach . lane
			tmpoutlane=outlane%10;
			outlane= (int) (outlane-tmpoutlane)/10;      // calculating the outgoing approach  and put it in inlane!
			sprintf(coutlane,"%d . %d", outlane, tmpoutlane);
			ReqEntry req_temp(OBU_ID,Veh_Class,ETA,Req_Phase,MinGrn,abs_time,0,inlane, outlane, strhour, strmin, strsec, endhour, endmin, endsec, vehstate,cinlane,coutlane, imsgcnt);
			Req_List.InsertAfter(req_temp);
		}
	}
	fss.close();
    return ReqNo;
}


int isTherePedCallInPSM(LinkedList<ReqEntry> ReqList)
{
	int temp=-1;
	if(ReqList.ListEmpty()==0)
	{
		ReqList.Reset();
		while(!ReqList.EndOfList())
		{
			if (ReqList.Data().VehClass==PEDESTRIAN)
				return 1;
			ReqList.Next();
		}
	}
	return temp;
}
void fillUpPedStatus(LinkedList<ReqEntry> ReqList,int ped[8][2])
{
	if(ReqList.ListEmpty()==0)
	{
		ReqList.Reset();
		for (int i=0;i<8;i++)
		{
			ReqList.Reset();
			while(!ReqList.EndOfList())
			{
				if (ReqList.Data().VehClass==PEDESTRIAN)
				{
					if (ReqList.Data().VehID==i+1)
					{
						ped[i][0]=i+1;
						ped[i][1]=ReqList.Data().iVehicleState;
					}
					
				}
				ReqList.Next();
			}
			
		}
	}
}
