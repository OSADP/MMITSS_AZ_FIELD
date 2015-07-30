/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   

/*  MMITSS_MRP_PriorityRequestServer_field.cpp  
*  Created by Mehdi Zamanipour
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.
*
*  Revision History:
*  1. Revised by Mehdi Zamanipour on 8/4/14 to add the posibility to receive SRM,decode it and populate 
*  Jun's buffer  (RcvMsg). Request_conbimnd txt file is updated. New informaiton such as InLane,OutLane,
*  BeginServiceTime, EndServiceTime and IsCanceld added to each line of request_combined.txt
*  2. PRS can integrate with ISIG (COP) with argument -c 2 and can work with priority+actuation by defaule argument -c 
*  3. PRS is capable to show the ped call in the table but does not change the flag for solver to solve peds
*  4. PRS is now capable of puting cordination request when argument -o 1 is inserted 
*  
*/
   

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sys/time.h>

#include "ReqEntryListHandle.h"
#include "LinkedList.h"
#include "ReqEntry.h"
#include "IntLanePhase.h"

#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>

#include "j2735common.h"
#include "srm.h"
#include "IntLanePhase.h"
#include <libgps.h>

using namespace std;

#define PHASE_STA_TIME2_ASC  "1.3.6.1.4.1.1206.3.5.2.1.18.1.6."  //NEED last "p"  for the phase
#define MAX_ITEMS 20
#define numPhases 8
#define RED 1
#define GREEN 3
#define YELLOW 4
#define MAX_SRM_BUFLEN  197
#define BSM_BLOB_SIZE 38
#ifndef byte
    #define byte  char  // needed if byte not defined
#endif

// for reading ped status from controller 
#define PEDCLEAR_GROUP 	 	"1.3.6.1.4.1.1206.4.2.1.1.4.1.6.1"
#define WALK_GROUP 		 	"1.3.6.1.4.1.1206.4.2.1.1.4.1.7.1"
#define PED_CALL	 		"1.3.6.1.4.1.1206.4.2.1.1.4.1.9.1"
int Ped_Info[3];    //[0]: ped status, [1]: ped call in integer number, [2]: ped clearing
int Ped_Phase_Considered[8]={0};   //Which ped phase need to be considered, If it is 0, there is no ped, if it is 1, there is call, if it is 2 , ped is in walking state, if it is 3, ped in in clearance state
int Check_Ped_Phase_Info();     //Check the current ped status and ped call to get the which ped phase needs to be included when planning COP    (Written by YF modified by MZ)
void Ped_Status_Read();  //The walking state of each ped phase
int savariPedReqFlagForClean[8]={};

// for socket communication with obu
int PORT=  4444; ////***Important*** Port: For receiving request from OBEs
int req_interval=45;  // if a request is not updated for req_interval second in request list, it should be deleted
double dRollingHorInterval=1.0;
char temp_log[256];
char tmp_log[256];  
int filename_cnt=0;
char INTport[8];   //this port is for the controller
// for opening the connection
int sockfd;
struct sockaddr_in sendaddr;
struct sockaddr_in recvaddr;
int iNumBytes=0;
int iAddrLeng=0;
int broadcast=1;
int addr_length;
//Struct for UDP socket timeout: 1s
struct timeval tv;

int PRS_INTERFACE_PORT=44444; // prs send a clear command to interface when all 
int flagForClearingInterfaceCmd=0; // a flag to clear all commands in th interface when the request passed
int ReqListUpdateFlag=0;    // The Flag to identify the ReqList update
LinkedList<ReqEntry> Stop_List;  // Linked list to record the STOPPED vehicles
string RSUID;    // will get from "rsuid.txt"
char pctimestamp[128];
vector<int> v_PhaseNotEnabled;
char INTip[64];// = "150.135.152.23";
char IPInfo[64]			  = "/nojournal/bin/IPInfo.txt";
char predir [64] 		  = "/nojournal/bin/";
char priorityConfiguartionFile[128]="/nojournal/bin/priorityConfiguration.txt"; 
char requestfilename[64]  = "/nojournal/bin/requests.txt";
//*** Add a file to restore the combined requests: requests_combined.txt BY DJ 2012.2.10
//*** The files will be modified and broadcast, and finally will be used by mpr_solver
char requestfilename_combined[64] = "/nojournal/bin/requests_combined.txt";
//This file stores reqeusts with SplitPhase: used for updating the request list.
char requestfilename_update[128]="/nojournal/bin/requests_update.txt";
char ConfigInfo[256]	  = "/nojournal/bin/ConfigInfo.txt";
char rsuid_filename[64] = "/nojournal/bin/rsuid.txt";
char logfilename[256] = "/nojournal/bin/log/MMITSS_PRS_";
char LanePhaseFile[255]="/nojournal/bin/InLane_OutLane_Phase_Mapping.txt";
char ConfigFile[256] ;
int CombinedPhase[8]={0};
uint8_t     SRMbuffer[MAX_SRM_BUFLEN];

//char SRM_buf[MAX_SRM_BUFLEN]=	{0x30,0x46,0x80,0x01,0x0E,0x81,0x01,0x64,0xA2,0x13,0x80,0x02,0x01,0x2D,0x81,0x01,0x0E,0x82,0x01,0x0E,0x83,0x01,0x1C,0x84,0x01,0x1C,0x85,0x01,0x44,0x87,0x26,0x01,0xC0,0xA8,0x01,0x15,0xE6,0x78,0xE3,0xDC,0xCA,0xAC,0x3B,0xAA,0xCE,0xEC,0x76,0x44,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x73,0x16,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x1D,0xA4,0x88,0x01,0x20};// test payload for ped call
//char SRM_buf[MAX_SRM_BUFLEN]=	{0x30,0x46,0x80,0x01,0x0E,0x81,0x01,0x64,0xA2,0x13,0x80,0x02,0x01,0x2D,0x81,0x01,0x0E,0x82,0x01,0x0E,0x83,0x01,0x04,0x84,0x01,0x1C,0x85,0x01,0x44,0x87,0x26,0x01,0xC0,0xA8,0x01,0x15,0xE6,0x78,0xE3,0xDC,0xCA,0xAC,0x3B,0xAA,0xCE,0xEC,0x76,0x44,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x73,0x16,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x1D,0xA4,0x88,0x01,0x20};
char        SRM_buf[MAX_SRM_BUFLEN];
J2735SRM_t  srm;
int         iType;
uint32_t    oob;
// code usage, if the argument is -c 1 , the program will be used with traffic interface (priority and actuation) . if -c 2 as argument, the program work with ISIG
int icodeUsage=1;
int iOnCoordination=0;
int  iCoordPhase[2];
double dCoordinationCycle=100.0;  // will be obtained from priorityConfiguration file
double fcurrentCycleTime=100.0;
double dCoordPhaseSplit=30.0;  // will be obtained from priorityConfiguratio
gps_data_t *gps_handle;
savari_gps_data_t gps;

double GetSeconds();
int  gps_init();
void read_gps ();
void PhaseTimingStatusRead(int PhaseStatus[8]);
int  GetSignalColor(int PhaseStatusNo);
int  outputlog(char *output);
void UpdateList(LinkedList<ReqEntry> &ReqList,char *RcvMsg);
void PrintFile2Log(char *resultsfile);
void xTimeStamp( char * pc_TimeStamp_ );
int  msleep(unsigned long milisec);
void get_rsu_id();
void get_ip_address();
void get_configfile();
void printsrmcsv (J2735SRM_t *srm);  // To print the received SRM
void obtainInLaneOutLane( int srmInLane, int srmOutLane, int &inApproach,int &outApproach,int &iInlane, int &Outlane);
void calculateETA(int beginMin, int beginSec, int endMin, int endSec, int &iETA );
void Pack_Event_List(char* tmp_event_data, int &size);
void setupConnection();
void send_Clear_Commands();  // To clearing the commandds after the request table becomes empty
void send_Ped_Call(int pedReqPhase);  // Used for savari SMARTCROSS project to handle the received SRM from phone!
void send_Clear_PedCall(int ped_phase);

void ReadInCoordinationConfig(char * priorityConfigFile);
//--------We need to start PRS first before solver-------important---------//
int main ( int argc, char* argv[] )
{
 	char cMsgType[32]; // whether it is a request or resuest_clear
 	char cIntersectionName[32]; 
 	char RcvMsg[1024];
    float fETA;//estimated travel time to the approaching signal
	int iRequestedPhase; 
	int iPriorityLevel;
	IntLanePhase lanePhase;
	lanePhase.ReadLanePhaseMap(LanePhaseFile);
	int iInApproach=0;
	int ioutApproach=0;
	int iInLane=0;
	int iOutlane=0;	
	int iETA=0;
	double dMinGrn;
	int	iStartMinute,iStartSecond,iEndMinute,iEndSecond;
	int iStartHour,iEndHour;
	int iVehicleState;
	long lvehicleID;
	int iMsgCnt;
	double dSpeed;
	double	dgpsTime=0.0; 
	int iTotalCurrentReq=0;
	long lTimeOut=499999; // Time out of waiting for a new socket 0.5 second!
	int ret;
	int Ped_Status=0;   //Whether there is a ped call or ped phase is on  0: on call and all phase is off 1: yes
	// coordination parameters
	double dCurrentTime=0.0;
    double fCoordSentTime=-100.0;  // - cycle time !!!
    double fLastReqFileRevisedTime=0.0;
    int iCoordSplitTimeFlag=1;
    int iCoordMsgCont=0;		
	iCoordPhase[0]=2; // be defulat the first coordinated phase is phase 2 and the second is phase 6
    iCoordPhase[1]=6;
    
	addr_length = sizeof ( recvaddr );		
	LinkedList<ReqEntry> Req_List;  // List of all received requests
    int PhaseStatus[8]; //----- Determine the phase status for generate the split phases
	while ((ret = getopt(argc, argv, "p:t:c:o:")) != -1)    // -p  the port that the program receive SRM -t the needed time for updating  the ETA in the vehicle interface (psm) . -c usage of the code 
	{
		switch (ret) 
		{
		case 'p':
			PORT = atoi (optarg);
			printf ("Port is : %d\n", PORT);
			break;
		case 't':
			lTimeOut = atoi (optarg);
			printf ("Time out is : %ld \n", lTimeOut);
			break;
		case 'c':
			icodeUsage = atoi (optarg);
			if (icodeUsage==2)
				printf ("Integrated with I-SIG \n");
			else 
				printf ("PRS is being used for priority and actuation \n");
			break;
		case 'o':
			iOnCoordination = atoi (optarg);
			if (iOnCoordination==1)
				printf ("Coordination priority request is ON \n");
			else 
				printf ("PRS works with NO coordination request \n");
			break;
		default:
			return 0;
		}
	}  
	//Struct for UDP socket timeout: 1s
    
    tv.tv_sec = 0;
    tv.tv_usec = lTimeOut;
    
	dRollingHorInterval=(double) lTimeOut/999999;  // rolling horizon is used for doing countdown on ETA of the received SRM
	//cout<<"dRollingHorInterval"<<dRollingHorInterval<<endl;
	
    //------log file name with Time stamp---------------------------
    //char timestamp[128];
    char temp_log[128];
    xTimeStamp(pctimestamp);
    strcat(logfilename,pctimestamp);
    strcat(logfilename,".log");
    // cout<<logfilename<<endl;
    //------end log file name-----------------------------------//

    fstream ftemp;
    ftemp.open(logfilename,fstream::out);
    if (!ftemp.good())
    {
        perror("Open logfilename failed!"); exit(1);
    }
    else
    {
        ftemp<<"Start Recording PRS at time:\t"<<time(NULL)<<endl;
        ftemp.close();
    }
    //------end log file name-----------------------------------//
    //--- deleting the residual the Results.txt and requests.txt-------//
    system("\\rm /nojournal/bin/requests.txt");
	FILE *fp_req=fopen(requestfilename,"w");
    fprintf(fp_req,"Num_req -1 0\n");
    fclose(fp_req);

    system("\\rm /nojournal/bin/requests_combined.txt");
    fp_req=fopen(requestfilename_combined,"w");
    fprintf(fp_req,"Num_req -1 0\n");
    fclose(fp_req);
	//--- end of deleting the Results.txt and requests.txt-------//

	// ---getting the configurations and setting up gps
	gps_init();		
    get_ip_address();   // Get the ip address of controller
    get_rsu_id();  		// Get the current RSU ID from "rsuid.txt" into string "RSUID"
    get_configfile();   //------- Read the configinfo_XX.txt from ConfigInfo.txt--------//
	ReadInCoordinationConfig(priorityConfiguartionFile); // getting the priority configuration 
	strcpy( cIntersectionName, RSUID.c_str());
    //--------------------------------For the wireless connection--------------------------------------//
    setupConnection();   
    //------------------------------End of the wireless connection----------------------------//

	// geting the current active phase from controller
    PhaseTimingStatusRead(PhaseStatus); // First found out the not used(disabled) phases if there is any.

    for(int i=0;i<8;i++)
    {
        if(PhaseStatus[i]==3)
            v_PhaseNotEnabled.push_back(i+1);
    }
    if(v_PhaseNotEnabled.size()>0)
    {
        v_PhaseNotEnabled.push_back(100); // 100 is a random number, later the v_PhaseNotEnabled will be used to search the unused phases
    }
    
    while ( true )
    {
		// --- Reading the ped status from controller to put the ped call in the table     
   		Ped_Status_Read();
		Ped_Status=Check_Ped_Phase_Info();		
		//~ if (Ped_Status>0) // there is at least one ped call or ped walk or ped clearance
		//~ {
			//~ ReqListFromFile(requestfilename,Req_List);
			//~ UpdateListForPed(Req_List,Ped_Phase_Considered);   // Update the Req List data structure considering ped status
			//~ PrintList2File(requestfilename,Req_List,1);  // Write the requests list into requests.txt, 
			//~ PrintFile2Log(requestfilename);
			//~ PrintList2File(requestfilename_combined,Req_List,0);//Write the requests list into  requests_combined.txt; 
		//~ }
		//~ else // clean the request list and resuest file if there is no ped calls
		//~ {
			//~ if (checkIfThereIsaPedInTable(Req_List)>0)
			//~ {
				//~ PrintList2File(requestfilename,Req_List,1);  // Write the requests list into requests.txt, 
				//~ PrintFile2Log(requestfilename);
				//~ PrintList2File(requestfilename_combined,Req_List,0);//Write the requests list into  requests_combined.txt; 
			//~ }
		//~ } 
		// ---- end of handling the ped calls
		
		cout << "..................Waiting for the request info..................\n";
        // reading gps to get gps time for synchronizing PRS logs with PRG logs
        read_gps();
		//get gps time
		if (isnan(gps.time)==0) 
		    dgpsTime=gps.time;
		else
			dgpsTime=0.0;
			
        //-----Receive a request---------------//
        if ((iNumBytes = recvfrom(sockfd, SRM_buf, sizeof(SRM_buf), 0,(struct sockaddr *)&recvaddr, (socklen_t *)&iAddrLeng)) == -1)
        {
            perror("No Message Received ");        
        }
        
		 if (iOnCoordination==1)
			dCurrentTime=GetSeconds();
			
        iTotalCurrentReq=ReqListFromFile(requestfilename,Req_List); // Read request file and put the request in data structure and return the number of current received requests
        // if iNumBytes is non negative, we have receive messeges and we have to decode it
	 
        if (iNumBytes>-1)
        {
			// decoding the received SRM
			for (unsigned int i=0;i<sizeof(SRM_buf);i++)
			{
				SRMbuffer[i]=SRM_buf[i];
				//cout<<SRMbuffer[i];
			}
			if (j2735_decode_srm (&srm, iType, SRMbuffer, sizeof(SRMbuffer),  &oob) < 0) 
			{
				sprintf(temp_log, "SRM: Decode Failure\n");
				outputlog(temp_log); 
				cout<<temp_log;
			} else {
				sprintf(temp_log, "SRM: Decode Success\n");
				outputlog(temp_log); 
				cout<<temp_log;
			}
			
			
			//j2735_dump_hex("SRM dump after decoding:", SRMbuffer, sizeof(SRMbuffer));
			//printsrmcsv(&srm);
	
			if (lanePhase.iIntersectionID==srm.intersection_id)  // if the intersection ID in SRM matches the MAP ID of the intersection, SRM should be processed
			{
				//printf("TYPE %d class%d",srm.vehicleclass_type,srm.vehicleclass_level  );
				//printf("INLANE %d",	srm.in_lanenum);
				if (srm.vehicleclass_type==4) // if the received message is a pedestrian request!!  
				{
					int iPedRequestedPhase = srm.in_lanenum ;
					send_Ped_Call(iPedRequestedPhase);
					//cout<<"Ped Request is being sent  for phase " << iPedRequestedPhase  <<endl;
				}
				else
				{
					if (srm.cancelreq_priority==0)
						strcpy( cMsgType, "request");
					else
						strcpy( cMsgType, "request_clear");
						  
					
					obtainInLaneOutLane( srm.in_lanenum , srm.out_lanenum, iInApproach, ioutApproach, iInLane, iOutlane); 
					iRequestedPhase=lanePhase.iInLaneOutLanePhase[iInApproach][iInLane][ioutApproach][iOutlane];  
					iPriorityLevel=srm.vehicleclass_type ; // is this the currec element of SRM to populate with vehicle  type?!
					dMinGrn=srm.yawrate; 			 // there was no place in SMR to store MinGrn , so I stored it in vehicle yawrate!!!!!
					iStartMinute=srm.starttime_min;
					iStartSecond=srm.starttime_sec;
					iEndMinute=srm.endtime_min;
					iEndSecond=srm.endtime_sec;
					iStartHour=srm.starttime_hour;
					iEndHour=srm.endtime_hour;
					calculateETA(iStartMinute,iStartSecond,iEndMinute,iEndSecond,iETA );
					fETA=(float) iETA;
					lvehicleID=srm.temp_ident_id;
					iVehicleState=srm.vehicle_status;
					iMsgCnt=srm.msgcount;
					dSpeed=srm.speed;
					//printsrmcsv (&srm);
					sprintf(RcvMsg,"%s %s %ld %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d", cMsgType, cIntersectionName, lvehicleID, iPriorityLevel,fETA,iRequestedPhase,dMinGrn, time(NULL), 
					srm.in_lanenum , srm.out_lanenum , iStartHour, iStartMinute, iStartSecond, iEndHour,iEndMinute, iEndSecond, iVehicleState, iMsgCnt);
					sprintf(temp_log,"\n******************  SRM  Received ****************** At GPS time %f. \n",dgpsTime); outputlog(temp_log); cout<< temp_log<< endl;
					sprintf(temp_log,"\n ID, Type, ETA , Phase, QCT, inLane, outLane, Shr, Smn, Ssec, Ehr, Emn, Esec, State, Speed, Cnt  \n");outputlog(temp_log); cout<< temp_log<< endl;
					sprintf(temp_log,"{%s}\t \n",RcvMsg); outputlog(temp_log); cout<< temp_log<< endl;			

					PhaseTimingStatusRead(PhaseStatus);  // Get the current phase status for determining the split phases
					sprintf(temp_log,"-----At gps time: %lf ********Current Signal status:************\n",dgpsTime);
					outputlog(temp_log);
					sprintf(temp_log,"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",PhaseStatus[0],PhaseStatus[1],PhaseStatus[2],PhaseStatus[3],
						PhaseStatus[4],PhaseStatus[5],PhaseStatus[6],PhaseStatus[7]);
					outputlog(temp_log);
					
					ReqListFromFile(requestfilename,Req_List);
					UpdateList(Req_List,RcvMsg,PhaseStatus);   // Update the Req List data structure considering received message
					PrintList2File(requestfilename,Req_List,1);  // Write the requests list into requests.txt, 
					PrintFile2Log(requestfilename);
					PrintList2File(requestfilename_combined,Req_List,0);//Write the requests list into  requests_combined.txt; 
					PrintFile2Log(requestfilename_combined);
					cout<<"Request List Update Flag  "<<ReqListUpdateFlag<<endl;
				}
			}
		}
		else if ( (iTotalCurrentReq>0 && iOnCoordination==0 ) || ( (dCurrentTime-fLastReqFileRevisedTime)>dRollingHorInterval && iTotalCurrentReq>0 && iOnCoordination==1) ) // no new message received but there are still some requests in the request list, their ETA should be updated
		{
			cout<<"Current time: "<< dCurrentTime << endl;
			cout<<"Current cycle time: "<< fcurrentCycleTime << endl;
			fLastReqFileRevisedTime=dCurrentTime;
			ReqListUpdateFlag=0;
			UpdateCurrentList(Req_List);
			PrintList2File(requestfilename,Req_List,1);  // Write the requests list into requests.txt, 
			PrintFile2Log(requestfilename);
			PrintList2File(requestfilename_combined,Req_List,0);//Write the requests list into  requests_combined.txt; This file will be different than requests.txt when we have EV
			PrintFile2Log(requestfilename_combined);
		}
		
		// ---- handeling coordination request 
		// --- Setting up the coordination requests -----
		if (iOnCoordination==1)
		{
			PhaseTimingStatusRead(PhaseStatus);  // Get the current phase for setting the coordination request
			
			if (fcurrentCycleTime >= dCoordinationCycle) //( (PhaseStatus[1]==7) && (PhaseStatus[5]==7) && (fcurrentCycleTime >= dCoordinationCycle) )
			{
				iCoordSplitTimeFlag=0;
				fCoordSentTime=dCurrentTime;
				ReqListUpdateFlag=6;
			}
			fcurrentCycleTime=dCurrentTime-fCoordSentTime;
			// at the master clock we should put the coordination request for current cycle. 
			//at the end of the coordination phase split we should cancel the current cycle request and put the next cycle coordination request						
			if ( (fcurrentCycleTime>=dCoordPhaseSplit) && (iCoordSplitTimeFlag==0)) //  if ( (fcurrentCycleTime>=dCoordPhaseSplit-4) && (iCoordSplitTimeFlag==0)) // !!!!!!! (End of Split)  sent 4second  eailier to let solver and cop solve the problem
			{
				ReqListUpdateFlag=6; 
				iCoordSplitTimeFlag=1;
			}
			
			if (ReqListUpdateFlag==6)
			{
				strcpy( cMsgType, "coord_request");
				if (fcurrentCycleTime<=2)
				{
					dMinGrn=dCoordPhaseSplit;
					fETA=0.0;
				}else
				{
					dMinGrn=0.0;
					fETA=dCoordinationCycle-dCoordPhaseSplit ;   // 
				}
				
				iCoordMsgCont=(iCoordMsgCont+10)%127;
				// ///// ***** IMPORTANT **** Coordination priotiy type is set to be 6 , AND the request  ID is 0 	
				sprintf(RcvMsg,"%s %s %d %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d", cMsgType, cIntersectionName, 0, 6 ,fETA,  iCoordPhase[0]  ,dMinGrn, time(NULL),	0 , 0 , 0, 0, 0, 0,0, 0, 1, iCoordMsgCont);
				//sprintf(temp_log,"\n******************  Coordination Request Is Set ****************** At GPS time %ld. \n",time(NULL)); 
				sprintf(temp_log,"\n******************  Coordination Request Is Set ****************** At simulation time %f. \n",dCurrentTime ); 
				outputlog(temp_log); 
				cout<< temp_log<< endl;
				sprintf(temp_log,"{%s}\t \n",RcvMsg); 
				outputlog(temp_log); 
				cout<< temp_log<< endl;		
				ReqListFromFile(requestfilename,Req_List);
				ReqListUpdateFlag=6;
				UpdateList(Req_List,RcvMsg,PhaseStatus);   // Update the Req List data structure considering first coordination , AND the request ID is 9999
				sprintf(RcvMsg,"%s %s %d %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d", cMsgType, cIntersectionName, 99999, 6 ,fETA,  iCoordPhase[1] ,dMinGrn, time(NULL),	0 , 0 , 0, 0, 0, 0,0, 0, 2, iCoordMsgCont); // the id for the second ring coordination request is set to 99999
				//sprintf(temp_log,"\n******************  Coordination Request Is Set ****************** At GPS time %ld. \n",time(NULL)); 
				sprintf(temp_log,"{%s}\t \n",RcvMsg); 
				outputlog(temp_log); 
				cout<< temp_log<< endl;		
				UpdateList(Req_List,RcvMsg,PhaseStatus);   // Update the Req List data structure considering first coordination 
				PrintList2File(requestfilename,Req_List,1);  // Write the requests list into requests.txt, 
				PrintFile2Log(requestfilename);
				PrintList2File(requestfilename_combined,Req_List,0);//Write the requests list into  requests_combined.txt; 
				PrintFile2Log(requestfilename_combined);
				ReqListUpdateFlag=0;
			}
		}
		
        if (ReqListUpdateFlag>0 && Req_List.ListSize()>0 ) // Request list changed and need to solve
        {
            sprintf(temp_log,"---At time: %ld. ********Need to solve.************\n",time(NULL));  
            cout<<temp_log<<endl; 
            outputlog(temp_log);
            //----- ReqListUpdateFlag will be set to "0" after solving the problem in rsu_mprsolver
            UpdateCurrentList(Req_List);
            PrintList2File(requestfilename,Req_List,1);
            PrintList2File(requestfilename_combined,Req_List,0);
  
            
        }
        if ( (ReqListUpdateFlag>0 && Req_List.ListSize()==0 ) ||flagForClearingInterfaceCmd==1) // Request list is empty and the last vehisle just passed the intersection 
        {
			ReqListUpdateFlag=0;
			flagForClearingInterfaceCmd=0;
			UpdateCurrentList(Req_List);
			PrintList2File(requestfilename,Req_List,1);  // Write the requests list into requests.txt, 
			PrintFile2Log(requestfilename);
			PrintList2File(requestfilename_combined,Req_List,0);//Write the requests list into  requests_combined.txt; This file will be different than requests.txt when we have EV
			PrintFile2Log(requestfilename_combined);
			// in case the program is being used for actuaion and prioirity this part of code send a hold command to interface for all phases 
			//when ever the last request pass the intersection to delete previous command
			if (icodeUsage==1) 
			{
				send_Clear_Commands();
			}
		}
   }
    return 0;  

} // End of main();


//========================================================================================//

int outputlog(char *output)
{
 

    FILE *stream = fopen(logfilename, "r");
    if (stream==NULL)
    {
        perror ("Error opening file");
    }
    fseek( stream, 0L, SEEK_END );
    long endPos = ftell( stream );
    fclose( stream );

    //cout<<"The position is:  "<<endPos<<endl;

    fstream fs;
    if (endPos <50000000)
        fs.open(logfilename, ios::out | ios::app);
    else
        fs.open(logfilename, ios::out | ios::trunc);

    if (!fs || !fs.good())
    {
        cout << "could not open file!\n";
        return -1;
    }

    fs << output;

    if (fs.fail())
    {
        cout << "failed to append to file!\n";
        return -1;
    }

    fs.close();
    return 1;
}




void xTimeStamp( char * pc_TimeStamp_ )
{
    struct tm  * ps_Time;
    time_t       i_CurrentTime;
    char         ac_TmpStr[256];

    i_CurrentTime =  time(NULL);
    ps_Time = localtime( &i_CurrentTime );

    //year
    sprintf(ac_TmpStr, "%d", ps_Time->tm_year + 1900);
    strcpy(pc_TimeStamp_, ac_TmpStr);

    //month
    sprintf(ac_TmpStr, "_%d", ps_Time->tm_mon + 1 );
    strcat(pc_TimeStamp_, ac_TmpStr);

    //day
    sprintf(ac_TmpStr, "_%d", ps_Time->tm_mday );
    strcat(pc_TimeStamp_, ac_TmpStr);

    //hour
    sprintf(ac_TmpStr, "_%d", ps_Time->tm_hour  );
    strcat(pc_TimeStamp_, ac_TmpStr);

    //min
    sprintf(ac_TmpStr, "_%d", ps_Time->tm_min );
    strcat(pc_TimeStamp_, ac_TmpStr);

    //sec
    sprintf(ac_TmpStr, "_%d", ps_Time->tm_sec );
    strcat(pc_TimeStamp_, ac_TmpStr);
}


int msleep(unsigned long milisec)
{
    struct timespec req={0};
    time_t sec=(int)(milisec/1000);
    milisec=milisec-(sec*1000);
    req.tv_sec=sec;
    req.tv_nsec=milisec*1000000L;
    while(nanosleep(&req,&req)==-1)
        continue;
    return 1;
}

void get_rsu_id() //DJ add 06/24/2011
{
    fstream fs;
    fs.open(rsuid_filename);

    char temp[128];

    getline(fs,RSUID);

    if(RSUID.size()!=0)
    {
        //cout<< "Current Vehicle ID:" << RSUID <<endl;
        sprintf(temp," RSU ID %s\n",RSUID.c_str());
        cout<<temp<<endl;
        outputlog(temp);
    }
    else
    {
        sprintf(temp,"Reading RSU ID problem.\n");
        cout<<temp<<endl;
        outputlog(temp);
        exit(0);
    }

    fs.close();
}

void get_configfile()
{
    fstream fs;
    fstream fs_phase; //*** Read in all phases in order to find the combined phase information.***//
    fs.open(ConfigInfo);

    string lineread;	getline(fs,lineread);

    if(lineread.size()!=0)
    {
        //std::cout<< "Current Vehicle ID:" << RSUID <<std::endl;
        sprintf(ConfigFile,"%s",lineread.c_str());

        cout<<ConfigFile<<endl; outputlog(ConfigFile);

        int phase_num;

        fs_phase.open(ConfigFile);

        getline(fs_phase,lineread); //*** Read the first line to get the number of all phases.
        sscanf(lineread.c_str(),"%*s %d ",&phase_num);

        getline(fs_phase,lineread); //*** Read the second line of the combined phase into array CombinedPhase[8]
        //*** If the phase exsits, the value is not 0; if not exists, the default value is '0'.
        sscanf(lineread.c_str(),"%*s %d %d %d %d %d %d %d %d",
            &CombinedPhase[0],&CombinedPhase[1],&CombinedPhase[2],&CombinedPhase[3],
            &CombinedPhase[4],&CombinedPhase[5],&CombinedPhase[6],&CombinedPhase[7]);

        fs_phase.close();
    }
    else
    {
        sprintf(temp_log,"Reading configure file %s problem",ConfigInfo);
        cout<<temp_log<<endl; outputlog(temp_log);
        exit(0);
    }

    fs.close();
}

int GetSignalColor(int PhaseStatusNo)
{
    int ColorValue=RED;

    switch (PhaseStatusNo)
    {
    case 2:
    case 3:
    case 4:
    case 5:
        ColorValue=RED;
        break;
    case 6:
    case 11:
        ColorValue=YELLOW;
        break;
    case 7:
    case 8:
        ColorValue=GREEN;
        break;
    default:
        ColorValue=0;
    }
    return ColorValue;
}



void PhaseTimingStatusRead(int PhaseStatus[8])
{
    netsnmp_session session, *ss;
    netsnmp_pdu *pdu;
    netsnmp_pdu *response;
    oid anOID[MAX_OID_LEN];
    size_t anOID_len;
    netsnmp_variable_list *vars;
    int status;
    init_snmp("ASC");   //Initialize the SNMP library
    snmp_sess_init( &session );  //Initialize a "session" that defines who we're going to talk to
    /* set up defaults */
    char ipwithport[64];
    strcpy(ipwithport,INTip);
    strcat(ipwithport,":");
    strcat(ipwithport,INTport); //for ASC get status, DO NOT USE port!!!
    session.peername = strdup(ipwithport);
    session.version = SNMP_VERSION_1; //for ASC intersection  /* set the SNMP version number */
    /* set the SNMPv1 community name used for authentication */
    session.community = (u_char *)"public";
    session.community_len = strlen((const char *)session.community);
    SOCK_STARTUP;
    ss = snmp_open(&session);                     /* establish the session */
    if (!ss)
    {
        snmp_sess_perror("ASC", &session);
        SOCK_CLEANUP;
        exit(1);
    }

    /*
    * Create the PDU for the data for our request.
    *   1) We're going to GET the system.sysDescr.0 node.
    */
    pdu = snmp_pdu_create(SNMP_MSG_GET);
    anOID_len = MAX_OID_LEN;
    //---#define CUR_TIMING_PLAN     "1.3.6.1.4.1.1206.3.5.2.1.22.0"      // return the current timing plan
    char ctemp[50];
    for(int i=1;i<=8;i++)
    {
        sprintf(ctemp,"%s%d",PHASE_STA_TIME2_ASC,i);
        if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit as enabled or not: "1"  enable; "0" not used
        {
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            exit(1);
        }
        snmp_add_null_var(pdu, anOID, anOID_len);
    }
    /*
    * Send the Request out.
    */
    status = snmp_synch_response(ss, pdu, &response);
    /*
    * Process the response.
    */
    if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
    {
        /*
        * SUCCESS: Print the result variables
        */
        int *out = new int[MAX_ITEMS];
        int i =0;
        //~ for(vars = response->variables; vars; vars = vars->next_variable)
            //~ print_variable(vars->name, vars->name_length, vars);
        /* manipuate the information ourselves */
        for(vars = response->variables; vars; vars = vars->next_variable)
        {
            if (vars->type == ASN_OCTET_STR)
            {
                char *sp = (char *)malloc(1 + vars->val_len);
                memcpy(sp, vars->val.string, vars->val_len);
                sp[vars->val_len] = '\0';
                //printf("value #%d is a string: %s\n", count++, sp);
                free(sp);
            }
            else
            {

                int *aa;
                aa =(int *)vars->val.integer;
                out[i++] = * aa;
                //printf("value #%d is NOT a string! Ack!. Value = %d \n", count++,*aa);
            }
        }
        //****** GET the results from controller *************//
        for(int i=0;i<8;i++)
        {
            PhaseStatus[i]=out[i];
            //PhaseStatus[i]=GetSignalColor(out[i]);// DO not Convert
        }
    }
    else
    {
        if (status == STAT_SUCCESS)
            fprintf(stderr, "Error in packet\nReason: %s\n",
            snmp_errstring(response->errstat));
        else if (status == STAT_TIMEOUT)
            fprintf(stderr, "Timeout: No response from %s.\n",
            session.peername);
        else
            snmp_sess_perror("snmpdemoapp", ss);

    }
    /*
    * Clean up:    *  1) free the response.   *  2) close the session.
    */
    if (response)        snmp_free_pdu(response);
    snmp_close(ss);
    SOCK_CLEANUP;
}


void get_ip_address()
{
    fstream fs;	fs.open(IPInfo);
    string lineread; 	
    getline(fs,lineread);

    if(lineread.size()!=0)
    {
        sprintf(INTip,"%s",lineread.c_str());
        cout<< " Controller IP: " <<INTip<<endl;
        getline(fs,lineread);
        sprintf(INTport,"%s",lineread.c_str());
        cout<< " Controller Port: " <<INTport<<endl;
    }
    else
    {
        cout<<"A problem in reading IPinfo file problem"<<endl;
        exit(0);
    }

    fs.close();
}

void PrintFile2Log(char *resultsfile)
{
    fstream fss;
    fss.open(resultsfile,fstream::in);

    if (!fss)
    {
        cout<<"***********Error opening the plan file in order to print to a log file!\n";
        sprintf(tmp_log,"***********Error opening the plan file in order to print to a log file!\n");
        outputlog(tmp_log);
        exit(1);
    }
    string lineread;

    while(!fss.eof())
    {
        getline(fss,lineread);
        strcpy(tmp_log,lineread.c_str());
        strcat(tmp_log,"\n"); outputlog(tmp_log);
    }

    fss.close();

}



// Mehdi Added

void obtainInLaneOutLane( int srmInLane, int srmOutLane, int & inApproach, int &outApproach, int &iInlane, int &Outlane)
{
	
	inApproach=(int) (srmInLane/10);
	iInlane=srmInLane-inApproach*10;
	outApproach=(int) (srmOutLane/10);
	Outlane=srmOutLane-outApproach*10;
	
	
}


void calculateETA(int beginMin, int beginSec, int endMin, int endSec, int &iETA )
{
	
	 if ((beginMin ==59) && (endMin == 0)) // 
	 	 iETA= (60-beginSec)+endSec;
	 if ((beginMin ==59) && (endMin == 1)) // 
	 	 iETA= (60-beginSec)+endSec+60;
 	 if ((beginMin ==58) && (endMin == 0)) // 
	 	 iETA= (60-beginSec)+endSec+60;
	 if (endMin - beginMin  ==0)
	 	 iETA= endSec-beginSec;
	 else if ( endMin - beginMin  ==1)
	 	 iETA=(60-beginSec)+endSec;
	 else if ( endMin - beginMin ==2)
	 	 iETA=(60-beginSec)+endSec+60;
}





void printsrmcsv (J2735SRM_t *srm) 
{
printf("Msg ID :%2x\n", srm->message_id);                                                             
printf("Msg Count :%2x\n", srm->msgcount);                                                            
printf("Intersection ID %x\n", srm->intersection_id);                                                 
printf("Cancel Priority %x Cancel Preemption %x\n", srm->cancelreq_priority, srm->cancelreq_preemption);                                                                           
printf("Signal  Preempt %x Priority %x \n", srm->signalreq_priority, srm->signalreq_preemption);                                              
printf("Lane Num  IN %x OUT %x \n", srm->in_lanenum, srm->out_lanenum);                      
printf("Veh Type %x  Veh Class %x \n", srm->vehicleclass_type, srm->vehicleclass_level);                                        
printf("Code word %s\n", srm->code_word);                                                    
printf("Start Time %02d:%02d:%02d -> End Time %02d:%02d:%02d\n", srm->starttime_hour,srm->starttime_min,srm->starttime_sec, srm->endtime_hour, srm->endtime_min, srm->endtime_sec);                                                                           
printf("Transit status %x\n", srm->transitstatus);                                               
printf("Vehicle Name %s\n", srm->vehicle_name);                                                  
printf("Vin %s\n", srm->vin);                                                                    
printf("Vehicle owner code %s\n", srm->vehicle_ownercode);                                       
printf("Ident temp id %llx\n", srm->temp_ident_id);                                              
printf("Vehicle type %d\n", srm->vehicle_type);                                                  
printf("Class %x\n", srm->vehicle_class);                                                        
printf("Group type %d\n", srm->vehicle_grouptype);                                               

printf("Temp ID :%llx\n", srm->temp_id);                                                         
printf("SecMark :%x\n", srm->dsecond);                                                           
printf("Latitude %8.2f\n", srm->latitude);                                                       
printf("Longitude %8.2f\n", srm->longitude);                                                     
printf("Elevation %8.2f\n", srm->elevation);                                                     
printf("Pos 0 %8.2f\n", srm->positionalaccuracy[0]);                                             
printf("Pos 1 %8.2f\n", srm->positionalaccuracy[1]);

printf("Pos 2 %8.2f\n", srm->positionalaccuracy[2]);        
printf("TState %lf\n", srm->transmissionstate);                                                            
printf("Speed %8.2f\n", srm->speed);                                                                       
printf("Heading %8.2f\n", srm->heading);                                                                   
printf("Angle %8.2lf\n", srm->angle);                                                                      
printf("longAccel %8.2f\n", srm->longaccel);                                                               
printf("LatAccel %8.2f\n", srm->lataccel);                                                                 
printf("VertAccel %8.2f\n", srm->vertaccel);                                                               
printf("YawRate %8.2f\n", srm->yawrate);                                                                   
printf("Wheel Brake %x\n", srm->wheelbrake);                                                               
printf("Wheel Brake AV %x\n", srm->wheelbrakeavailable);                                                   
printf("SpareBit %x\n", srm->sparebit);                                                                    
printf("Traction %x\n", srm->traction);                                                                    
printf("ABS %x\n", srm->abs);                                                                              
printf("Stab Contl %x\n", srm->stabilitycontrol);                                                          
printf("BrakeBoost %x\n", srm->brakeboost);                                                                
printf("Aux Brakes %x\n", srm->auxbrakes);                                                                 
printf("Width %8.2f\n", srm->vehicle_width);                                                               
printf("Length %8.2f\n", srm->vehicle_length);                                                             


printf("*******srm PART1*********\n");                                                                     
printf("Vehicle status %x\n", srm->vehicle_status);     


}


int gps_init() 
{
    int is_async = 0;
    int fd;
    gps_handle = savari_gps_open(&fd, is_async);
    if (gps_handle == 0) {
        printf("sorry no gpsd running\n");
        return -1;
    }
   return 0;
}
 
 
 
 
 
void read_gps () 
{
    savari_gps_read (&gps, gps_handle);
    /* printgpscsv (); */
 
}


void setupConnection()
{
	 if((sockfd = socket(PF_INET,SOCK_DGRAM,0)) == -1)
    {
        perror("sockfd");
        exit(1);
    }
    
    //Setup time out
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0)  
    {
		      perror("Error");
    }
    if((setsockopt(sockfd,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof broadcast)) == -1)
    {
        perror("setsockopt - SO_SOCKET ");
        exit(1);
    }
    recvaddr.sin_family = AF_INET;
    recvaddr.sin_port = htons(PORT);
    recvaddr.sin_addr.s_addr = INADDR_ANY;
    memset(recvaddr.sin_zero,'\0',sizeof recvaddr.sin_zero);
    if(bind(sockfd, (struct sockaddr*) &recvaddr, sizeof recvaddr) == -1)
    {
        perror("bind");
        exit(1);
    }
    //addr_len = sizeof  sendaddr ;
     
	sendaddr.sin_family = AF_INET;
	sendaddr.sin_port = htons(PRS_INTERFACE_PORT);//htons(PortOfInterface);   // PRS sends a mesage to traffic control interface to delete all the commands when ther is no request in the request table
	sendaddr.sin_addr.s_addr = inet_addr("127.0.0.1") ; //INADDR_BROADCAST;
	memset(sendaddr.sin_zero,'\0',sizeof sendaddr.sin_zero);

}


void Pack_Event_List(char* tmp_event_data, int &size)
{
	int offset=0;
	byte*   pByte;      // pointer used (by cast)to get at each byte 
                            // of the shorts, longs, and blobs
	unsigned short   tempUShort;
    long    tempLong;
    //header 2 bytes
	tmp_event_data[offset]=0xFF;
	offset+=1;
	tmp_event_data[offset]=0xFF;
	offset+=1;
	//MSG ID: 0x03 for signal event data send to Signal Control Interface
	tmp_event_data[offset]=0x03;
	offset+=1;
	//No. events in R1
	int numberOfPhase=4;
	int tempTime=0;
	int tempCmd=3;
	tempUShort = (unsigned short)numberOfPhase;
	pByte = (byte* ) &tempUShort;
    tmp_event_data[offset+0] = (byte) *(pByte + 1); 
    tmp_event_data[offset+1] = (byte) *(pByte + 0); 
	offset = offset + 2;
	//Events in R1
	for (int iii=0;iii<4;iii++)
	{
		//Time 
		tempLong = (long)(tempTime); 
		pByte = (byte* ) &tempLong;
		tmp_event_data[offset+0] = (byte) *(pByte + 3); 
		tmp_event_data[offset+1] = (byte) *(pByte + 2); 
		tmp_event_data[offset+2] = (byte) *(pByte + 1); 
		tmp_event_data[offset+3] = (byte) *(pByte + 0); 
		offset = offset + 4;
		//phase
		tempUShort = (unsigned short) iii+1 ;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//action
		tempUShort = (unsigned short)tempCmd;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
	}

	tempUShort = (unsigned short)numberOfPhase;
	pByte = (byte* ) &tempUShort;
    tmp_event_data[offset+0] = (byte) *(pByte + 1); 
    tmp_event_data[offset+1] = (byte) *(pByte + 0); 
	//Events in R
	offset = offset + 2;
	for (int iii=0;iii<4;iii++)
	{
		//Time 
		tempLong = (long)(tempTime); 
		pByte = (byte* ) &tempLong;
		tmp_event_data[offset+0] = (byte) *(pByte + 3); 
		tmp_event_data[offset+1] = (byte) *(pByte + 2); 
		tmp_event_data[offset+2] = (byte) *(pByte + 1); 
		tmp_event_data[offset+3] = (byte) *(pByte + 0); 
		offset = offset + 4;
		//phase
		tempUShort = (unsigned short) iii+5 ;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//action
		tempUShort = (unsigned short)tempCmd;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
	}
	size=offset;
}


void send_Ped_Call(int pedReqPhase)    
{
	if  (pedReqPhase==28)
		pedReqPhase=2;
	else if (pedReqPhase==20)
		pedReqPhase=8;
	else if (pedReqPhase==4)
		pedReqPhase=4;
	else if (pedReqPhase==12)
		pedReqPhase=6;
	else
		pedReqPhase=1;
	savariPedReqFlagForClean[pedReqPhase-1]=1;
	byte tmp_event_data[500];
	int size=0;
	//Pack_Event_List(tmp_event_data, size);
	int offset=0;
	byte*   pByte;      // pointer used (by cast)to get at each byte 
                           // of the shorts, longs, and blobs
	unsigned short   tempUShort;
    long    tempLong;
    //header 2 bytes
	tmp_event_data[offset]=0xFF;
	offset+=1;
	tmp_event_data[offset]=0xFF;
	offset+=1;
	//MSG ID: 0x03 for signal event data send to Signal Control Interface
	tmp_event_data[offset]=0x03;
	offset+=1;
	
	int numberOfPhase;
	int tempTime=0;
	int tempCmd=4; // Traffic Interface will consider 4 as PED_CALL
	
	// if the ped call is for phases in the first ring
	if (pedReqPhase==2 || pedReqPhase ==4)
	{
		//number of the phases in first ring
		numberOfPhase=1;
		tempUShort = (unsigned short)numberOfPhase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Time 
		tempLong = (long)(tempTime); 
		pByte = (byte* ) &tempLong;
		tmp_event_data[offset+0] = (byte) *(pByte + 3); 
		tmp_event_data[offset+1] = (byte) *(pByte + 2); 
		tmp_event_data[offset+2] = (byte) *(pByte + 1); 
		tmp_event_data[offset+3] = (byte) *(pByte + 0); 
		offset = offset + 4;
		//phase
		tempUShort = (unsigned short) pedReqPhase ;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//action
		tempUShort = (unsigned short)tempCmd;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2; 
		//number of the phases in second ring
		numberOfPhase=0;
		tempUShort = (unsigned short)numberOfPhase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		//Events in R
		offset = offset + 2;
	}
	
	// if the ped call is for phases in the second ring
	if (pedReqPhase==6 || pedReqPhase ==8)
	{
		//number of the phases in first ring
		numberOfPhase=0;
		tempUShort = (unsigned short)numberOfPhase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		//Events in R
		offset = offset + 2;
		
		numberOfPhase=1;
		//number of the phases in second ring
		tempUShort = (unsigned short)numberOfPhase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Time 
		tempLong = (long)(tempTime); 
		pByte = (byte* ) &tempLong;
		tmp_event_data[offset+0] = (byte) *(pByte + 3); 
		tmp_event_data[offset+1] = (byte) *(pByte + 2); 
		tmp_event_data[offset+2] = (byte) *(pByte + 1); 
		tmp_event_data[offset+3] = (byte) *(pByte + 0); 
		offset = offset + 4;
		//phase
		tempUShort = (unsigned short) pedReqPhase ;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//action
		tempUShort = (unsigned short)tempCmd;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2; 
		
		numberOfPhase=0;
		tempUShort = (unsigned short)numberOfPhase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		//Events in R
		offset = offset + 2;
	}
	size=offset;
	char* event_data;
	event_data= new char[size];			
	for(int i=0;i<size;i++)
	{
			event_data[i]=tmp_event_data[i];	
			
	}
	if (sendto(sockfd,event_data,size+1 , 0,(struct sockaddr *)&sendaddr, addr_length))
	{
		sprintf(temp_log," PED CALL SENT TO SignalControllerInterface FOR PHASE %d  \n", pedReqPhase); 
		outputlog(temp_log);
		cout<< temp_log<< endl;
	}
}


void send_Clear_Commands()
{
	byte tmp_event_data[500];
	int size=0;
	Pack_Event_List(tmp_event_data, size);
	char* event_data;
	event_data= new char[size];			
	for(int i=0;i<size;i++)
		event_data[i]=tmp_event_data[i];	
	if (sendto(sockfd,event_data,size+1 , 0,(struct sockaddr *)&sendaddr, addr_length))
	{
		sprintf(temp_log," The Event List sent to SignalControllerInterface to delete all previous commands, The size is %d  \n", size); 
		outputlog(temp_log);
		cout<< temp_log<< endl;
	}
}




int Check_Ped_Phase_Info()  // The output of this function shows there is either a ped call, ped is in walikong state, or ped is in clearing state
{
	int i;
	int Ped_Phase_on[8]={0};
	int Ped_Phase_call[8]={0};
	int Ped_Clear_on[8]={0};
	int ped_status=0;	
	if(Ped_Info[0]!=0 ||Ped_Info[1]!=0 ||Ped_Info[2]!=0)
		ped_status=1;
	for(i=7;i>=0;i--)
	{
		if(Ped_Info[0]-pow(2.0,i*1.0)>=0)
		{
			Ped_Phase_on[i]=1;
			Ped_Info[0]-= (int) pow(2.0,i*1.0);
		}
	}
	for(i=7;i>=0;i--)
	{
		if(Ped_Info[1]-pow(2.0,i*1.0)>=0)
		{
			Ped_Phase_call[i]=1;
			Ped_Info[1]-=  (int) pow(2.0,i*1.0);
		}
	}
	for(i=7;i>=0;i--)
	{
		if(Ped_Info[2]-pow(2.0,i*1.0)>=0)
		{
			Ped_Clear_on[i]=1;
			Ped_Info[2]-= (int) pow(2.0,i*1.0);
		}
	}
	for(i=0;i<8;i++)
	{
		if( Ped_Phase_call[i]!=0)
			Ped_Phase_Considered[i]=1;
		else if (Ped_Phase_on[i]!=0)
			Ped_Phase_Considered[i]=2;
		else if (Ped_Clear_on[i]!=0)
			Ped_Phase_Considered[i]=3;	
		else
			Ped_Phase_Considered[i]=0;	 	
	}
	cout<<"Ped_Status is: "<<ped_status<<endl;
	for (int kk=0;kk<8;kk++)
	{
		 if (Ped_Phase_Considered[kk] >0)
			cout<<"PED state is "<<Ped_Phase_Considered[kk]<<" and the phase is " << kk+1<<endl;
	}
	
	
	// check if we should clear the previous ped call from Savari ped app
	for (i=0;i<8;i++)
	{
		if (Ped_Phase_on[i]==1 && savariPedReqFlagForClean[i]==1) // means if the ped status is on ped walking time and a ped resuest from ped app is being receiveid, then send a clear command t0o controller
		{
			send_Clear_PedCall(i+1);
			savariPedReqFlagForClean[i]=0;			
		}
	}

	
	
	return ped_status;
}



void send_Clear_PedCall(int ped_phase)
{
	byte tmp_event_data[500];
	int size=0;
	//Pack_Event_List(tmp_event_data, size);
	int offset=0;
	byte*   pByte;      // pointer used (by cast)to get at each byte 
                           // of the shorts, longs, and blobs
	unsigned short   tempUShort;
    long    tempLong;
    //header 2 bytes
	tmp_event_data[offset]=0xFF;
	offset+=1;
	tmp_event_data[offset]=0xFF;
	offset+=1;
	//MSG ID: 0x03 for signal event data send to Signal Control Interface
	tmp_event_data[offset]=0x03;
	offset+=1;
	
	int numberOfPhase;
	int tempTime=0;
	int tempCmd=5; // Traffic Interface will consider 5 as PED_Clear
	
	// if the ped call is for phases in the first ring
	if (ped_phase==2 || ped_phase ==4)
	{
		//number of the phases in first ring
		numberOfPhase=1;
		tempUShort = (unsigned short)numberOfPhase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Time 
		tempLong = (long)(tempTime); 
		pByte = (byte* ) &tempLong;
		tmp_event_data[offset+0] = (byte) *(pByte + 3); 
		tmp_event_data[offset+1] = (byte) *(pByte + 2); 
		tmp_event_data[offset+2] = (byte) *(pByte + 1); 
		tmp_event_data[offset+3] = (byte) *(pByte + 0); 
		offset = offset + 4;
		//phase
		tempUShort = (unsigned short) ped_phase ;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//action
		tempUShort = (unsigned short)tempCmd;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2; 
		//number of the phases in second ring
		numberOfPhase=0;
		tempUShort = (unsigned short)numberOfPhase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		//Events in R
		offset = offset + 2;
	}
	
	// if the ped call is for phases in the second ring
	if (ped_phase==6 || ped_phase ==8)
	{
		//number of the phases in first ring
		numberOfPhase=0;
		tempUShort = (unsigned short)numberOfPhase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		//Events in R
		offset = offset + 2;
		
		numberOfPhase=1;
		//number of the phases in second ring
		tempUShort = (unsigned short)numberOfPhase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Time 
		tempLong = (long)(tempTime); 
		pByte = (byte* ) &tempLong;
		tmp_event_data[offset+0] = (byte) *(pByte + 3); 
		tmp_event_data[offset+1] = (byte) *(pByte + 2); 
		tmp_event_data[offset+2] = (byte) *(pByte + 1); 
		tmp_event_data[offset+3] = (byte) *(pByte + 0); 
		offset = offset + 4;
		//phase
		tempUShort = (unsigned short) ped_phase ;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//action
		tempUShort = (unsigned short)tempCmd;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2; 
		
		numberOfPhase=0;
		tempUShort = (unsigned short)numberOfPhase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		//Events in R
		offset = offset + 2;
	}
	size=offset;
	char* event_data;
	event_data= new char[size];			
	for(int i=0;i<size;i++)
	{
			event_data[i]=tmp_event_data[i];	
			
	}
	if (sendto(sockfd,event_data,size+1 , 0,(struct sockaddr *)&sendaddr, addr_length))
	{
		sprintf(temp_log," PED CLEAR SENT TO SignalControllerInterface FOR PHASE %d  \n", ped_phase); 
		outputlog(temp_log);
		cout<< temp_log<< endl;
	}
	
}

void Ped_Status_Read()
{
    netsnmp_session session, *ss;
	netsnmp_pdu *pdu;
	netsnmp_pdu *response;
	oid anOID[MAX_OID_LEN];
	size_t anOID_len;
	netsnmp_variable_list *vars;
	int status;
	init_snmp("ASC");   //Initialize the SNMP library
	snmp_sess_init( &session );  //Initialize a "session" that defines who we're going to talk to
	/* set up defaults */
	//char *ip = m_rampmeterip.GetBuffer(m_rampmeterip.GetLength());
	//char *port = m_rampmeterport.GetBuffer(m_rampmeterport.GetLength());
	char ipwithport[64];
	strcpy(ipwithport,INTip);
	strcat(ipwithport,":");
	strcat(ipwithport,INTport);
	session.peername = strdup(ipwithport);
	//session.version = SNMP_VERSION_2c; //for ASC intersection  /* set the SNMP version number */
	session.version = SNMP_VERSION_1; //for ASC intersection  /* set the SNMP version number */
	/* set the SNMPv1 community name used for authentication */
	session.community = (u_char *)"public";
	session.community_len = strlen((const char *)session.community);
	SOCK_STARTUP;
	ss = snmp_open(&session);                     /* establish the session */
	if (!ss)
	{
		snmp_sess_perror("ASC", &session);
		SOCK_CLEANUP;
		exit(1);
	}

	/*
	* Create the PDU for the data for our request.
	*   1) We're going to GET the system.sysDescr.0 node.
	*/
	pdu = snmp_pdu_create(SNMP_MSG_GET);
	anOID_len = MAX_OID_LEN;

	//---#define CUR_TIMING_PLAN     "1.3.6.1.4.1.1206.3.5.2.1.22.0"      // return the current timing plan

	char ctemp[50];
	sprintf(ctemp,"%s",WALK_GROUP);  //Check Ped phase status
	if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit as enabled or not: "1"  enable; "0" not used
	{
		snmp_perror(ctemp);
		SOCK_CLEANUP;
		exit(1);
	}
	snmp_add_null_var(pdu, anOID, anOID_len);
	sprintf(ctemp,"%s",PED_CALL);  //Check ped call
	if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit as enabled or not: "1"  enable; "0" not used
	{
		snmp_perror(ctemp);
		SOCK_CLEANUP;
		exit(1);
	}
	snmp_add_null_var(pdu, anOID, anOID_len);
	sprintf(ctemp,"%s",PEDCLEAR_GROUP);  //Check clear call
	if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit as enabled or not: "1"  enable; "0" not used
	{
		snmp_perror(ctemp);
		SOCK_CLEANUP;
		exit(1);
	}
	snmp_add_null_var(pdu, anOID, anOID_len);
	/*
	* Send the Request out.
	*/
	status = snmp_synch_response(ss, pdu, &response);
	/*
	* Process the response.
	*/
	if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
	{
		/*
		* SUCCESS: Print the result variables
		*/
		int *out = new int[MAX_ITEMS];
		int i =0;
	   // for(vars = response->variables; vars; vars = vars->next_variable)
	   //     print_variable(vars->name, vars->name_length, vars);

		/* manipuate the information ourselves */
		for(vars = response->variables; vars; vars = vars->next_variable)
		{
			if (vars->type == ASN_OCTET_STR)
			{
				char *sp = (char *)malloc(1 + vars->val_len);
				memcpy(sp, vars->val.string, vars->val_len);
				sp[vars->val_len] = '\0';
				//printf("value #%d is a string: %s\n", count++, sp);
				free(sp);
			}
			else
			{

				int *aa;
				aa =(int *)vars->val.integer;
				out[i++] = * aa;
				//printf("value #%d is NOT a string! Ack!. Value = %d \n", count++,*aa);
			}
		}
		
	   // Out[0] is ped status, Out[1] is ped call;
	   Ped_Info[0]=out[0];
	   Ped_Info[1]=out[1];
	   Ped_Info[2]=out[2];
			
	}
	else
	{
		if (status == STAT_SUCCESS)
			fprintf(stderr, "Error in packet\nReason: %s\n",
			snmp_errstring(response->errstat));
		else if (status == STAT_TIMEOUT)
			fprintf(stderr, "Timeout: No response from %s.\n",
			session.peername);
		else
			snmp_sess_perror("snmpdemoapp", ss);

	}

	/*
	* Clean up:    *  1) free the response.   *  2) close the session.
	*/
	if (response)        snmp_free_pdu(response);

	snmp_close(ss);

    SOCK_CLEANUP;
}

double GetSeconds()
{
    struct timeval tv_tt;
    gettimeofday(&tv_tt, NULL);
    return (tv_tt.tv_sec+tv_tt.tv_usec/1.0e6);    
}


void ReadInCoordinationConfig( char * filename)
{
	char TempStr[16];
	double dCoordinationWeight;
    int iCoordinatedPhase[2];
    double dTransitWeight;
    double dTruckWeight;
    double dCoordOffset;
    double dCoordCycle;
    double dCoordSplit;
    string lineread;
    fstream FileRead2;
    FileRead2.open(filename,ios::in);
    if(!FileRead2)
    {
        cerr<<"Unable to open file!"<<endl;
        exit(1);
    }
    //----------------- Read in the parameters---------------
    while(!FileRead2.eof())
    {
        getline(FileRead2,lineread);

        if (lineread.size()!=0)
        {
            sscanf(lineread.c_str(),"%s",TempStr);
            if(strcmp(TempStr,"coordination")==0)
            {					
                sscanf(lineread.c_str(),"%*s %lf ",&dCoordinationWeight);
            }
            else if(strcmp(TempStr,"cycle")==0)
            {
                sscanf(lineread.c_str(),"%*s %lf ",&dCoordCycle );
            }
            else if(strcmp(TempStr,"offset")==0)
            {
                sscanf(lineread.c_str(),"%*s %lf ",&dCoordOffset );
            }
            else if(strcmp(TempStr,"split")==0)
            {
                sscanf(lineread.c_str(),"%*s %lf ",&dCoordSplit );
            }
            else if(strcmp(TempStr,"coordinated_phase")==0)
            {
                sscanf(lineread.c_str(),"%*s %d %d ",&iCoordinatedPhase[0],&iCoordinatedPhase[1] );
            }
            else if(strcmp(TempStr,"transit_weight")==0)
            {
                sscanf(lineread.c_str(),"%*s %lf ",&dTransitWeight);
            }
            else if(strcmp(TempStr,"truck_weight")==0)
            {
                sscanf(lineread.c_str(),"%*s %lf ", &dTruckWeight);                
            }
        }
    }
     FileRead2.close();
    dCoordinationCycle =dCoordCycle;
    dCoordPhaseSplit=dCoordSplit;
    iCoordPhase[0]=iCoordinatedPhase[0];
    iCoordPhase[1]=iCoordinatedPhase[1];
}
