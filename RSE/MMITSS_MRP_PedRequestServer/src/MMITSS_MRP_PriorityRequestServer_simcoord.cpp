/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   

/*  MMITSS_MRP_PriorityRequestServer_sim.cpp  
*  Created by Mehdi Zamanipour
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.
*
*  Revision History:
*  1. Revised by Mehdi Zamanipour on 8/4/14 to add the posibility to receive SRM,decode it and populate it.
*  2. Request_conbimnd txt file is updated. New informaiton such as InLane,OutLane,
*  BeginServiceTime, EndServiceTime and IsCanceld added to each line of request_combined.txt
*  3. ASN1 decoder applied instead of savari decoder
*  4. PRS can integrate with ISIG (COP) with argument -c 2 and can work with priority+actuation by defaule argument -c 1
*  5. PRS is able to set coordination request with arguument -o 1  . By default the PRS does not put coordination request
*/
   
// The app receives priority requests in the format of SAE j2735 SRM from OBU_PRG_v2 , then updates the "requests.txt".
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <istream>
#include <string>
#include "ReqEntryListHandle.h"
#include "LinkedList.h"
#include "ReqEntry.h"
#include "IntLanePhase.h"
#include <math.h>
#include <vector>


#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>


#include "SRM.h"
#include "IntLanePhase.h"
#include "BasicVehicle.h"


using namespace std;

///* Reading Phase Status through NTCIP: different status has a number.  // FROM "rsu_config" app
//**********asc3PhaseStatusTiming2
// (1) X: XPED timing
// (2) N: Phase Next
// (3) -: Phase Not enabled
// (4) .: Phase Not Timing
// (5) R: Phase Timing RED
// (6) Y: Phase Timing YEL
// (7) G: Phase Timing GREEN
// (8) D: Phase Timing DELAY GREEN
// (9) O: Phase Timing YEL & RED
//(10) g: Phase Timing FLASHING GREEN
//(11) y: Phase Timing FLASHING YELLOW "

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
#define SIM_BROADCAST_ADDR "10.254.56.5"   //is used for getting VISSIM Simulation Time 

// for socket communication with obu
int PORT=  4444; ////***Important*** Port: For receiving request from OBEs
int req_interval=60;  // if a request is not updated for req_interval second in request list, it should be deleted
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
//~ //Parameters for setting up socket for system time
//~ int sockfd_clock;
//~ struct sockaddr_in sendaddr_clock;
//~ int numbytes_clock, addr_len_clock;
//~ int addr_length_clock = sizeof ( sendaddr_clock );
//char buf[1000];
//~ int CLOCK_PORT=10000;

int PRS_INTERFACE_PORT=44444; // prs send a clear command to interface when all 

double dRollingHorInterval=1.0;
char temp_log[256];
char tmp_log[256];  
int filename_cnt=0;
const char *INTport = "501";
int ReqListUpdateFlag=0;    // The Flag to identify the ReqList update
LinkedList<ReqEntry> Stop_List;  // Linked list to record the STOPPED vehicles
string RSUID;    // will get from "rsuid.txt"
char pctimestamp[128];
vector<int> v_PhaseNotEnabled;
char IPInfo[64]			  = "/nojournal/bin/IPInfo.txt";
char predir [64] 		  = "/nojournal/bin/"; 
char requestfilename[64]  = "/nojournal/bin/requests.txt";
//*** Add a file to restore the combined requests: requests_combined.txt BY DJ 2012.2.10
//*** The files will be modified and broadcast, and finally will be used by mpr_solver
char requestfilename_combined[64] = "/nojournal/bin/requests_combined.txt";
//This file stores reqeusts with SplitPhase: used for updating the request list.
char requestfilename_update[128]="/nojournal/bin/requests_update.txt";
char priorityConfiguartionFile[128]="/nojournal/bin/priorityConfiguration.txt";
char ConfigInfo[256]	  = "/nojournal/bin/ConfigInfo.txt";
char rsuid_filename[64]   = "/nojournal/bin/rsuid.txt";
char logfilename[256]     = "/nojournal/bin/log/MMITSS_PRS_";
char LanePhaseFile[255]   ="/nojournal/bin/InLane_OutLane_Phase_Mapping.txt";

char INTip[64];// = "150.135.152.23";
char ConfigFile[256] ;
int CombinedPhase[8]={0};
char        SRM_buf[MAX_SRM_BUFLEN];
SRM_t  * srm=0;
asn_enc_rval_t ec; /* Encoder return value */
asn_dec_rval_t rval;
// code usage, if the argument is -c 1 , the program will be used with traffic interface (priority and actuation) . if -c 2 as argument, the program work with ISIG
int icodeUsage=1;
// coordination request is set by PRS if the value of iOnCoordination=1 otherwise there is no coordination request.
int iOnCoordination=0;
int flagForClearingInterfaceCmd=0; // a flag to clear all commands in th interface when the request passed
double dCoordinationCycle=100.0;
double dCoordPhase1Split=30.0;
double dCoordPhase2Split=30.0;
double dCoordPhaseSplit=30.0;
double dCoordPhaseSplitDiff=0.0;
double dOffset=0.0;
int iCoordinatePhase1=2;
int iCoordinatePhase2=6;
float fcurrentCycleTime=100.0;

float Simtime(char *);  //Get simulation time from VISSIM
//int msleep_sim(int duration);
void obtainInLaneOutLane( int srmInLane, int srmOutLane, int &inApproach,int &outApproach,int &iInlane, int &Outlane);
void calculateETA(int beginMin, int beginSec, int endMin, int endSec, int &iETA );
void Pack_Event_List(char* tmp_event_data, int &size);
//void setupConnection(int broadcast, struct timeval tv, struct sockaddr_in sendaddr, struct sockaddr_in recvaddr, int &sockfd, int &addr_len);
void setupConnection();
void PhaseTimingStatusRead(int PhaseStatus[8]);
int GetSignalColor(int PhaseStatusNo);
int outputlog(char *output);
void UpdateList(LinkedList<ReqEntry> &ReqList,char *RcvMsg);
void PrintFile2Log(char *resultsfile);
void xTimeStamp( char * pc_TimeStamp_ );
int msleep(unsigned long milisec);
void get_rsu_id();
void get_ip_address();
void get_configfile();
void ReadInCoordinationConfig(char *);

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
	int iTotalCurrentReq=0;
	long lTimeOut=100000; // Time out of waiting for a new socket 1 second!
	int ret;
	unsigned long lintersectionID;
	char bsmBlob[BSM_BLOB_SIZE];
	BasicVehicle vehIn;
	float fSimCurrentTime=0.0;
    // coordination parameters
    
    float fLastReqFileRevisedTime=0.0;
    bool bCoordSplitTimeFlag=0;
    bool bCoord2ndSplitTimeFlag=0;
    bool bCoordEndSplitTimeFlag=0;
    bool bCoord2ndSplitTimeFlag2=0;
    bool bCoordEndSplitTimeFlag2=0;
    float fCoordPhase1ETA=0.0;
    float fCoordPhase2ETA=0.0;
    float fCoordPhase1MinGreen=0.0;
    float fCoordPhase2MinGreen=0.0;
    
	
    int iCoordMsgCont=0;	
    addr_length = sizeof ( recvaddr );
    LinkedList<ReqEntry> Req_List;  // List of all received requests
    int PhaseStatus[8]; //----- Determine the phase status for generate the split phases
    int iCycleNumber=0; // number of cycle up to now
    ReadInCoordinationConfig(priorityConfiguartionFile);
    bool bCoordPhsSpltSame=1;
    if (dCoordPhase1Split==dCoordPhase2Split) // if the coordinated phase splits are the same
		bCoordPhsSpltSame=1;
	else
	{
		dCoordPhaseSplit=max(dCoordPhase1Split,dCoordPhase2Split);
		dCoordPhaseSplitDiff=max(dCoordPhase1Split-dCoordPhase2Split,dCoordPhase2Split-dCoordPhase1Split);
		bCoordPhsSpltSame=0;
	}

	while ((ret = getopt(argc, argv, "p:t:c:o:")) != -1)    // -p  the port that the program receive SRM
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
				printf ("No coordination request \n");
			break;
		default:
			return 0;
		}
	}  
	//Struct for UDP socket timeout: 1s
    tv.tv_sec = 0;
    tv.tv_usec = lTimeOut;
    
	dRollingHorInterval=1.0; //double) lTimeOut/999999;  // rolling horizon is used for doing countdown on ETA of the received SRM
		
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
    //--- DELETE the Results.txt and requests.txt-------//
    system("\\rm /nojournal/bin/requests.txt");
	FILE *fp_req=fopen(requestfilename,"w");
    fprintf(fp_req,"Num_req -1 0\n");
    fclose(fp_req);

    system("\\rm /nojournal/bin/requests_combined.txt");
    fp_req=fopen(requestfilename_combined,"w");
    fprintf(fp_req,"Num_req -1 0\n");
    fclose(fp_req);
	// initializing the gps to get time later 

    get_ip_address();   // Get the ip address of controller
    get_rsu_id();  		// Get the current RSU ID from "rsuid.txt" into string "RSUID"
    get_configfile();   //------- Read the configinfo_XX.txt from ConfigInfo.txt--------//

    //--------------------------------For the wireless connection--------------------------------------//
    setupConnection();
    //------------------------------End of the wireless connection----------------------------//

	strcpy( cIntersectionName, RSUID.c_str());  // get the intersection name
	
	// geting the current active phase from controller
    PhaseTimingStatusRead(PhaseStatus); // First found out the not used(enabled) phases if there is any.
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
		
        cout << "..................Waiting for the request info..................\n";
        //-----Receive a request---------------//
        iNumBytes=-1;
        if ((iNumBytes = recvfrom(sockfd, SRM_buf, sizeof(SRM_buf), 0,(struct sockaddr *)&recvaddr, (socklen_t *)&iAddrLeng)) == -1)
        {
            perror("No Message Received ");  
                
        }
       	iTotalCurrentReq=ReqListFromFile(requestfilename,Req_List); // Read request file and put the request in data structure and return the number of current received requests
        if (iNumBytes>-1)    // if iNumBytes is non negative, we have receive messeges and we have to decode it
        {
			cout<<"SRM RECIEVED "<<endl;
			// decoding the received SRM
			srm =  (SRM_t  *) calloc(1, sizeof * (srm) );
			srm->timeOfService=(struct DTime*)calloc(1,sizeof( DTime_t));
			srm->endOfService=(struct DTime*)calloc(1,sizeof(DTime_t));
			srm->transitStatus= (BIT_STRING_t*)calloc(1, sizeof(BIT_STRING_t));
			srm->vehicleVIN=( VehicleIdent_t*)calloc(1, sizeof(VehicleIdent_t));
			//memset(&srmType, 0, sizeof(SRM_t));
			//memset(&vehIn,0, sizeof(BasicVehicle));
			rval = ber_decode(0, &asn_DEF_SRM,(void **)&srm, SRM_buf, sizeof(SRM_buf));	
			//xer_fprint(stdout, &asn_DEF_SRM, srm);
			if (rval.code==RC_OK)
			{
				sprintf(temp_log,"SRM Recieved From VISSIM: Decode Success\n");
				outputlog(temp_log);
				cout<<temp_log<<endl; 
				for (int i=0;i<38;i++) 
					bsmBlob[i]=srm->vehicleData.buf[i];
				vehIn.BSMToVehicle(bsmBlob);
				lintersectionID=(srm->request.id.buf[2]<<16)+(srm->request.id.buf[1]<<8)+srm->request.id.buf[0];
				if (lanePhase.iIntersectionID==lintersectionID)  // if the intersection ID in SRM matches the MAP ID of the intersection, SRM should be processed
				{
					if (srm->request.isCancel->buf[0]==0)
						strcpy( cMsgType, "request");
					else
						strcpy( cMsgType, "request_clear");
					obtainInLaneOutLane( srm->request.inLane->buf[0] , srm->request.outLane->buf[0] , iInApproach, ioutApproach, iInLane, iOutlane); 
					iRequestedPhase=lanePhase.iInLaneOutLanePhase[iInApproach][iInLane][ioutApproach][iOutlane];  
					iPriorityLevel=srm->request.type.buf[0] ; // is this the currect element of SRM to populate with vehicle  type?!
					dMinGrn=((srm->vehicleVIN->id->buf[1]<<8)+srm->vehicleVIN->id->buf[0])/10; 			 // there was no place in SMR to store MinGrn !!!!!
					iStartMinute=srm->timeOfService->minute;
					iStartSecond=srm->timeOfService->second;
					iEndMinute=srm->endOfService->minute;
					iEndSecond=srm->endOfService->second;
					iStartHour=srm->timeOfService->hour;
					iEndHour=srm->endOfService->hour;
					calculateETA(iStartMinute,iStartSecond,iEndMinute,iEndSecond,iETA );
					fETA=(float) iETA;
					lvehicleID=vehIn.TemporaryID;
					iVehicleState=srm->status->buf[0];
					iMsgCnt=srm->msgCnt;
					dSpeed=vehIn.motion.speed;
					//printsrmcsv (&srm);
					sprintf(RcvMsg,"%s %s %ld %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d", cMsgType, cIntersectionName, lvehicleID, iPriorityLevel,fETA,iRequestedPhase,dMinGrn, time(NULL), 
					srm->request.inLane->buf[0] , srm->request.outLane->buf[0] , iStartHour, iStartMinute, iStartSecond, iEndHour,iEndMinute, iEndSecond, iVehicleState, iMsgCnt);
					sprintf(temp_log," ******** The Received SRM  match the Intersection ID  ,  at time %ld. \n",time(NULL)); 
					outputlog(temp_log);
					cout<< temp_log<< endl;
					//~ sprintf(temp_log," ID, Type, ETA , Phase, QCT, inLane, outLane, Shr, Smn, Ssec, Ehr, Emn, Esec, State, Speed, Cnt  \n");
					//~ outputlog(temp_log);
					//~ cout<< temp_log<< endl;
					sprintf(temp_log,"%s\t \n",RcvMsg);
					outputlog(temp_log);
					cout<< temp_log<< endl;			
					PhaseTimingStatusRead(PhaseStatus);  // Get the current phase status for determining the split phases
					sprintf(temp_log,"Current Signal status:     %d  %d  %d  %d  %d  %d  %d  %d\t",PhaseStatus[0],PhaseStatus[1],PhaseStatus[2],PhaseStatus[3],PhaseStatus[4],PhaseStatus[5],PhaseStatus[6],PhaseStatus[7]);
					outputlog(temp_log);
					cout<<temp_log<<endl;
					ReqListFromFile(requestfilename,Req_List);
					UpdateList(Req_List,RcvMsg,PhaseStatus);   // Update the Req List data structure considering received message
					PrintList2File(requestfilename,Req_List,1);  // Write the requests list into requests.txt, 
					PrintFile2Log(requestfilename);
					PrintList2File(requestfilename_combined,Req_List,0);//Write the requests list into  requests_combined.txt; 
					PrintFile2Log(requestfilename_combined);
					cout<<"ReqListUpdateFlag  "<<ReqListUpdateFlag<<endl;
				}
			}
			else
			{	
				sprintf(temp_log," SRM Decode Failed !!!  Message Received From VISSIM is clock , so Update CLOCK\n");
				outputlog(temp_log);
				cout<<temp_log<<endl; 
				if (iOnCoordination==1)
					fSimCurrentTime=Simtime(SRM_buf);
			}
			free(srm->vehicleVIN);
			free(srm->transitStatus);
			free(srm->endOfService);		
			free(srm->timeOfService);
			free(srm);
		}
		
		if ( (iTotalCurrentReq>0 && iOnCoordination==0 && iNumBytes<0) || ( fLastReqFileRevisedTime>0 && (fSimCurrentTime-fLastReqFileRevisedTime)>dRollingHorInterval && iTotalCurrentReq>0 && iOnCoordination==1) ) // no new message received but there are still some requests in the request list, their ETA should be updated
		{
			cout<<"Simulation time  first is     : "<< fSimCurrentTime << endl;
			cout<<"Current cycle time  first is  : "<< fcurrentCycleTime << endl;
			if (iOnCoordination==1)
				fLastReqFileRevisedTime=fSimCurrentTime;
			
			ReqListUpdateFlag=0;
			fcurrentCycleTime=fSimCurrentTime-dOffset-dCoordinationCycle*(iCycleNumber-1);
			
			UpdateCurrentList(Req_List,iCoordinatePhase1,iCoordinatePhase2,dCoordPhase1Split,dCoordPhase2Split);
			PrintList2File(requestfilename,Req_List,1);  // Write the requests list into requests.txt, 
			PrintFile2Log(requestfilename);
			PrintList2File(requestfilename_combined,Req_List,0);//Write the requests list into  requests_combined.txt; This file will be different than requests.txt when we have EV
			PrintFile2Log(requestfilename_combined);
		}
			
		// --- Setting up the coordination requests -----
		if (iOnCoordination==1)
		{
			cout<<"Simulation time:    "<< fSimCurrentTime << endl;
			cout<<"Current cycle time: "<< fcurrentCycleTime << endl;
			fLastReqFileRevisedTime=fSimCurrentTime;
			// at the master clock we should put the coordination request for current cycle. If we have two different coordinated split times , we should put consider the beginning of the later split time too
			// at the end of the coordination phase split we should cancel the current cycle request and put the next cycle coordination request						
			bCoord2ndSplitTimeFlag2=0;
			bCoordEndSplitTimeFlag2=0;

			if ( ((int(fSimCurrentTime-dOffset)% int(dCoordinationCycle)) <= 2) && (dCoordinationCycle*iCycleNumber <= fSimCurrentTime-dOffset) ) // if we hit the beginnig of coordination split
			{
				iCycleNumber++;
				bCoordSplitTimeFlag=1;
				bCoordEndSplitTimeFlag=0;
				bCoord2ndSplitTimeFlag=0;
				ReqListUpdateFlag=6;
			}
			fcurrentCycleTime=fSimCurrentTime-dOffset-dCoordinationCycle*(iCycleNumber-1);
			
			if ( (fcurrentCycleTime>=dCoordPhaseSplit) && (bCoordEndSplitTimeFlag==0) )  // if we hit the end of coordination split
			{
				ReqListUpdateFlag=6; 
				bCoordEndSplitTimeFlag=1;
				bCoordEndSplitTimeFlag2=1;
			}

			if ( (bCoordPhsSpltSame==0) && (fcurrentCycleTime >= dCoordPhaseSplitDiff) && (bCoord2ndSplitTimeFlag==0) ) // if we hit the beiginnirng of the second coordinated split  ( incase the two coordinated phase split time are diferent)
			{	
				ReqListUpdateFlag=6;
				bCoord2ndSplitTimeFlag=1;
				bCoord2ndSplitTimeFlag2=1;
			}

			
			if (ReqListUpdateFlag==6)
			{
				strcpy( cMsgType, "coord_request");
						
				if (bCoordPhsSpltSame==1) // if the two coordinated phase split time are the same
				{
					if (bCoordSplitTimeFlag==1) // if signal is in the coordination split time 
					{
						bCoordSplitTimeFlag=0;
						fCoordPhase1MinGreen=dCoordPhaseSplit;
						fCoordPhase2MinGreen=dCoordPhaseSplit;
						fCoordPhase1ETA=0.0;
						fCoordPhase2ETA=0.0;					
					}else // if the signal already passed the coordination split time 
					{
						fCoordPhase1MinGreen=0.0;
						fCoordPhase2MinGreen=0.0;
						fCoordPhase1ETA=dCoordinationCycle-dCoordPhaseSplit;
						fCoordPhase2ETA=dCoordinationCycle-dCoordPhaseSplit;					
					}
				}else // if the two coorinated phase split times are different
				{
					if (bCoordSplitTimeFlag==1) // if signal is in the coordination split time 
					{
						bCoordSplitTimeFlag=0;
						if (dCoordPhase1Split>dCoordPhase2Split)
						{
							fCoordPhase1MinGreen=dCoordPhase1Split;
							fCoordPhase2MinGreen=0.0;
							fCoordPhase1ETA=0.0;
							fCoordPhase2ETA=dCoordPhaseSplitDiff;					
						}else
						{
							fCoordPhase1MinGreen=0.0;
							fCoordPhase2MinGreen=dCoordPhase2Split;
							fCoordPhase1ETA=dCoordPhaseSplitDiff;
							fCoordPhase2ETA=0.0;					
						}
					}else if (bCoord2ndSplitTimeFlag2==1)  
					{
						if (dCoordPhase1Split>dCoordPhase2Split)
						{
							fCoordPhase1MinGreen=dCoordPhase2Split;
							fCoordPhase2MinGreen=dCoordPhase2Split;
							fCoordPhase1ETA=0.0;
							fCoordPhase2ETA=0.0;					
						}else
						{
							fCoordPhase1MinGreen=dCoordPhase1Split;
							fCoordPhase2MinGreen=dCoordPhase1Split;
							fCoordPhase1ETA=0.0;
							fCoordPhase2ETA=0.0;					
						}
					}else if (bCoordEndSplitTimeFlag2==1) // if the signal already passed the coordination split time 
					{
						if (dCoordPhase1Split>dCoordPhase2Split)
						{
							fCoordPhase1MinGreen=0.0;
							fCoordPhase2MinGreen=0.0;
							fCoordPhase1ETA=dCoordinationCycle-dCoordPhaseSplit;
							fCoordPhase2ETA=dCoordinationCycle-dCoordPhaseSplit+dCoordPhaseSplitDiff;					
						}else
						{
							fCoordPhase1MinGreen=0.0;
							fCoordPhase2MinGreen=0.0;
							fCoordPhase1ETA=dCoordinationCycle-dCoordPhaseSplit+dCoordPhaseSplitDiff;					
							fCoordPhase2ETA=dCoordinationCycle-dCoordPhaseSplit;
						}
					}
				}
				
				iCoordMsgCont=(iCoordMsgCont+10)%127;
				sprintf(temp_log,"\n******************  Coordination Request Is Set ****************** At simulation time %f. \n",fSimCurrentTime ); 
				outputlog(temp_log); 
				cout<< temp_log<< endl;
				ReqListFromFile(requestfilename,Req_List);
				
				
				if (iCoordinatePhase1>0)
				{
					// Coordination priotiy type is set to be 6 and there are two fake id for the two coorinated phase 99998 and 99999
					sprintf(RcvMsg,"%s %s %d %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d", cMsgType, cIntersectionName, 99998, 6 ,fCoordPhase1ETA, iCoordinatePhase1 ,fCoordPhase1MinGreen, time(NULL),	0 , 0 , 0, 0, 0, 0,0, 0, 1, iCoordMsgCont);
					//sprintf(temp_log,"\n******************  Coordination Request Is Set ****************** At GPS time %ld. \n",time(NULL)); 
					sprintf(temp_log,"{%s}\t \n",RcvMsg); 
					outputlog(temp_log); 
					cout<< temp_log<< endl;		
					ReqListUpdateFlag=6;
					UpdateList(Req_List,RcvMsg,PhaseStatus);   // Update the Req List data structure considering first coordination 
				}
				if (iCoordinatePhase2>0)
				{
					sprintf(RcvMsg,"%s %s %d %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d", cMsgType, cIntersectionName, 99999, 6 ,fCoordPhase2ETA, iCoordinatePhase2 ,fCoordPhase2MinGreen, time(NULL),	0 , 0 , 0, 0, 0, 0,0, 0, 1, iCoordMsgCont); // the id for the second ring coordination request is set to 99999
					//sprintf(temp_log,"\n******************  Coordination Request Is Set ****************** At GPS time %ld. \n",time(NULL)); 
					sprintf(temp_log,"{%s}\t \n",RcvMsg); 
					outputlog(temp_log); 
					cout<< temp_log<< endl;		
					ReqListUpdateFlag=6;
					UpdateList(Req_List,RcvMsg,PhaseStatus);   // Update the Req List data structure considering seond coordination 
				}
				
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
            UpdateCurrentList(Req_List,iCoordinatePhase1,iCoordinatePhase2,dCoordPhase1Split,dCoordPhase2Split);
            PrintList2File(requestfilename,Req_List,1);
            PrintList2File(requestfilename_combined,Req_List,0);
        }
        
        if ( iOnCoordination==0 && ( (ReqListUpdateFlag>0 && Req_List.ListSize()==0 ) ||flagForClearingInterfaceCmd==1)  ) // Request list is empty and the last vehisle just passed the intersection 
        {
			ReqListUpdateFlag=0;
			flagForClearingInterfaceCmd=0;
			UpdateCurrentList(Req_List,iCoordinatePhase1,iCoordinatePhase2,dCoordPhase1Split,dCoordPhase2Split);
			PrintList2File(requestfilename,Req_List,1);  // Write the requests list into requests.txt, 
			PrintFile2Log(requestfilename);
			PrintList2File(requestfilename_combined,Req_List,0);//Write the requests list into  requests_combined.txt; This file will be different than requests.txt when we have EV
			PrintFile2Log(requestfilename_combined);
			// in case the program is being used for actuaion and prioirity this part of code send a hold command to interface for all phases 
			//when ever the last request pass the intersection to delete previous command
			if (icodeUsage==1) 
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
        //~	 print_variable(vars->name, vars->name_length, vars);
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

    string lineread; 	getline(fs,lineread);

    if(lineread.size()!=0)
    {
        sprintf(INTip,"%s",lineread.c_str());
        cout<<INTip<<endl;
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




//~ 
//~ void printsrmcsv (J2735SRM_t *srm) 
//~ {
//~ printf("Msg ID :%2x\n", srm->message_id);                                                             
//~ printf("Msg Count :%2x\n", srm->msgcount);                                                            
//~ printf("Intersection ID %x\n", srm->intersection_id);                                                 
//~ printf("Cancel Priority %x Cancel Preemption %x\n", srm->cancelreq_priority, srm->cancelreq_preemption);                                                                           
//~ printf("Signal  Preempt %x Priority %x \n", srm->signalreq_priority, srm->signalreq_preemption);                                              
//~ printf("Lane Num  IN %x OUT %x \n", srm->in_lanenum, srm->out_lanenum);                      
//~ printf("Veh Type %x  Veh Class %x \n", srm->vehicleclass_type, srm->vehicleclass_level);                                        
//~ printf("Code word %s\n", srm->code_word);                                                    
//~ printf("Start Time %02d:%02d:%02d -> End Time %02d:%02d:%02d\n", srm->starttime_hour,srm->starttime_min,srm->starttime_sec, srm->endtime_hour, srm->endtime_min, srm->endtime_sec);                                                                           
//~ printf("Transit status %x\n", srm->transitstatus);                                               
//~ printf("Vehicle Name %s\n", srm->vehicle_name);                                                  
//~ printf("Vin %s\n", srm->vin);                                                                    
//~ printf("Vehicle owner code %s\n", srm->vehicle_ownercode);                                       
//~ printf("Ident temp id %llx\n", srm->temp_ident_id);                                              
//~ printf("Vehicle type %d\n", srm->vehicle_type);                                                  
//~ printf("Class %x\n", srm->vehicle_class);                                                        
//~ printf("Group type %d\n", srm->vehicle_grouptype);                                               
//~ 
//~ printf("Temp ID :%llx\n", srm->temp_id);                                                         
//~ printf("SecMark :%x\n", srm->dsecond);                                                           
//~ printf("Latitude %8.2f\n", srm->latitude);                                                       
//~ printf("Longitude %8.2f\n", srm->longitude);                                                     
//~ printf("Elevation %8.2f\n", srm->elevation);                                                     
//~ printf("Pos 0 %8.2f\n", srm->positionalaccuracy[0]);                                             
//~ printf("Pos 1 %8.2f\n", srm->positionalaccuracy[1]);
//~ 
//~ printf("Pos 2 %8.2f\n", srm->positionalaccuracy[2]);        
//~ printf("TState %lf\n", srm->transmissionstate);                                                            
//~ printf("Speed %8.2f\n", srm->speed);                                                                       
//~ printf("Heading %8.2f\n", srm->heading);                                                                   
//~ printf("Angle %8.2lf\n", srm->angle);                                                                      
//~ printf("longAccel %8.2f\n", srm->longaccel);                                                               
//~ printf("LatAccel %8.2f\n", srm->lataccel);                                                                 
//~ printf("VertAccel %8.2f\n", srm->vertaccel);                                                               
//~ printf("YawRate %8.2f\n", srm->yawrate);                                                                   
//~ printf("Wheel Brake %x\n", srm->wheelbrake);                                                               
//~ printf("Wheel Brake AV %x\n", srm->wheelbrakeavailable);                                                   
//~ printf("SpareBit %x\n", srm->sparebit);                                                                    
//~ printf("Traction %x\n", srm->traction);                                                                    
//~ printf("ABS %x\n", srm->abs);                                                                              
//~ printf("Stab Contl %x\n", srm->stabilitycontrol);                                                          
//~ printf("BrakeBoost %x\n", srm->brakeboost);                                                                
//~ printf("Aux Brakes %x\n", srm->auxbrakes);                                                                 
//~ printf("Width %8.2f\n", srm->vehicle_width);                                                               
//~ printf("Length %8.2f\n", srm->vehicle_length);                                                             
//~ 
//~ 
//~ printf("*******srm PART1*********\n");                                                                     
//~ printf("Vehicle status %x\n", srm->vehicle_status);     
//~ 
//~ 
//~ }
//~ 

 
 
 
 

//void setupConnection(int broadcast, struct timeval tv, struct sockaddr_in sendaddr, struct sockaddr_in recvaddr, int &sockfd, int &addr_len)
void setupConnection()
{
	// -------------------------Network Connection for receiving SRM from VISSIM--------//
	 if((sockfd = socket(AF_INET,SOCK_DGRAM,0)) == -1)
    {
        perror("sockfd");
        exit(1);
    }
    
    int optval = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    
    //Setup time out
   if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0)  
    {
		      perror("Error");
    }
    
    // setup socket so that it can be reuse for vissim clock as well
    
    //
    //~ if((setsockopt(sockfd,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof broadcast)) == -1)
    //~ {
        //~ perror("setsockopt - SO_SOCKET ");
        //~ exit(1);
    //~ }
    //~ 
     // set up sending socket to interface to clean all commands when there is no request in the table
    recvaddr.sin_family = AF_INET;
    recvaddr.sin_port = htons(PORT);
    recvaddr.sin_addr.s_addr = INADDR_ANY;//inet_addr("10.254.56.255") ;;
    memset(recvaddr.sin_zero,'\0',sizeof recvaddr.sin_zero);
    if(bind(sockfd, (struct sockaddr*) &recvaddr, sizeof recvaddr) == -1)
    {
        perror("bind");
        exit(1);
    }
    //addr_len = sizeof  sendaddr ;
 	//~ sendaddr.sin_family = AF_INET;
	sendaddr.sin_port = htons(PRS_INTERFACE_PORT);//htons(PortOfInterface);   // PRS sends a mesage to traffic control interface to delete all the commands when ther is no request in the request table
	sendaddr.sin_addr.s_addr = inet_addr("127.0.0.1") ; //INADDR_ANY; // //INADDR_BROADCAST;
	memset(sendaddr.sin_zero,'\0',sizeof sendaddr.sin_zero);



//~ 
    //~ // -------------------------Network Connection for receiving the system clcok from VISSIM--------//
   //~ 
    //~ 
	//~ if((sockfd_clock = socket(AF_INET,SOCK_DGRAM,0)) == -1)
	//~ {
		//~ perror("sockfd");
		//~ exit(1);
	//~ }
	                     //~ 
    //~ int UdpBufSize = 1024;
	//~ setsockopt(sockfd_clock, SOL_SOCKET, SO_RCVBUF, &UdpBufSize, sizeof(int));
	//~ 
	//~ int optval = 1;
    //~ if (setsockopt(sockfd_clock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) ==-1)
    //~ {
		//~ printf("Setup clock socket failed! ");
	//~ }
    //~ // This part let the port to be used again .. This part is neccesary when we used the code for coordination in the simulation. because the MMITSS_MARP_TrafficControllerInterface_vissimtime also used this port to get the clock time from vissim
    //~ 
         //~ 
	//~ sendaddr_clock.sin_family = AF_INET;
	//~ sendaddr_clock.sin_port = htons(CLOCK_PORT);  //*** IMPORTANT: the vissim,signal control and performance observer should also have this port. ***//
	//~ sendaddr_clock.sin_addr.s_addr = INADDR_ANY; //inet_addr(SIM_BROADCAST_ADDR);//inet_addr(LOCAL_HOST_ADDR);//inet_addr(OBU_ADDR);//INADDR_ANY;
//~ 
	//~ memset(sendaddr_clock.sin_zero,'\0',sizeof sendaddr_clock.sin_zero);
//~ 
	//~ if(bind(sockfd_clock, (struct sockaddr*) &sendaddr_clock, sizeof sendaddr_clock) == -1)
	//~ {
		//~ printf("Bind clock socket failed");
		//~ perror("bind");   
		//~ exit(1);
	//~ }
}




void Pack_Event_List(char* tmp_event_data, int &size)
{
	int i,j;
	int offset=0;
	byte*   pByte;      // pointer used (by cast)to get at each byte 
                            // of the shorts, longs, and blobs
    byte    tempByte;   // values to hold data once converted to final format
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

float Simtime(char * buffer)
{
	unsigned char byteA, byteB, byteC, byteD;
	byteA = buffer[0];
	byteB = buffer[1];
	byteC = buffer[2];
	byteD = buffer[3];
	long  DSecond = (long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD)); // in fact unsigned
	return DSecond/10.0;
		
}
//~ 
//~ int msleep_sim(int duration)
//~ {
	//~ float t1,t2;
	//~ 
	//~ recvfrom(sockfd_clock, buf_clock, sizeof(buf_clock), 0,
                        //~ (struct sockaddr *)&sendaddr_clock, (socklen_t *)&addr_length_clock);
	//~ recvfrom(sockfd_clock, buf_clock, sizeof(buf_clock), 0,
                        //~ (struct sockaddr *)&sendaddr_clock, (socklen_t *)&addr_length_clock);    
	//~ 
	//~ 
	//~ unsigned char byteA, byteB, byteC, byteD;
    //~ byteA = buf_clock[0];
    //~ byteB = buf_clock[1];
    //~ byteC = buf_clock[2];
    //~ byteD = buf_clock[3];
    //~ 
	//~ long  DSecond = (long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD)); // in fact unsigned
	//~ 
	//~ t1=DSecond/10.0;
	//~ t2=t1;
	//~ while(t2-t1<duration/1000)
	//~ {
		//~ recvfrom(sockfd_clock, buf_clock, sizeof(buf_clock), 0,
                        //~ (struct sockaddr *)&sendaddr_clock, (socklen_t *)&addr_length_clock);
		//~ recvfrom(sockfd_clock, buf_clock, sizeof(buf_clock), 0,
                        //~ (struct sockaddr *)&sendaddr_clock, (socklen_t *)&addr_length_clock);    
//~ 
		//~ byteA = buf_clock[0];
		//~ byteB = buf_clock[1];
		//~ byteC = buf_clock[2];
		//~ byteD = buf_clock[3];
    //~ 
		//~ DSecond = (long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD)); // in fact unsigned
	//~ 
		//~ t2=DSecond/10.0;
	//~ }
//~ return 0;	
	//~ 
//~ }
void ReadInCoordinationConfig( char * filename)
{
	char TempStr[256];
	double dCoordinationWeight;
    int iCoordinatedPhase[2];
    double iTransitWeight;
    double iTruckWeight;
    double dCoordOffset;
    double dCoordCycle;
    double dCoordSplit[2];
    string lineread;
    fstream FileRead2;
    FileRead2.open(filename,ios::in);
    //FileRead.open("ConfigurationInfo1268.txt",ios::in);

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
           if(strcmp(TempStr,"coordination_weigth")==0)
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
            else if(strcmp(TempStr,"coordinated_phases_number")==0)
            {
				int iNumber_of_Phases=0;
				sscanf(lineread.c_str(),"%*s %d ",&iNumber_of_Phases);
				for (int cnt=0;cnt<iNumber_of_Phases;cnt++)
				{
					getline(FileRead2,lineread);
					 sscanf(lineread.c_str(),"%s",TempStr);
					if(strcmp(TempStr,"coordinated_phase1")==0)
						sscanf(lineread.c_str(),"%*s %ld ",&iCoordinatedPhase[0]);
		
					if (strcmp(TempStr,"coordinated_phase2")==0)
						sscanf(lineread.c_str(),"%*s %ld ",&iCoordinatedPhase[1] );
				}
				for (int cnt=0;cnt<iNumber_of_Phases;cnt++)
				{
					getline(FileRead2,lineread);
					 sscanf(lineread.c_str(),"%s",TempStr);
					if(strcmp(TempStr,"coordinated_phase1_split")==0)
						sscanf(lineread.c_str(),"%*s %lf ",&dCoordSplit[0]);
		
					if (strcmp(TempStr,"coordinated_phase2_split")==0)
						sscanf(lineread.c_str(),"%*s %lf ",&dCoordSplit[1] );
				}
            }
            else if(strcmp(TempStr,"transit_weight")==0)
            {
                sscanf(lineread.c_str(),"%*s %lf ",&iTransitWeight);
            }
            else if(strcmp(TempStr,"truck_weight")==0)
            {
                sscanf(lineread.c_str(),"%*s %lf ", &iTruckWeight);                
            }
        }
    }
    FileRead2.close();
    dCoordinationCycle =dCoordCycle;
    dCoordPhase1Split=dCoordSplit[0];
    dCoordPhase2Split=dCoordSplit[1];
    iCoordinatePhase1=iCoordinatedPhase[0];
    iCoordinatePhase2=iCoordinatedPhase[1];
    dOffset=dCoordOffset;	
}
