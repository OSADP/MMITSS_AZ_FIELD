/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   

/*  mprSolver.cpp  
*  Created by Mehdi Zamanipour
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.
*
*  Revision History:
*  1. For saturated condition, use -s 1 argument
* 
*  
*/
   





#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <netdb.h>
#include <unistd.h>
#include <getopt.h>
#include <arpa/inet.h>   
#include <iostream>
#include <sstream>
#include <istream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <glpk.h>

#include "math.h"
#include "GetInfo.h"
#include "Mib.h"
#include "Config.h"   // <vector>
#include "Signal.h"  
#include "Array.h"
#include "Schedule.h"
#include "LinkedList.h"
#include "PriorityRequest.h"
#include "ReqEntry.h"

using namespace std;

//---- The following two for function RunScheduleG()
#define NEW 1   //??
#define REGULAR 0  //??
#define OPT_MSG_SIZE 256
#ifndef byte
    #define byte  char  // needed if byte not defined
#endif

#ifndef DEG2ASNunits
    #define DEG2ASNunits  (1000000.0)  // used for ASN 1/10 MICRO deg to unit converts
#endif

#define ROBUSTTIME_LOW 2   // Lower bound in second: add to the ETA for ROBUST control (should be positive)
#define ROBUSTTIME_UP  4   // Upper bound in second: add to the ETA for ROBUST control

#define ROBUSTTIME_LOW_EV 4   // Lower bound in second: add to the ETA for ROBUST control (should be positive)
#define ROBUSTTIME_UP_EV  3   // Upper bound in second: add to the ETA for ROBUST control

#define EV  1
#define TRANSIT 2
#define TRUCK 3
#define PED 4
#define COORDINATION 6
#define MaxGrnTime 50         //ONLY change the phase {2,4,6,8} for EV requested phases


//define log file name with time stamp.
char logfilename[256]     			= "/nojournal/bin/log/mehdi_mprsolver_";
char signal_plan_file[256]      	= "/nojournal/bin/log/signal_Plan_";
//---Store the file name of the Config file.
//--- For Example: ConfigInfo_Hastings.txt, which has the minGrn and maxGrn etc.
char ConfigInfo[256]	  			= "/nojournal/bin/ConfigInfo.txt";
char IPInfo[64]			  			= "/nojournal/bin/IPInfo.txt";
char rsuid_filename[64]   			= "/nojournal/bin/rsuid.txt";
char PriorityConfigFile[64]  		= "/nojournal/bin/priorityConfiguration.txt";   // for importing the priority related inputs such as weights 
char request_combined_filename[128] = "/nojournal/bin/requests_combined.txt"; //requests_combined
char requestfilename[128] 			= "/nojournal/bin/requests.txt"; //requests
char signal_filename[128]		 	= "/nojournal/bin/signal_status.txt";
char prioritydatafile[128]			= "/nojournal/bin/NewModelData.dat";
char resultsfile[128]    			= "/nojournal/bin/Results.txt"; // Result from the GLPK solver.


// -------- BEGIN OF GLOABAL PARAMETERS-----//
// wireless connection variables
int sockfd;
int broadcast=1;
struct sockaddr_in sendaddr;
struct sockaddr_in recvaddr;
void setupConnection(); //  setup socket connection to send the optimize schedule to traffic controller interface (or to intelligent traffic control component)
int RxArrivalTablePort=6666; // The port that arrival table data comes from
int ArrTabMsgID=66;
int TxOPTPort=5555;          // the port that the optimal phase timing plane sends to SignalControl (COP)
int TxEventListPort=44444;
int OPTMsgID=55;
char cOPTbuffer[256];

char temp_log[256];
double MaxGrnExtPortion=0.15;  // Extended the MaxGrn of requested phases to this portion
double dGlobalGmax[8]={0.0}; // this vector get the values of ConfigIS.Gmax an will keep those values unless we need to change the max green time to ConfigIS.Gmax*(1+MaxGrnExtPortion)
RSU_Config ConfigIS;
RSU_Config ConfigIS_EV;
LinkedList<ReqEntry> Req_List_Combined;  // Req list for requests in requests_combined.txt
LinkedList<ReqEntry> ReqList;   // Req list for requests in requests.txt: For updating the ReqListUpdateFlag only
double  adCriticalPoints[2][3][5];  // This array keeps the crtitical point of the optimal schedule . The first dimention is Ring ( 2 rings [2] ) the senod dimention is the left or right side of the Critical zone ( [2] ) the third dimension is the total number of phases we look ahead in the schedule. which is 5 phases.
int omitPhase[8];   // The phases that should be omitted when we have EV among the priority vehicles.
LinkedList <Schedule> Eventlist_R1;
LinkedList <Schedule> Eventlist_R2;
Phase Phases;  //----Important one----//
string RSUID;
char ConfigFile[256];   //= "/nojournal/bin/ConfigInfo_MountainSpeedway.txt";
int AddPhantom=0;
int ClearFO=1;  
int ReqListUpdateFlag=0;
int HaveEVInList=0;   // if there is EV in Req, =1; No EV, =0.
int HaveCoordInList=0;   // if there is Coordination in Req, =1; No Coordination, =0.
int HaveTransitInList=0;   // if there is Transair in Req, =1; No Transair, =0.
int HaveTruckInList=0;   // if there is Truck in Req, =1; No Truck, =0.
int HavePedCall=0;    // if there is ped call in Req, =1; else =0.
char tmp_log[256];
int CurPhaseStatus[8]={0};    // when do phaseReading: the same results as phase_read withoug conversion
PhaseStatus phase_read={0};   // value would be {1,3,4}
char INTip[64];
char INTport[16];
int CombinedPhase[8]={0};
int Phase_Status[NumPhases]={0};
int codeUsage=1;
int congested=0;

int outputlog(char *output); // for logging 
int GLPKSolutionValidation(char *filename); //determine whether glpksolver get a feasible solution
void GenerateMod(char *Filename,RSU_Config ConfigIS,char *OutFilename);  // generate .mod file for glpk optimizer , //------- If there is no EV, using ConfigIS; if there is EV, using ConfigIS_EV. --------//
int numberOfPlannedPhase(double adCritPont[3][5], int rng);
void packOPT(char * buf,double cp[2][3][5],int msgCnt);
void printCriticalPoints();
void Construct_eventlist(double cp[2][3][5]);
void Construct_eventlist_EV(double cp[2][3][5],int omitPhas[8]);
void Pack_Event_List(byte* tmp_event_data,int &size);
double findLargestCP(double cp[3][5], int rng, int maxNumOfPlannedPhases);
void matchTheBarrierInCPs(double cp[2][3][5]);
int  RequestPhaseInList(LinkedList<ReqEntry> ReqList,int phase);  // return the position of the phase in the ReqList
int  FindRingNo(int phase);
void PrintList2File(char *Filename,LinkedList<ReqEntry> &ReqList,int BroadCast=0); //If BroadCast=1, will add RSUID as the first line
void PrintList2File_EV(char *Filename,LinkedList<ReqEntry> &ReqList,int BroadCast=0);
void ReqListFromFile(char *filename,LinkedList<ReqEntry>& Req_List);
void ReqListFromFile_EV(char *filename,LinkedList<ReqEntry>& Req_List);
void LinkList2DatFile(LinkedList<ReqEntry> Req_List,char *filename,double InitTime[2],int InitPhase[2],double GrnElapse[2],double transitWeigth, double truckWeigth, double coordinationweigt);
void LinkList2DatFileForEV(LinkedList<ReqEntry> Req_List,char *filename,double InitTime[2],int InitPhase[2],double GrnElapse[2],RSU_Config configIS, int ChangeMaxGrn=0);
void removeZeroWeightReq(LinkedList<ReqEntry> Req_List_,double dTrnWeight,double dTrkWeight, LinkedList<ReqEntry> &Req_List_New);
void FindReqWithSamePriority(LinkedList<ReqEntry> Req_List_, int priority,LinkedList<ReqEntry> &Req_List_New);
int  FindListHighestPriority(LinkedList<ReqEntry> Req_List);
int  FindVehClassInList(LinkedList<ReqEntry> Req_List,int VehClass);
int  FindSplitPhase(int phase,int phaseStatus[8]);
int  FindCompensatePhase(int ReqPhase, int InitPhase[2]);
void readOptPlanFromFile(char *filename,double adCriticalPoints[2][3][5]);
void readOptPlanFromFileForEV(char *filename,double adCriticalPoints[2][3][5], int omitPhase[8]);
int  findTheEarliestReqInERP(LinkedList<PriorityRequest> PR);  // find the position of the request in the PriorityRequestList that has the eailiet arrival time among the requests in (Earliest Requested Phase ) (ERP) requests set !)
void creatFeasibleRegionRing1(double CP[3][5], LinkedList<PriorityRequest> PrioReqListOfRing1, int phaseInRing, double rl, double ru, double delay, int type, double *EndOfPhase,int *phaseSeq ,int t, RSU_Config ConfigIS,double globalgmax[8], double initialGreen, double ElapsedGreen,int IsthereReqInOtherRing);
void creatFeasibleRegionRing2(double CP[3][5], LinkedList<PriorityRequest> PrioReqListOfRing2, int phaseInRing, double rl, double ru, double delay, int type, double *EndOfPhase,int *phaseSeq ,int t, RSU_Config ConfigIS, double globalgmax[8], double initialGreen, double ElapsedGreen,int IsthereReqInOtherRing);
void creatFeasibleRegionRing1_EV(double  CP[3][5], int phaseInRing, double ru,double delay,double *EndOfPhase,int *phaseSeq ,int t);
void creatFeasibleRegionRing2_EV(double  CP[3][5], int phaseInRing, double ru,double delay,double *EndOfPhase,int *phaseSeq ,int t);
void creatCPexactlyAsOptSol(double  CP[2][3][5], double *EndOfPhaseRing1,int *phaseSeqRing1 ,int t1, double *EndOfPhaseRing2,int *phaseSeqRing2 ,int t2);
void deductSolvingTimeFromCPs(double aCriticalPoints[2][3][5],double tt2,double tt1);
void PrintPlan2Log(char *resultsfile);
void PrintFile2Log(char *resultsfile);
void GLPKSolver();

int main ( int argc, char* argv[] )
{
	
     int ch;
     while ((ch = getopt(argc, argv, "c:s:")) != -1) 
     {
		switch (ch) 
		{
		    case 'c':
				codeUsage=atoi (optarg);
				if (codeUsage==1)
					printf ("Code usage is for Prioirty Alg + Actuation \n");
				else
					printf ("Code usage is for Prioirty Alg + COP \n");
				break;
			case 's':
				congested=atoi (optarg);
				if (congested==1)
					printf (" We have super congested situation \n");
				else
					printf (" Normal Traffic \n");
				break;
            default:
			     return 0;
		 }
    }
	int iOPTmsgCnt=0;
	int iArrTabMsgCnt;
    unsigned int type = 0;
    int SEND_OFF_CMD=3;
    int RequestFileNo=0;
    int IsListEmpty=1;  // ---IsListEmpty==1 means empty, will do nothing; =0, is not empty, need to carry on the plan.--//
    int InitPhase[2];
    double InitTime[2],GrnElapse[2];
    int coordinationIndicator=0;
 
    double TimeStamp[2][2];   // For Recording the end time of each cycle including the InitTime. // MZ changed to consider 2 cycles! 10/8/14

    //------log file name with Time stamp---------------------------
    char timestamp[128];
    string Logfile;
    xTimeStamp(timestamp);
    strcat(logfilename,timestamp);    Logfile=string(logfilename);
    strcat(logfilename,".log");
    strcat(signal_plan_file,timestamp);     strcat(signal_plan_file,".log");
    std::fstream fs_log,fs_signal_plan,fs_signal_status,fs_request_send;
    fs_log.open(logfilename, fstream::out | fstream::trunc);
    fs_signal_plan.open(signal_plan_file,fstream::out | fstream::trunc);
    //------end log file name-----------------------------------
	
	
	// ------ Readng Configuration---
	get_ip_address();           // READ the ASC controller IP address into INTip from "IPInfo.txt"
    get_rsu_id();               // Get rsu id for string RSUID from "rsuid.txt"
    //-------(1) Read the name of "configinfo_XX.txt" into "ConfigFile" from ConfigInfo.txt--------//
    //-------(2) Read in the CombinedPhase[8] for finding the split phases--
    get_configfile();
    //-----------------Read in the ConfigIS Every time in case of changing plan-----------------------//
    int curPlan=CurTimingPlanRead();
    sprintf(tmp_log,"Current timing plan is:\t%d\n",curPlan);  
    outputlog(tmp_log);

    IntersectionConfigRead(curPlan,ConfigFile);  // Generate: configinfo_XX.txt
    PrintPlan2Log(ConfigFile);
    ReadInConfig(ConfigFile,PriorityConfigFile); // Get Configuration from priority configurationa and controler
    PrintRSUConfig();
    GenerateMod(ConfigFile,ConfigIS,"/nojournal/bin/NewModel.mod");
    setupConnection();
    int addr_length = sizeof ( recvaddr );
    //--------------End of Read in the ConfigIS Every time in case of changing plan-----------------------//  //---- If we have EV priority, then we need to generate the special "NewModelData_EV.mod" through "ConfigIS" and "requested phases"
    //--- When in Red or Yellow, init1 & 2 are non-zero, Grn1&Grn2 (GrnElapse) are zero
    //--- When in Green, init1 & 2 are zero, Grn1&Grn2 are non-zero
    InitTime[0]=0; 
    InitTime[1]=0;
    GrnElapse[0]=0;
    GrnElapse[1]=0;
    
    while ( true )
    {
		double t_1,t_2; //---time stamps used to determine wether we connect to the RSE or not.
		t_1=GetSeconds();
		//---- Read the signal information------//
		PhaseTimingStatusRead();
		t_2=GetSeconds();
		if( (t_2-t_1)<2.0 ) // We do connect to RSE
		{
			Phases.UpdatePhase(phase_read);
			////------------Begin of recording the phase_status into signal_status.txt----------------//
			Phases.RecordPhase(signal_plan_file);
			for(int ii=0;ii<2;ii++)
			{
				InitPhase[ii]=Phases.InitPhase[ii]+1;   //{1-8}
				InitTime[ii] =Phases.InitTime[ii];      // ALSO is the (Yellow+Red) Left for the counting down time ***Important***
				GrnElapse[ii]=Phases.GrnElapse[ii];     // If in Green
			}
			
			cout<< " ***** initial phase of ring 1 is "<<  InitPhase[0] << " Ring 2  is "<<InitPhase[1]<< endl;
			//IF we can get signal information, then we connect to the ASC controller
			FILE * fp=fopen("/nojournal/bin/connection.txt","w"); // For determination of connecting to a ASC controller or not. Nothing to do with OBU
			fprintf(fp,"%ld",time(NULL));
			fclose(fp);
			msleep(20);   
			Req_List_Combined.ClearList();   
			ReqList.ClearList();
			//*** IMPORTANT***: THESE two request files are created in MRP_PRS, so MRP_PRS needs to be run ealier
			//*** Read from the uncombined phase file: requests.txt get ReqListUpdateFlag
			ReqListFromFile(requestfilename,ReqList);
			// Read from the combined phase file: requests_combined.txt***//
			ReqListFromFile_EV(request_combined_filename,Req_List_Combined);
			
			HaveEVInList=FindVehClassInList(Req_List_Combined,EV);
			HaveCoordInList=FindVehClassInList(Req_List_Combined,COORDINATION);
			HaveTruckInList=FindVehClassInList(Req_List_Combined,TRUCK);
			HaveTransitInList=FindVehClassInList(Req_List_Combined,TRANSIT);	
			HavePedCall=FindVehClassInList(Req_List_Combined, PED);
			
			
			if (Req_List_Combined.ListSize()>0)
			{
				IsListEmpty=0;
				if(ReqListUpdateFlag>0)
				{
					sprintf(tmp_log,"Requests Modified : Flag is{%d} at %s",ReqListUpdateFlag,GetDate());    //cout<<tmp_log;
					outputlog(tmp_log);
					cout<<tmp_log<<endl;
					PrintFile2Log(request_combined_filename);
					PrintFile2Log(requestfilename);
				}
				else
				{
					sprintf(tmp_log,"Read in the request list at %s",GetDate());    //cout<<tmp_log;
					outputlog(tmp_log);
					cout<<tmp_log<<endl;
					PrintFile2Log(request_combined_filename);
					PrintFile2Log(requestfilename);
				}
			}
			else  // if (Req_List_Combined.ListSize()>0)
			{
				sprintf(tmp_log,"No Request yet, at %lf",GetSeconds());    //cout<<tmp_log;
				outputlog(tmp_log);
				IsListEmpty=1;
			}  // end of "if (Req_List_Combined.ListSize()>0)"

				
			
			if( ReqListUpdateFlag>0  ) 
			{	//If there is EV, we need to generate a new model file and new data file 
				sprintf(tmp_log,"\n...............Solve the problem..............\t FLAG {%d} At time: %10.2lf",ReqListUpdateFlag,GetSeconds());
				outputlog(tmp_log);
				cout<<tmp_log<<endl;
				if(HaveEVInList==1)
				{
					sprintf(tmp_log,"...... We have EV. Need Dynamic Mod file......\t FLAG {%d} At time: %s",ReqListUpdateFlag,GetDate());
					outputlog(tmp_log); 
					cout<<tmp_log<<endl;
					//----------------------------------------Begin to generate the dynamic mod file----------------------------------------//
					int size_init=2;   //--- We always have two initial phases: if has{1,2,6,8}, timing 8, initial will be {4,8}
					int size_request=Req_List_Combined.ListSize();
					//---------------- EV phase vector------------------
					vector<int> EV_Phase_vc;
					Req_List_Combined.Reset();
					for(int i=0;i<size_request;i++)
					{
						if(Req_List_Combined.Data().VehClass==EV)
							EV_Phase_vc.push_back(Req_List_Combined.Data().Phase);
						Req_List_Combined.Next();
					}
					int EV_Phase_size=EV_Phase_vc.size();
					int SamePhase=AllSameElementOfVector(EV_Phase_vc); // If only have one element, return 0
					int ReqPhase,PhaseVCMissing,PhaseInitMissing;
					for(int i=0;i<EV_Phase_size;i++)
					{
						ReqPhase=EV_Phase_vc[i];
						PhaseVCMissing=BarrierPhaseIsMissing(ReqPhase,EV_Phase_vc);  // phase missing in the vector
						PhaseInitMissing=BarrierPhaseIsMissing(ReqPhase,InitPhase,2);
						if(PhaseInitMissing!=0 && PhaseVCMissing!=0)  // Then they should be the same
						{
							EV_Phase_vc.push_back(PhaseInitMissing);
						}
						int Pos=RequestPhaseInList(Req_List_Combined,ReqPhase);
						if(PhaseInitMissing==0 && PhaseVCMissing!=0)
						{
							if(Pos>=0)	Req_List_Combined.Reset(Pos);
							ReqEntry PhantomEntry=Req_List_Combined.Data();
							PhantomEntry.Phase=PhaseVCMissing;
							if(AddPhantom>0)
							{
								Req_List_Combined.InsertAfter(PhantomEntry);
								outputlog("\n Add a Phantom Req Entry!\n");
							}
						}
					}
					EV_Phase_size=EV_Phase_vc.size(); //  EV_Phase_size could be changed.
					int TotalSize=size_init+EV_Phase_size;
					int *Phase_Infom=new int[TotalSize];
					for(int i=0;i<EV_Phase_size;i++)
					{
						Phase_Infom[i]=EV_Phase_vc[i];
					}
					for(int i=0;i<size_init;i++)
					{
						Phase_Infom[EV_Phase_size+i]=InitPhase[i];
					}
					selectionSort(Phase_Infom, TotalSize);   // Sort all the involved phases for removing duplicated phases
					int NoRepeatSize=removeDuplicates(Phase_Infom, TotalSize);
					RSUConfig2ConfigFile("/nojournal/bin/ConfigInfo_EV.txt",Phase_Infom,NoRepeatSize,ConfigIS);
					PrintFile2Log("/nojournal/bin/ConfigInfo_EV.txt");// Log the EV configInfo.
					delete [] Phase_Infom;
					GenerateMod("/nojournal/bin/ConfigInfo_EV.txt",ConfigIS,"/nojournal/bin/NewModel_EV.mod"); // We need all {2,4,6,8}
					ConfigIS_EV=ReadInConfig("/nojournal/bin/ConfigInfo_EV.txt",1);
					RequestFileNo++;
					PrintRSUConfig2File(ConfigIS_EV,tmp_log);
					PrintRSUConfig(ConfigIS_EV);
					LinkList2DatFileForEV(Req_List_Combined,prioritydatafile,InitTime,InitPhase,GrnElapse,ConfigIS_EV,HaveEVInList);// construct .dat file for the glpk
					PrintFile2Log(prioritydatafile); 
					outputlog("\n");   // Log the .dat file for glpk solver
					if(PhaseVCMissing!=0 && PhaseInitMissing==0)
					{
						int Pos=RequestPhaseInList(Req_List_Combined,PhaseVCMissing);
						if(Pos>=0)
						{
							Req_List_Combined.Reset(Pos);
							Req_List_Combined.DeleteAt();
						}
					}
					//----------------------------------------End of generating the dynamic mod file for EV case --------------------------------------//
				}
				if ((HaveEVInList==0)&&(Req_List_Combined.ListSize()>0)) // Atleast one priority vehicle except EV is in the list!
				{
					LinkList2DatFile(Req_List_Combined,prioritydatafile,InitTime,InitPhase,GrnElapse, ConfigIS.iTransitWeight, ConfigIS.iTruckWeight,ConfigIS.dCoordinationWeight ); // construct .dat file for the glpk
					PrintFile2Log(prioritydatafile); // Log the .dat file for glpk solver
					outputlog("\n");  
					RequestFileNo++;
				}    // End of "if(HaveEVInList==1)"
	
				//-----According to initial phase, solve the problem, write to "Results.txt"//
				//----- ALSO SET the ReqListUpdateFlag in requests.txt to:"0"   ***IMPORTANT***
				PrintList2File(requestfilename,ReqList);
				PrintList2File_EV(request_combined_filename,Req_List_Combined);
				double t1=GetSeconds();
				GLPKSolver();  
				double t2=GetSeconds();
				sprintf(tmp_log,"Time for solving the problem is about: {%.3f}.\n",t2-t1); 
				outputlog(tmp_log);
				cout<< tmp_log<<endl;
				int success=GLPKSolutionValidation(resultsfile);
				if (success==0)  
				{					
					sprintf(tmp_log,"...............New plan..............:\t At time: %.2f\n",GetSeconds());
					outputlog(tmp_log); 
					cout<<tmp_log<<endl;
					PrintPlan2Log(resultsfile);
					if(HaveEVInList==1)
						readOptPlanFromFileForEV(resultsfile,adCriticalPoints, omitPhase);
					else
						readOptPlanFromFile(resultsfile,adCriticalPoints);
					
					deductSolvingTimeFromCPs(adCriticalPoints,t2,t1);
					
								
					// If the Solver works jointly with COP, we pack the Critical Points and send it to COP, othewise we transfer the CP to event list and send the event list to controller.
					if (codeUsage==1) // Send to Interface 
					{
						Eventlist_R1.ClearList();
						Eventlist_R2.ClearList();
						if(HaveEVInList==1)
						{
							matchTheBarrierInCPs(adCriticalPoints);
							Construct_eventlist_EV(adCriticalPoints,omitPhase);
						}	
						else
							Construct_eventlist(adCriticalPoints);
						byte tmp_event_data[500];
						int size=0;
						Pack_Event_List(tmp_event_data, size);
						char* event_data;
						event_data= new char[size];			
						for(int i=0;i<size;i++)
							event_data[i]=tmp_event_data[i];	
						
						recvaddr.sin_port = htons(TxEventListPort);
						if (sendto(sockfd,event_data,size+1 , 0,(struct sockaddr *)&recvaddr, addr_length))
						{
							sprintf(temp_log," The new Event List sent to SignalControllerInterface, The size is %d and time is : %.2f.......... \n", size, GetSeconds()); 
							outputlog(temp_log);
							cout<< temp_log<< endl;
						}
					}
					else // Send to COP
					{
						iOPTmsgCnt++;
						iOPTmsgCnt=iOPTmsgCnt%127;
						packOPT(cOPTbuffer,adCriticalPoints,iOPTmsgCnt);
						recvaddr.sin_port = htons(TxOPTPort);
						if ((sendto(sockfd, cOPTbuffer,OPT_MSG_SIZE, 0,(struct sockaddr *)&recvaddr, addr_length)>0) )
						{
							sprintf(temp_log," The new OPT set sent to SignalControl (COP) At time: %.2f.......... \n", GetSeconds());
							outputlog(temp_log);
							cout<< temp_log<< endl;
						}
					}
					printCriticalPoints();
				}
				else
				{
				outputlog("No feasible solution!\n");
				}
			}
			else  // else: if(ReqListUpdateFlag<=0)
			{
				sprintf(tmp_log,"No Need to solve, At time: %s",GetDate());
				outputlog(tmp_log); outputlog("\n");
				msleep(100);   // TODO: NEED this longer 0413
			}  // End of if(ReqListUpdateFlag<=0)
		}
		else  // // We do not connect to RSE
		{
			sprintf(tmp_log,"******Cannot connect to RSE, At time: [%.2f].\n",GetSeconds());
			cout<<tmp_log;      outputlog(tmp_log);
			msleep(400);
			continue;
		}  // end of  if( (t_2-t_1)<2.0 ) // We do connect to RSE
    }  // end of While(true)

    fs_log.close();
    fs_signal_plan.close();
    return 0;
}



					
int outputlog(char *output)
{
    FILE * stream = fopen( logfilename, "r" );
    fseek( stream, 0L, SEEK_END );
    long endPos = ftell( stream );
    fclose( stream );

    std::fstream fs;
    if (endPos <10000000)
        fs.open(logfilename, std::ios::out | std::ios::app);
    else
        fs.open(logfilename, std::ios::out | std::ios::trunc);
    if (!fs || !fs.good())
    {
        std::cout << "could not open file!\n";
        return -1;
    }
    fs << output;// << std::endl;

    if (fs.fail())
    {
        std::cout << "failed to append to file!\n";
        return -1;
    }
    fs.close();
    return 1;
}



void setupConnection()
{
	if((sockfd = socket(AF_INET,SOCK_DGRAM,0)) == -1)
	{
		perror("sockfd");
		exit(1);
	}
	if((setsockopt(sockfd,SOL_SOCKET,SO_BROADCAST,
					&broadcast,sizeof broadcast)) == -1)
	{
		perror("setsockopt - SO_SOCKET ");
		exit(1);
	}
	sendaddr.sin_family = AF_INET;
	sendaddr.sin_port = htons(RxArrivalTablePort);  //*** IMPORTANT: MPRsolver uses this port to receive the arrival table form Trajectory aware compponent //
	sendaddr.sin_addr.s_addr =  INADDR_ANY; //inet_addr ("127.0.0.1"); //inet_addr(OBU_ADDR);// inet_addr ("150.135.152.35"); //

	memset(sendaddr.sin_zero,'\0',sizeof sendaddr.sin_zero);

	if(bind(sockfd, (struct sockaddr*) &sendaddr, sizeof sendaddr) == -1)
	{
		perror("bind");        exit(1);
	}


	recvaddr.sin_family = AF_INET;
	recvaddr.sin_port = htons(TxOPTPort);   // MPRsolver uses this port to send the optimal solution to 
	recvaddr.sin_addr.s_addr = inet_addr("127.0.0.1") ; //INADDR_BROADCAST;
	memset(recvaddr.sin_zero,'\0',sizeof recvaddr.sin_zero);
}



int GLPKSolutionValidation(char *filename)
{
		fstream r_f;
		r_f.open(filename,fstream::in);
		int i,j;
		int IsValid=1;
		int flag=0;
		if (!r_f)
		{
			cout<<"***********Error opening the result file!\n";
			exit(1);
		}
		
		 string lineread;
	  double V[2][8];
	  getline(r_f,lineread);
	  getline(r_f,lineread);
	  for (i=0;i<2;i++)
	  {
		getline(r_f,lineread);
		sscanf(lineread.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf",&V[i][0],&V[i][1],&V[i][2],&V[i][3],&V[i][4],&V[i][5],&V[i][6],&V[i][7]);
		sprintf(tmp_log,"%lf %lf %lf %lf %lf %lf %lf %lf ",V[i][0],V[i][1],V[i][2],V[i][3],V[i][4],V[i][5],V[i][6],V[i][7]);
		outputlog(tmp_log);
	  }
	  outputlog("\n");
	  for(i=0;i<2;i++)
	  {
		for(j=0;j<8;j++)
		{

			if (V[i][j]>0.0)
			{
				IsValid=0;  //have valid solution
				flag=1;
				break;
			}
		}
		if (flag==1)
		break;
	  }
	  return IsValid;
}

  

void GenerateMod(char *Filename,RSU_Config ConfigIS,char *OutFilename)
{
	
	//-------reading the signal configuration file ---------
    fstream FileRead;
    FileRead.open(Filename,ios::in);

    if(!FileRead)
    {
        cerr<<"Unable to open file: "<<Filename<<endl;
        exit(1);
    }

    string lineread;

    int PhaseNo;
    char TempStr[16];
    vector<int> P11,P12,P21,P22;
    int PhaseSeq[8], Gmin[8], Gmax[8];
    float Yellow[8], Red[8];  // If

    for(int i=1;i<8;i=i+2) // Add {2,4,6,8} into the PhaseSeq: NECESSARY Phases
    {
        PhaseSeq[i]=i+1;
    }
    //------- Read in the parameters--
    while(!FileRead.eof())
    {
        getline(FileRead,lineread);

        if (lineread.size()!=0)
        {
            sscanf(lineread.c_str(),"%s",TempStr);

            if (strcmp(TempStr,"Phase_Num")==0)
            {
                sscanf(lineread.c_str(),"%s %d",TempStr,&PhaseNo);
                cout<<"Total Phase Number is:"<<PhaseNo<<endl;
            }

            else if(strcmp(TempStr,"Phase_Seq")==0)
            {
                //sscanf(lineread.c_str(),"%*s %d %d %d %d %d %d %d %d",
                //    &PhaseSeq[0],&PhaseSeq[1],&PhaseSeq[2],&PhaseSeq[3],
                //    &PhaseSeq[4],&PhaseSeq[5],&PhaseSeq[6],&PhaseSeq[7]);
                sscanf(lineread.c_str(),"%*s %d %*d %d %*d %d %*d %d %*d",
                    &PhaseSeq[0],&PhaseSeq[2],&PhaseSeq[4],&PhaseSeq[6]);

            }
            else if(strcmp(TempStr,"Yellow")==0)
            {
                sscanf(lineread.c_str(),"%*s %f %f %f %f %f %f %f %f",
                    &Yellow[0],&Yellow[1],&Yellow[2],&Yellow[3],
                    &Yellow[4],&Yellow[5],&Yellow[6],&Yellow[7]);
            }
            else if(strcmp(TempStr,"Red")==0)
            {
                sscanf(lineread.c_str(),"%*s %f %f %f %f %f %f %f %f",
                    &Red[0],&Red[1],&Red[2],&Red[3],
                    &Red[4],&Red[5],&Red[6],&Red[7]);
            }
            else if(strcmp(TempStr,"Gmin")==0)
            {
                sscanf(lineread.c_str(),"%*s %d %d %d %d %d %d %d %d",
                    &Gmin[0],&Gmin[1],&Gmin[2],&Gmin[3],
                    &Gmin[4],&Gmin[5],&Gmin[6],&Gmin[7]);
            }
            else if(strcmp(TempStr,"Gmax")==0)
            {
                sscanf(lineread.c_str(),"%*s %d %d %d %d %d %d %d %d",
                    &Gmax[0],&Gmax[1],&Gmax[2],&Gmax[3],
                    &Gmax[4],&Gmax[5],&Gmax[6],&Gmax[7]);
            }
        }
    }

    FileRead.close();
    

    //-------------Handle the parameters for non-complete phases case----//
    for(int i=0;i<8;i++)
    {
        if (PhaseSeq[i]>0)
        {
            switch (PhaseSeq[i])
            {
            case 1:
            case 2:
                P11.push_back(PhaseSeq[i]);
                break;
            case 3:
            case 4:
                P12.push_back(PhaseSeq[i]);
                break;
            case 5:
            case 6:
                P21.push_back(PhaseSeq[i]);
                break;
            case 7:
            case 8:
                P22.push_back(PhaseSeq[i]);
                break;
            }
        }
    }


	//-------READING the priority Configuratio file ---------------
    double dCoordinationWeight;
    int iCoordinatedPhase[2];
    double iTransitWeight;
    double iTruckWeight;
    int iCoordinationOffset;
    int iCoordinationCycle;
    int iCoordinationSplit;
    dCoordinationWeight=ConfigIS.dCoordinationWeight;
    iCoordinatedPhase[0]=ConfigIS.iCoordinatedPhase[0];
    iCoordinatedPhase[1]=ConfigIS.iCoordinatedPhase[1];
    iTransitWeight=ConfigIS.iTransitWeight;
    iTruckWeight=ConfigIS.iTruckWeight;
    iCoordinationCycle=ConfigIS.iCoordinationCycle;
    iCoordinationOffset=ConfigIS.iCoordinationOffset;
    iCoordinationSplit=ConfigIS.iCoordinationSplit;


    // ---------------- Writing the .mod  file------------------
    
	fstream FileMod;
    FileMod.open(OutFilename,ios::out);

    if(!FileMod)
    {
        cerr<<"Cannot open file: NewModel.mod to write!\n";
        exit(1);
    }

    int PhaseSeqArray[8],PhaseNumArray[8];
    int kk=0;

     if(P11.size()==1)
    {
        FileMod<<"set P11:={"<<P11[0]<<"}; \n";
        PhaseSeqArray[kk]=P11[0];
                kk++;
    }
    else if(P11.size()==2)
    {
        FileMod<<"set P11:={"<<P11[0]<<","<<P11[1]<<"};  \n";
        PhaseSeqArray[kk]=P11[0];
        kk++;
        PhaseSeqArray[kk]=P11[1];
        kk++;

     }

     if(P12.size()==1)
    {
        FileMod<<"set P12:={"<<P12[0]<<"}; \n";
        PhaseSeqArray[kk]=P12[0];
        kk++;
    }
    else if(P12.size()==2)
    {
        FileMod<<"set P12:={"<<P12[0]<<","<<P12[1]<<"};  \n";
        PhaseSeqArray[kk]=P12[0];
        kk++;
        PhaseSeqArray[kk]=P12[1];
        kk++;
     }

     if(P21.size()==1)
    {
        FileMod<<"set P21:={"<<P21[0]<<"}; \n";
        PhaseSeqArray[kk]=P21[0];
        kk++;
    }
    else if(P21.size()==2)
    {
        FileMod<<"set P21:={"<<P21[0]<<","<<P21[1]<<"};  \n";
        PhaseSeqArray[kk]=P21[0];
        kk++;
        PhaseSeqArray[kk]=P21[1];
        kk++;
     }

     if(P22.size()==1)
    {
        FileMod<<"set P22:={"<<P22[0]<<"}; \n";
        PhaseSeqArray[kk]=P22[0];
        kk++;
    }
    else if(P22.size()==2)
    {
        FileMod<<"set P22:={"<<P22[0]<<","<<P22[1]<<"};  \n";
        PhaseSeqArray[kk]=P22[0];
        kk++;
        PhaseSeqArray[kk]=P22[1];
        kk++;
     }

    FileMod<<"set P:={";

    for(int i=0;i<kk;i++)
    {
        if(i!=kk-1)
        { 
            FileMod<<" "<<PhaseSeqArray[i]<<",";
		}
        else
        {
            FileMod<<" "<<PhaseSeqArray[i];
         }
    }

    FileMod<<"};\n"; 

    //------------- Print the Main body of mode----------------
    FileMod<<"set K  := {1,2};\n";
    FileMod<<"set J  := {1..10};\n";
    FileMod<<"set P2  :={1..8};  \n";
    FileMod<<"set I  :={1..2};  \n";
    FileMod<<"set T  := {1..10}; 	# at most 10 different types of vehicle may be considered , EV are 1, Transit are 2, Trucks are 3  \n";      
    
    FileMod<<"  \n";
    FileMod<<"param y    {p in P}, >=0,default 0;  \n";
    FileMod<<"param red  {p in P}, >=0,default 0;  \n";
    FileMod<<"param gmin {p in P}, >=0,default 0;  \n";
    FileMod<<"param gmax {p in P}, >=0,default 0;  \n";
    FileMod<<"param init1,default 0;  \n";
    FileMod<<"param init2,default 0;  \n";
    FileMod<<"param Grn1, default 0;  \n";
    FileMod<<"param Grn2, default 0;  \n";
    FileMod<<"param SP1,  integer,default 0;  \n";
    FileMod<<"param SP2,  integer,default 0;  \n";
    FileMod<<"param M:=9999,integer;  \n";
    FileMod<<"param alpha:=100,integer;  \n";
    FileMod<<"param Rl{p in P, j in J}, >=0,  default 0;  \n";
    FileMod<<"param Ru{p in P, j in J}, >=0,  default 0;  \n";
    FileMod<<"param Tq{p in P, j in J}, >=0,  default 5;  \n";
    FileMod<<"  \n";
    
    if (dCoordinationWeight<0)
			dCoordinationWeight=0.0;
	if (iCoordinatedPhase[0]==0)
		iCoordinatedPhase[0]=2;
	if (iCoordinatedPhase[1]==0)
		iCoordinatedPhase[1]=6;
	FileMod<<"param current,>=0,default 0;   # current is  mod(time,cycle)  \n";
	FileMod<<"param coord, :="<<dCoordinationWeight <<"; 	# this paramter indicated where we consider coorrdination or not  \n";
	FileMod<<"param isItCoordinated, :=(if coord>0 then 1 else 0); \n";
	FileMod<<"param cycle, :="<<iCoordinationCycle  <<";    # if we have coordination, the cycle length  \n";
	FileMod<<"param offset,:="<<iCoordinationOffset <<";   # if we have coordination, the offset  \n";
	FileMod<<"param split, :="<<iCoordinationSplit  <<";    # if we have coordination, the split  \n";
	FileMod<<"param coordphase1,:="<< iCoordinatedPhase[0]  <<";    # if we have coordination, thecoordinated phase in ring1  \n";
	FileMod<<"param coordphase2,:="<< iCoordinatedPhase[1]  <<";    # if we have coordination, thecoordinated phase in ring2  \n";
	FileMod<<"param isCurPhCoord1, :=(if coordphase1=SP1 then 1 else 0);  \n";
	FileMod<<"param isCurPhCoord2, :=(if coordphase2=SP2 then 1 else 0);  \n";
	FileMod<<"param earlyReturnPhase1,  := (if (isCurPhCoord1=1 and current>gmax[coordphase1] and coordphase1>0) then 1 else 0); #  if we are in earlier coordinated phase green time, this earlyReturnPhase is 1 \n"; 
	FileMod<<"param earlyReturnPhase2,  := (if (isCurPhCoord2=1 and current>gmax[coordphase2] and coordphase1>0) then 1 else 0);  \n";
	FileMod<<"param PrioType { t in T}, >=0, default 0;  \n";
	FileMod<<"param PrioWeigth { t in T}, >=0, default 0;  \n";
	FileMod<<"param priorityType{j in J}, >=0, default 0;  \n";
	FileMod<<"param priorityTypeWeigth{j in J, t in T}, := (if (priorityType[j]=t) then PrioWeigth[t] else 0);  \n";
    FileMod<<"param Arr{p in P, i in I}, >=0,  default 0;  \n";
    FileMod<<"param active_arr{p in P, i in I}, integer, :=(if Arr[p,i]>0 then 1 else 0); \n";
    FileMod<<"param SumOfActiveArr, := (if (sum{p in P, i in I} Arr[p,i])>0 then (sum{p in P, i in I} Arr[p,i]) else 1); \n";
    FileMod<<"  \n";
    FileMod<<"param active_pj{p in P, j in J}, integer, :=(  if Rl[p,j]>0 then	1  else	0);  \n";
    FileMod<<"param coef{p in P,k in K}, integer,:=(   if ((((p<SP1 and p<5) or (p<SP2 and p>4 )) and k==1))then   	0   else   	1);  \n";
    FileMod<<"param MinGrn1{p in P,k in K},:=(  if (((p==SP1 and p<5) and k==1))then   	Grn1   else   	0);  \n";
    FileMod<<"param MinGrn2{p in P,k in K},:=(  if (((p==SP2 and p>4 ) and k==1))then   	Grn2   else   	0);  \n";
    FileMod<<"param ReqNo:=sum{p in P,j in J} active_pj[p,j];  \n";
    FileMod<<"param queue_pj{p in P, j in J}, integer, :=(  if Tq[p,j]>0 then	1  else	0);  \n";
    FileMod<<"  \n";
    FileMod<<"var t{p in P,k in K}, >=0;    # starting time vector  \n";
    FileMod<<"var g{p in P,k in K}, >=0;  \n";
    FileMod<<"var v{p in P,k in K}, >=0;  \n";
    FileMod<<"var d{p in P,j in J}, >=0;  \n";
    FileMod<<"var theta{p in P,j in J}, binary;  \n";
    FileMod<<"var PriorityDelay,>=0;  \n";
    FileMod<<" \n";
    FileMod<<"var miu{p in P,i in I}, binary; \n";
	FileMod<<"var rd{p in P,i in I}, >=0; \n";
    FileMod<<"var tmiu{p in P, i in I}, >=0; \n";
    FileMod<<"var TotRegVehDel, >=0; \n";
    FileMod<<"var QueClrTime{p in P, i in I},>=0; \n";
    FileMod<<"var ttheta{p in P,j in J}, >=0; \n";
    //FileMod<<"var dc1, >=0;\n";
    //FileMod<<"var dc2, >=0;\n";
    
    FileMod<<"  \n";
    FileMod<<"#================ Begin of cycle 1======================#  \n";
    FileMod<<"s.t. initial{p in P:(p<SP1) or (p<SP2 and p>4)}: t[p,1]=0;  \n";
    FileMod<<"s.t. initial1{p in P:p=SP1}: t[p,1]=init1;  \n";
    FileMod<<"s.t. initial2{p in P:p=SP2}: t[p,1]=init2;  \n";
    FileMod<<"\n # constraints in the same cycle in same P??  \n";
    FileMod<<"s.t. Prec_11_11_c1{p in P11: (p+1)in P11 and p>=SP1  }:  t[p+1,1]=t[p,1]+v[p,1];  \n";
    FileMod<<"s.t. Prec_12_12_c1{p in P12: (p+1)in P12 and p>=SP1  }:  t[p+1,1]=t[p,1]+v[p,1];  \n";
    FileMod<<"s.t. Prec_21_21_c1{p in P21: (p+1)in P21 and p>=SP2  }:  t[p+1,1]=t[p,1]+v[p,1];  \n";
    FileMod<<"s.t. Prec_22_22_c1{p in P22: (p+1)in P22 and p>=SP2  }:  t[p+1,1]=t[p,1]+v[p,1];  \n";
    FileMod<<"  \n";
    FileMod<<"# constraints in the same cycle in connecting   \n";
    FileMod<<"s.t. Prec_11_12_c1{p in P12: (card(P12)+p)<=5 and p>SP1  }:  t[p,1]=t[2,1]+v[2,1];  \n";
    FileMod<<"s.t. Prec_11_22_c1{p in P22: (card(P22)+p)<=9 and p>SP2  }:  t[p,1]=t[2,1]+v[2,1];  \n";
    FileMod<<"s.t. Prec_21_12_c1{p in P12: (card(P12)+p)<=5 and p>SP1  }:  t[p,1]=t[6,1]+v[6,1];  \n";
    FileMod<<"s.t. Prec_21_22_c1{p in P22: (card(P22)+p)<=9 and p>SP2  }:  t[p,1]=t[6,1]+v[6,1];  \n";
    FileMod<<"  \n";
    FileMod<<"#================ END of cycle 1======================#  \n";
    FileMod<<"  \n";
    FileMod<<"# constraints in the same cycle in same P??  \n";
    FileMod<<"s.t. Prec_11_11_c23{p in P11, k in K: (p+1)in P11 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  \n";
    FileMod<<"s.t. Prec_12_12_c23{p in P12, k in K: (p+1)in P12 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  \n";
    FileMod<<"s.t. Prec_21_21_c23{p in P21, k in K: (p+1)in P21 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  \n";
    FileMod<<"s.t. Prec_22_22_c23{p in P22, k in K: (p+1)in P22 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  \n";
    FileMod<<"  \n";
    FileMod<<"# constraints in the same cycle in connecting   \n";
    FileMod<<"s.t. Prec_11_12_c23{p in P12, k in K: (card(P12)+p)=5 and k>1 }:  t[p,k]=t[2,k]+v[2,k];\n";
    FileMod<<"s.t. Prec_11_22_c23{p in P22, k in K: (card(P22)+p)=9 and k>1 }:  t[p,k]=t[2,k]+v[2,k];  \n";
    FileMod<<"s.t. Prec_21_12_c23{p in P12, k in K: (card(P12)+p)=5 and k>1 }:  t[p,k]=t[6,k]+v[6,k];  \n";
    FileMod<<"s.t. Prec_21_22_c23{p in P22, k in K: (card(P22)+p)=9 and k>1 }:  t[p,k]=t[6,k]+v[6,k];  \n";
    FileMod<<"  \n";
    FileMod<<"# constraints in connecting in different cycles  \n";
    FileMod<<"s.t. Prec_12_11_c23{p in P11, k in K: (card(P11)+p+1)=4 and k>1 }:    t[p,k]=t[4,k-1]+v[4,k-1];  \n";
    FileMod<<"s.t. Prec_22_11_c23{p in P11, k in K: (card(P11)+p+1+4)=8 and k>1 }:  t[p,k]=t[8,k-1]+v[8,k-1];  \n";
    FileMod<<"s.t. Prec_12_21_c23{p in P21, k in K: (card(P21)+p+1-4)=4 and k>1 }:  t[p,k]=t[4,k-1]+v[4,k-1];  \n";
    FileMod<<"s.t. Prec_22_21_c23{p in P21, k in K: (card(P21)+p+1)=8 and k>1 }:    t[p,k]=t[8,k-1]+v[8,k-1];  \n";
    FileMod<<"  \n";
    FileMod<<"#==================================================#  \n";
    FileMod<<"  \n";
    FileMod<<" s.t. RD: PriorityDelay=( sum{p in P,j in J, tt in T} (priorityTypeWeigth[j,tt]*active_pj[p,j]*d[p,j] ) );  \n";  
    FileMod<<"  \n";
    FileMod<<" s.t. TotRegVehDelay: TotRegVehDel=(sum{p in P, i in I} active_arr[p,i]*Arr[p,i]*rd[p,i])/SumOfActiveArr;  \n";
    FileMod<<"  \n";
    FileMod<<"s.t. PhaseLen{p in P, k in K}:  v[p,k]=(g[p,k]+y[p]+red[p])*coef[p,k];  \n";
    FileMod<<"  \n";
    FileMod<<"s.t. PhaseLen2{p in P, k in K}:  v[p,k]*coef[p,k]>=g[p,k]; \n";
    FileMod<<"  \n";
    FileMod<<"s.t. GrnMax{p in P : (p!= coordphase1 and p!=coordphase2)}:  g[p,1]<=gmax[p];    \n";
    FileMod<<"  \n";
    FileMod<<"  s.t. GrnMax2{p in P}:  (1-isItCoordinated)*g[p,2]<=gmax[p];   \n";    //  !!!!!!!!!!!!!!!!!!!!!! FileMod<<"  s.t. GrnMax2{p in P}:  g[p,2]<=gmax[p];   \n";
    FileMod<<"  \n";
    FileMod<<"  s.t. GrnMax3{p in P}:  (1-isItCoordinated)*g[p,1]<=gmax[p];  \n";
    FileMod<<"  \n";
	FileMod<<"  s.t. GrnMin{p in P,k in K}:  g[p,k]>=(gmin[p]-MinGrn1[p,k]-MinGrn2[p,k])*coef[p,k];  \n";
	FileMod<<"  \n";
	//FileMod<<"  s.t. GrnMin2{p in P:(p=coordphase1 and isCurPhCoord1=1)} :g[p,1]>=isItCoordinated*( split-current ); \n";
	//FileMod<<"  \n";
	//FileMod<<"  s.t. GrnMin3{p in P:(p=coordphase2 and isCurPhCoord2=1)} :g[p,1]>=isItCoordinated*( split-current ); \n";
	//FileMod<<"  \n";
	//FileMod<<"  s.t. CoordPhasMax1{p in P: p=coordphase1}: isItCoordinated*(1-earlyReturnPhase1)*g[p,1]<=gmax[p]; \n";
	//FileMod<<"  \n";
	//FileMod<<"  s.t. CoordPhasMax2{p in P: p=coordphase2}: isItCoordinated*(1-earlyReturnPhase2)*g[p,1]<=gmax[p]; \n";
	//FileMod<<"  \n";
	//FileMod<<"s.t. CoordDelay1{p in P: p=coordphase1}: dc1>=isItCoordinated*((cycle-current+split) - (t[p,2]+v[p,2]) ); \n";
	//FileMod<<"  \n";
	//FileMod<<"s.t. CoordDelay2{p in P: p=coordphase2}: dc2>=isItCoordinated*((cycle-current+split) - (t[p,2]+v[p,2]) ); \n";
	//FileMod<<"  \n";
	//FileMod<<"s.t. CoordDelay5{p in P: ((p in P21)and current>gmax[coordphase1] and coordphase1>0)}: dc2>= isItCoordinated*((cycle-current+split) - (t[coordphase2,1]+v[coordphase2,1]));   # I assumed the coordinated phase of ring 2 is in the second brrier \n";
	//FileMod<<"  \n";
	//FileMod<<"s.t. CoordDelay6{p in P: ((p in P21)and current>gmax[coordphase2] and coordphase1>0)}: dc2>= isItCoordinated*((cycle-current+split) - (t[coordphase2,1]+v[coordphase2,1]));   # I assumed the coordinated phase of ring 2 is in the second brrier \n";
	//FileMod<<"  \n";
	FileMod<<"  \n";
	FileMod<<"	s.t. PrioDelay1{p in P,j in J: active_pj[p,j]>0 }:    d[p,j]>=t[p,1]-Rl[p,j];  \n";
	FileMod<<"  \n";
	FileMod<<" s.t. PrioDelay2{p in P,j in J: active_pj[p,j]>0 }:    M*theta[p,j]>=Ru[p,j]-(t[p,1]+g[p,1]); \n";
	FileMod<<"  \n";
	FileMod<<" s.t. PrioDelay3{p in P,j in J: active_pj[p,j]>0 }:    d[p,j]>= ttheta[p,j]-Rl[p,j]*theta[p,j]; \n";
	FileMod<<"  \n";
	FileMod<<" s.t. PrioDelay4{p in P,j in J: active_pj[p,j]>0 }:    g[p,1]>= (Ru[p,j]-Rl[p,j])*(1-theta[p,j]); \n";
	FileMod<<"  \n";
	FileMod<<" s.t. PrioDelay5{p in P, j in J: active_pj[p,j]>0}:    ttheta[p,j]<=M*theta[p,j]; \n";
	FileMod<<"  \n";
	FileMod<<" s.t. PrioDelay6{p in P, j in J: active_pj[p,j]>0}:    t[p,2]-M*(1-theta[p,j])<=ttheta[p,j]; \n";
	FileMod<<"  \n";
	FileMod<<" s.t. PrioDelay7{p in P, j in J: active_pj[p,j]>0}:    t[p,2]+M*(1-theta[p,j])>=ttheta[p,j]; \n";
	FileMod<<"  \n";
	FileMod<<" s.t. PrioDelay8{p in P, j in J: active_pj[p,j]>0}:   g[p,2]>=(Ru[p,j]-Rl[p,j])*theta[p,j]; \n ";
	FileMod<<"  \n";
	FileMod<<"s.t. PrioDelay9 {p in P, j in J: active_pj[p,j]>0}:   Ru[p,j]*theta[p,j] <= ( t[p,2]+g[p,2]) ; \n ";
	FileMod<<"  \n";
	//FileMod<<"  s.t. RegVehDel1{p in P, i in I: active_arr[p,i]>0}: rd[p,i] >= t[p,1]-i; \n";
	//FileMod<<"  \n";
	//FileMod<<"s.t. RegVehDel2{p in P, i in I: active_arr[p,i]>0} : M*miu[p,i] >= i-(t[p,1]+g[p,1]); \n";
	//FileMod<<"  \n";
	//FileMod<<"s.t. RegVehDel3{p in P, i in I: active_arr[p,i]>0} : rd[p,i] >=  tmiu[p,i]-i*miu[p,i]; \n";
	//FileMod<<"  \n";
	//FileMod<<"s.t. RegVehDel4{p in P, i in I: active_arr[p,i]>0} : tmiu[p,i]<=M*miu[p,i]; \n";
	//FileMod<<"  \n";
	//FileMod<<"s.t. RegVehDel5{p in P, i in I: active_arr[p,i]>0} : t[p,2]-M*(1-miu[p,i])<=tmiu[p,i]; \n";
	//FileMod<<"  \n";
	//FileMod<<"s.t. RegVehDel6{p in P, i in I: active_arr[p,i]>0} : t[p,2]+M*(1-miu[p,i])>=tmiu[p,i]; \n";
	//FileMod<<"  \n";
	
 
	if(HaveEVInList==1)
	{
		// WITH EV case: the minimum green time, a[p,k] should be 0
		FileMod<<"minimize delay: PriorityDelay  ;\n";
	}
	else
	{
		// WITHOUT EV case: the Maximum green time, a[p,k] should be as large as possible.
		FileMod<<"  minimize delay:  TotRegVehDel+  PriorityDelay;     \n";
	}

    FileMod<<"  \n";
    FileMod<<"solve;  \n";
    FileMod<<"  \n";
    FileMod<<"printf \" \" > \"/nojournal/bin/Results.txt\";  \n";
    FileMod<<"printf \"%3d  %3d \\n \",SP1, SP2 >>\"/nojournal/bin/Results.txt\";  \n";
    FileMod<<"printf \"%5.2f  %5.2f %5.2f  %5.2f \\n \",init1, init2,Grn1,Grn2 >>\"/nojournal/bin/Results.txt\";  \n";
    FileMod<<"for {k in K}   \n";
    FileMod<<" { \n";
    FileMod<<"     for {p in P2} \n";
    FileMod<<"        { \n";
    FileMod<<"           printf \"%5.2f  \", if(p in P)  then v[p,k] else 0  >>\"/nojournal/bin/Results.txt\";   \n";
    FileMod<<"        } \n";
    FileMod<<"        printf \" \\n \">>\"/nojournal/bin/Results.txt\";\n";
    FileMod<<" } \n";
    FileMod<<"  \n";
    // Added on 2012.4.4: the g[p,k] is added for TimeStamp[2][3].
    FileMod<<"for {k in K}   \n";
    FileMod<<" { \n";
    FileMod<<"     for {p in P2} \n";
    FileMod<<"        { \n";
    FileMod<<"           printf \"%5.2f  \", if(p in P)  then g[p,k] else 0  >>\"/nojournal/bin/Results.txt\";   \n";
    FileMod<<"        } \n";
    FileMod<<"        printf \" \\n \">>\"/nojournal/bin/Results.txt\";\n";
    FileMod<<" } \n";
    FileMod<<"  \n";
    FileMod<<"printf \"%3d \\n \", ReqNo >>\"/nojournal/bin/Results.txt\";  \n";
    FileMod<<"  \n";
    FileMod<<"for {p in P,j in J : Rl[p,j]>0}  \n";
    FileMod<<" {  \n";
    FileMod<<"   printf \"%d  %5.2f  %5.2f  %5.2f %d \\n \", (p+ 10*(theta[p,j])), Rl[p,j],Ru[p,j], d[p,j] , priorityType[j] >>\"/nojournal/bin/Results.txt\";\n";
    FileMod<<" } \n";
    FileMod<<"  \n";
    FileMod<<"printf \"%5.2f \\n \", PriorityDelay >>\"/nojournal/bin/Results.txt\"; \n";
    FileMod<<"printf \"%5.2f \\n \", TotRegVehDel >>\"/nojournal/bin/Results.txt\";  \n ";
    FileMod<<"printf \" \\n \">>\"/nojournal/bin/Results.txt\";\n";
    //------------- End of Print the Main body of mode----------------
    FileMod<<"end;\n";
    FileMod.close();
}





int numberOfPlannedPhase( double cp[3][5], int  rng)
{
	if (rng==1)
	{
		int NoPhaseRng1=0;
		for (int i=0; i<4; i++)
		{
			if (( cp[0][i]>0 && cp[0][i+1]==0 )|| (cp[1][i]>0 && cp[1][i+1]==0)) 
			{
				NoPhaseRng1=i+1;	
			}
		}
		if (cp[1][4]>0)
			NoPhaseRng1=5;
		
		//~ else if (NoPhaseRng1==-1 && cp[][1]==0 && cp[1][1]==0)
			//~ NoPhaseRng1=1;
		return  NoPhaseRng1;
	}
	else 
	{
		int NoPhaseRng2=0;
		for (int i=0; i<4; i++)
		{
			if ((cp[0][i]>0 && cp[0][i+1]==0) || (cp[1][i]>0 && cp[1][i+1]==0)) 
			{
				NoPhaseRng2=i+1;	
			}
		}
		if (cp[1][4]>0)
			NoPhaseRng2=5;
		return  NoPhaseRng2;
	}
}

double findLargestCP(double cp[3][5], int rng, int maxNumOfPlannedPhases)
{
	double temp=0.0;
	for(int i=0;i<maxNumOfPlannedPhases;i++)
	{
		if (cp[0][i]>temp)
			temp=cp[0][i];		
	}
	return temp;
}
void printCriticalPoints()
{
	cout<<" RING 1 CRITICAL POINTS "<<endl;
	for (int ii=0;ii<5;ii++)
	{
		cout<<"CP left              "<< ii << " "<< adCriticalPoints[0][0][ii]<<endl;
		cout<<"CP right             "<< ii << " "<< adCriticalPoints[0][1][ii]<<endl;
		cout<<"phase                "<< adCriticalPoints[0][2][ii] +1 <<endl;
	}
	cout<<endl;
	cout<<" RING 2 CRITICAL POINTS "<<endl;
	for (int ii=0;ii<5;ii++)
	{
		cout<<"CP left              "<< ii << " "<< adCriticalPoints[1][0][ii]<<endl;
		cout<<"CP right             "<< ii << " "<< adCriticalPoints[1][1][ii]<<endl;
		cout<<"phase                "<< adCriticalPoints[1][2][ii] + 5<<endl;
	}
	//sprintf(temp_log,".......... The new OPT set sent  At time: %.2f.......... \n", GetSeconds()); outputlog(temp_log); cout<< temp_log<< endl;
}


void packOPT(char* buffer,double cp[2][3][5], int msgCnt)
{
	int temp;
	int iNoPlannedPhaseInRing1=0;
    int iNoPlannedPhaseInRing2=0;
	iNoPlannedPhaseInRing1=numberOfPlannedPhase(cp[0],1);
	iNoPlannedPhaseInRing2=numberOfPlannedPhase(cp[1],2);
	cout<<"iNoPlannedPhaseInRing1 "<<iNoPlannedPhaseInRing1<<endl;
	cout<<"iNoPlannedPhaseInRing2 "<<iNoPlannedPhaseInRing2<<endl;
	int offset=0;
	char*   pByte;      // pointer used (by cast)to get at each byte 
	unsigned short   tempUShort;
	long templong;
	
	tempUShort = (unsigned short)OPTMsgID;
	pByte = (char*) & tempUShort;
	buffer[offset+0]= (char) * (pByte+1);
	buffer[offset+1]= (char) * (pByte+0);
	offset=offset+2;
	
	tempUShort = (unsigned short)msgCnt;
	pByte = (char*) & tempUShort;
	buffer[offset+0]= (char) * (pByte+1);
	buffer[offset+1]= (char) * (pByte+0);
	offset=offset+2;
	
	buffer[offset+0]=0X01;
	offset++;
	
	tempUShort = (unsigned short)iNoPlannedPhaseInRing1;
	pByte = (char*) & tempUShort;
	buffer[offset+0]= (char) * (pByte+1);
	buffer[offset+1]= (char) * (pByte+0);
	offset=offset+2;
	
	for (int ii=0;ii<5;ii++)
	{
		templong = (long)cp[0][0][ii]*1000000;
		pByte = (char*) & templong;
		buffer[offset+0]= (char) * (pByte+3);
		buffer[offset+1]= (char) * (pByte+2);
		buffer[offset+2]= (char) * (pByte+1);
		buffer[offset+3]= (char) * (pByte+0);
		offset=offset+4;
		
		templong = (long)cp[0][1][ii]*1000000;
		pByte = (char*) & templong;
		buffer[offset+0]= (char) * (pByte+3);
		buffer[offset+1]= (char) * (pByte+2);
		buffer[offset+2]= (char) * (pByte+1);
		buffer[offset+3]= (char) * (pByte+0);
		offset=offset+4;
		
		temp= (int) (cp[0][2][ii]+1);  // by adding 1 to cp[0][2][ii]  ,  the sent phase number is 1 2 3 or 4 
		tempUShort = (unsigned short)temp;
		pByte = (char*) & tempUShort;
		buffer[offset+0]= (char) * (pByte+1);
		buffer[offset+1]= (char) * (pByte+0);
		offset=offset+2;
	}
	buffer[offset+0]=0X02;
	offset++;
	
	tempUShort = (unsigned short)iNoPlannedPhaseInRing2;
	pByte = (char*) & tempUShort;
	buffer[offset+0]= (char) * (pByte+1);
	buffer[offset+1]= (char) * (pByte+0);
	offset=offset+2;	
	for (int i=0;i<5;i++)
	{
		templong = (long)cp[1][0][i]*1000000;
		pByte = (char*) & templong;
		buffer[offset+0]= (char) * (pByte+3);
		buffer[offset+1]= (char) * (pByte+2);
		buffer[offset+2]= (char) * (pByte+1);
		buffer[offset+3]= (char) * (pByte+0);
		offset=offset+4;
		
		templong = (long)cp[1][1][i]*1000000;
		pByte = (char*) & templong;
		buffer[offset+0]= (char) * (pByte+3);
		buffer[offset+1]= (char) * (pByte+2);
		buffer[offset+2]= (char) * (pByte+1);
		buffer[offset+3]= (char) * (pByte+0);
		offset=offset+4;
		
		temp= (int) (cp[1][2][i]+5);  // by adding 1 to cp[0][2][ii]  ,  the sent phase number is 5 6 7 or 8 
		tempUShort = (unsigned short)temp;
		pByte = (char*) & tempUShort;
		buffer[offset+0]= (char) * (pByte+1);
		buffer[offset+1]= (char) * (pByte+0);
		offset=offset+2;
	}
}

void Construct_eventlist_EV(double cp [2][3][5], int omitphase[8])
{
	int temp;
	int tempOmitPhases[8];
	int iNoOfOmit=0;
	Schedule Temp_event;
	double iLargestCPinRing1=0.0;
	double iLargestCPinRing2=0.0;
	double dOmitThereshold=0.0;
	int iNoPlannedPhaseInRing1=0;
    int iNoPlannedPhaseInRing2=0;
	iNoPlannedPhaseInRing1=numberOfPlannedPhase(cp[0],1);
	iNoPlannedPhaseInRing2=numberOfPlannedPhase(cp[1],2);
	iLargestCPinRing1=findLargestCP(cp[0],1,iNoPlannedPhaseInRing1);
	iLargestCPinRing2=findLargestCP(cp[1],2,iNoPlannedPhaseInRing2);
	cout<<"No of Planned Phase In Ring1 "<<iNoPlannedPhaseInRing1<<endl;
	cout<<"No of Planned Phase In Ring2 "<<iNoPlannedPhaseInRing2<<endl;
	cout<<"Longest CP in Ring1 "<<iLargestCPinRing1<<endl;
	cout<<"Longest CP in Ring2 "<<iLargestCPinRing2<<endl;
	if (iLargestCPinRing1>0 && iLargestCPinRing2>0 )
		dOmitThereshold=min(iLargestCPinRing1,iLargestCPinRing2);
	else if (iLargestCPinRing1==0)
		dOmitThereshold=iLargestCPinRing2;
	else if (iLargestCPinRing2==0)
		dOmitThereshold=iLargestCPinRing1;
	for (int i=0;i<8;i++)
	{
		if (omitphase[i]>0)
		{
			tempOmitPhases[iNoOfOmit]=omitphase[i];
			iNoOfOmit++;	
		}
	}
	//cout<<"No Of Omit"<<iNoOfOmit<<endl;
	for (int i=0;i<iNoOfOmit;i++)
	{
		Temp_event.time=dOmitThereshold;
		Temp_event.action=PHASE_OMIT;
		Temp_event.phase=tempOmitPhases[i]; // converting phase number from 0-3 to  1-4
		if (tempOmitPhases[i]<5)
			Eventlist_R1.InsertRear(Temp_event);	
		else
			Eventlist_R2.InsertRear(Temp_event);	
	}
	for (int i=0; i<iNoPlannedPhaseInRing1; i++)
	{
		// hold
		Temp_event.time=cp[0][0][i];
		Temp_event.action=PHASE_HOLD;
		Temp_event.phase=((int)(cp[0][2][i]))+1; // converting phase number from 0-3 to  1-4
		Eventlist_R1.InsertRear(Temp_event);	
		// call , the call is neccessary before force off. The controller should know where to go ( which phase will come up after force off )
		if ( i<iNoPlannedPhaseInRing1-1)
		{
			
			Temp_event.time=cp[0][1][i];
			if ( ( ((int)(cp[0][2][i+1]))+1 ) ==5 )
				Temp_event.phase=1;
			else
				Temp_event.phase=((int)(cp[0][2][i+1]))+1;// converting phase number from 0-3 to  1-4
			Temp_event.action=PHASE_VEH_CALL;
			Eventlist_R1.InsertRear(Temp_event);	
			
		}
		// force off
		Temp_event.time=cp[0][1][i];
		Temp_event.action=PHASE_FORCEOFF;
		Temp_event.phase=((int)(cp[0][2][i]))+1;// converting phase number from 0-3 to  1-4
		Eventlist_R1.InsertRear(Temp_event);	
	}
	for (int i=0; i<iNoPlannedPhaseInRing2; i++)
	{
		// hold
		Temp_event.time=cp[1][0][i];
		Temp_event.action=PHASE_HOLD;
		Temp_event.phase=((int) (cp[1][2][i]))+5;// converting phase number from 0-3 to  5-8
		Eventlist_R2.InsertRear(Temp_event);	
		// call
		if ( i<iNoPlannedPhaseInRing2-1)
		{
			Temp_event.time=cp[1][1][i];
			Temp_event.action=PHASE_VEH_CALL;
			if ( ((int)(cp[1][2][i+1]))+5 == 9 )
				Temp_event.phase=5;
			else
				Temp_event.phase=((int)(cp[1][2][i+1]))+5;// converting phase number from 0-3 to  5-8
			Eventlist_R1.InsertRear(Temp_event);	
			
		}
		// force off
		Temp_event.time=cp[1][1][i];
		Temp_event.action=PHASE_FORCEOFF;
		Temp_event.phase=((int) (cp[1][2][i]))+5;   // converting phase number from 0-3 to  5-8
		Eventlist_R2.InsertRear(Temp_event);	
	}
	Eventlist_R1.Reset();
	Eventlist_R2.Reset();
	cout<<"RING1 Event List!"<<endl;
	while (! Eventlist_R1.EndOfList())
	{
		cout<<"time "<<Eventlist_R1.Data().time<<" phse "<<Eventlist_R1.Data().phase<<" event "<<Eventlist_R1.Data().action<<endl;
		Eventlist_R1.Next();
	}
	cout<<"RING2 Event List!"<<endl;
	while (! Eventlist_R2.EndOfList())
	{
		cout<<"time "<<Eventlist_R2.Data().time<<" phse "<<Eventlist_R2.Data().phase<<" event "<<Eventlist_R2.Data().action<<endl;
		Eventlist_R2.Next();
	}
	
	for (int ii=0;ii<8;ii++)
		cout<<"  omitPhase[i] "<< omitPhase[ii]<< endl;
}



void Construct_eventlist(double cp [2][3][5])
{
	int temp;
	int iNoPlannedPhaseInRing1=0;
    int iNoPlannedPhaseInRing2=0;
	iNoPlannedPhaseInRing1=numberOfPlannedPhase(cp[0],1);
	iNoPlannedPhaseInRing2=numberOfPlannedPhase(cp[1],2);
	cout<<"No of Planned Phase In Ring1 "<<iNoPlannedPhaseInRing1<<endl;
	cout<<"No of Planned Phase In Ring2 "<<iNoPlannedPhaseInRing2<<endl;
	Schedule Temp_event;
	
	for (int i=0; i<iNoPlannedPhaseInRing1; i++)
	{
		// call
		Temp_event.time=cp[0][0][i];
		Temp_event.action=PHASE_HOLD;
		Temp_event.phase=(int) (cp[0][2][i]+1); // converting phase number from 0-3 to  1-4
		Eventlist_R1.InsertRear(Temp_event);	
		// call , the call is neccessary before force off. The controller should know where to go ( which phase will come up after force off )
		if ( i<iNoPlannedPhaseInRing1-1)
		{
			if ( ( ((int)(cp[0][2][i+1]))+1 )%2 ==0 )  // JUST PUT CALL FOR MAJOR MOVEMENT!!!!!!!!!  THIS IS BC IN THE VERY LOW DEMAND, WE DONT SERVE LEFT TURN!! IF THERE IS NO CAR ON THE DETECTORS 
			{
			Temp_event.time=cp[0][1][i];
			Temp_event.action=PHASE_VEH_CALL;
			Temp_event.phase=((int)(cp[0][2][i+1]))+1;// converting phase number from 0-3 to  1-4
			Eventlist_R1.InsertRear(Temp_event);
			}	
		}
		// force off
		Temp_event.time=cp[0][1][i];
		Temp_event.action=PHASE_FORCEOFF;
		Temp_event.phase=(int) (cp[0][2][i]+1);// converting phase number from 0-3 to  1-4
		Eventlist_R1.InsertRear(Temp_event);	
	}
	
	for (int i=0; i<iNoPlannedPhaseInRing2; i++)
	{
		// hold
		Temp_event.time=cp[1][0][i];
		Temp_event.action=PHASE_HOLD;
		Temp_event.phase=(int) (cp[1][2][i]+5);// converting phase number from 0-3 to  5-8
		Eventlist_R2.InsertRear(Temp_event);
		// call
		if ( i<iNoPlannedPhaseInRing2-1)
		{
			if ( (((int)(cp[1][2][i+1]))+5 )%2 == 0 ) // JUST PUT CALL FOR MAJOR MOVEMENT!!!!!!!!!  THIS IS BC IN THE VERY LOW DEMAND, WE DONT SERVE LEFT TURN!! IF THERE IS NO CAR ON THE DETECTORS 
			{
				Temp_event.time=cp[1][1][i];
				Temp_event.action=PHASE_VEH_CALL;
				Temp_event.phase=((int)(cp[1][2][i+1]))+5;// converting phase number from 0-3 to  5-8
				Eventlist_R1.InsertRear(Temp_event);	
			}
		}
		//force off	
		Temp_event.time=cp[1][1][i];
		Temp_event.action=PHASE_FORCEOFF;
		Temp_event.phase=(int) (cp[1][2][i]+5);   // converting phase number from 0-3 to  5-8
		Eventlist_R2.InsertRear(Temp_event);	
	}
	Eventlist_R1.Reset();
	Eventlist_R2.Reset();
	cout<<"RING1 Event List!"<<endl;
	while (! Eventlist_R1.EndOfList())
	{
		cout<<"time "<<Eventlist_R1.Data().time<<" phse "<<Eventlist_R1.Data().phase<<" event "<<Eventlist_R1.Data().action<<endl;
		Eventlist_R1.Next();
	}
	cout<<"RING2 Event List!"<<endl;
	while (! Eventlist_R2.EndOfList())
	{
		cout<<"time "<<Eventlist_R2.Data().time<<" phse "<<Eventlist_R2.Data().phase<<" event "<<Eventlist_R2.Data().action<<endl;
		Eventlist_R2.Next();
	}
}

void Pack_Event_List(char* tmp_event_data, int &size) // This function is written by YF
{
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
	int No_Event_R1= Eventlist_R1.ListSize();
	tempUShort = (unsigned short)No_Event_R1;
	pByte = (byte* ) &tempUShort;
    tmp_event_data[offset+0] = (byte) *(pByte + 1); 
    tmp_event_data[offset+1] = (byte) *(pByte + 0); 
	offset = offset + 2;
	//Events in R1
	Eventlist_R1.Reset();
	while(!Eventlist_R1.EndOfList())
	{
		//Time 
		tempLong = (long)(Eventlist_R1.Data().time * DEG2ASNunits); 
		pByte = (byte* ) &tempLong;
		tmp_event_data[offset+0] = (byte) *(pByte + 3); 
		tmp_event_data[offset+1] = (byte) *(pByte + 2); 
		tmp_event_data[offset+2] = (byte) *(pByte + 1); 
		tmp_event_data[offset+3] = (byte) *(pByte + 0); 
		offset = offset + 4;
		//phase
		tempUShort = (unsigned short)Eventlist_R1.Data().phase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//action
		tempUShort = (unsigned short)Eventlist_R1.Data().action;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		Eventlist_R1.Next();
	}
	//No. events in R2
	int No_Event_R2= Eventlist_R2.ListSize();
	tempUShort = (unsigned short)No_Event_R2;
	pByte = (byte* ) &tempUShort;
    tmp_event_data[offset+0] = (byte) *(pByte + 1); 
    tmp_event_data[offset+1] = (byte) *(pByte + 0); 
	offset = offset + 2;
	//Events in R1
	Eventlist_R2.Reset();
	while(!Eventlist_R2.EndOfList())
	{
		//Time 
		tempLong = (long)(Eventlist_R2.Data().time * DEG2ASNunits); 
		pByte = (byte* ) &tempLong;
		tmp_event_data[offset+0] = (byte) *(pByte + 3); 
		tmp_event_data[offset+1] = (byte) *(pByte + 2); 
		tmp_event_data[offset+2] = (byte) *(pByte + 1); 
		tmp_event_data[offset+3] = (byte) *(pByte + 0); 
		offset = offset + 4;
		//phase
		tempUShort = (unsigned short)Eventlist_R2.Data().phase;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//action
		tempUShort = (unsigned short)Eventlist_R2.Data().action;
		pByte = (byte* ) &tempUShort;
		tmp_event_data[offset+0] = (byte) *(pByte + 1); 
		tmp_event_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		Eventlist_R2.Next();	
	}
	size=offset;
}


void matchTheBarrierInCPs(double cp[2][3][5])
{
	int temp1 =numberOfPlannedPhase(cp[0],1);
	int temp2 =numberOfPlannedPhase(cp[1],2);
	double dtemp1=0.0;
	double dtemp2=0.0;
	double dtemp3=0.0;
	double dtemp4=0.0;
	if (temp1==0)
	{
		for (int it=0;it<3;it++)
			for(int it2=0;it2<5;it2++)
				cp[0][it][it2]=cp[1][it][it2];
		
	}else if (temp2==0)
	{
		for (int it=0;it<3;it++)
			for(int it2=0;it2<5;it2++)
				cp[1][it][it2]=cp[0][it][it2];	
	}else if (temp1>0 && temp2>0 && temp2<5 && temp1<5)   // match the barrier 
	{
		for(int it=0;it<5;it++)
		{
			for(int it2=0;it2<5;it2++)
				if ( (( (int) cp[0][2][it] ) % 2 == 1) && (cp[0][2][it]==cp[1][2][it2]) ) 
				{	
					dtemp1=cp[0][0][it];	
					dtemp2=cp[1][0][it2];
					dtemp1=max(dtemp1,dtemp2);
					dtemp3=cp[0][1][it];	
					dtemp4=cp[1][1][it2];
					dtemp3=min(dtemp3,dtemp4);  // temp 3 should be larger than temp 1 
					if (dtemp3<dtemp1)
						dtemp3=dtemp1;
					cp[0][0][it]=dtemp1;
					cp[1][0][it2]=dtemp1;
					cp[0][1][it]=dtemp3;
					cp[1][1][it2]=dtemp3;
				}	
		}
	}
}


int FindCompensatePhase(int ReqPhase, int InitPhase[2])
{
    // Find the compensate phase for EV requested phase in case some phases missed.
    int CompensatePhase=-10;

    if(ReqPhase!=InitPhase[0] && ReqPhase!=InitPhase[1])
    {
        switch(ReqPhase)
        {
        case 1:
            CompensatePhase=6;
            break;
        case 2:
            CompensatePhase=5;
            break;
        case 3:
            CompensatePhase=8;
            break;
        case 4:
            CompensatePhase=7;
            break;
        case 5:
            CompensatePhase=2;
            break;
        case 6:
            CompensatePhase=1;
            break;
        case 7:
            CompensatePhase=4;
            break;
        case 8:
            CompensatePhase=3;
            break;
        default:
            outputlog("***Error finding compensate Phase Information.***\n");
            break;
        }
    }

    return CompensatePhase;

}

int FindSplitPhase(int phase,int phaseStatus[8])
{
    //*** From global array CombinedPhase[] to get the combined phase :get_configfile() generates CombinedPhase[]
    //*** If the phase exsits, the value is not 0; if not exists, the default value is '0'.; "-1"  means will not change later
    //*** The argument phase should be among {1..8}
    //int CombinedPhase[8]={1,2,3,4,5,6,7,8};
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
        //cout<<"***Error! Wrong Phase Information.***\n";
        outputlog("***Error finding Split Phase Information.***\n");
        //system("pause");
        break;
    }

    return combined_phase;
}

// Return the first position of the request phase in the ReqTimePtList
// In case there are more vehicles request the same phases.
int RequestPhaseInList(LinkedList<ReqEntry> ReqList,int phase)
{
    int posOfReq=-1;
    int i=0;
    ReqList.Reset();
    if(!ReqList.ListEmpty())
    {
        while(!ReqList.EndOfList())
        {
            if(ReqList.Data().Phase==phase)
            {
                posOfReq=i;
                break;
            }
            else
            {
                i++;
            }

            ReqList.Next();
        }
    }

    return posOfReq;
}


int FindRingNo(int phase)
{
    int RingNo;
    switch (phase%10)
    {
    case 0:
    case 1:
    case 2:
    case 3:
        RingNo=0;
        break;
    case 4:
    case 5:
    case 6:
    case 7:
        RingNo=1;
        break;
    default:
        cout<<"******Error: No such phase!\n";
        RingNo=-1;
    }
    return RingNo;
}


// Return the earliest request in the ring with respect to both time and phase sequence 
int findTheEarliestReqInERP(LinkedList<PriorityRequest> PR)
{
	PR.Reset();
	int iTemp=-1;
	int iERP=99;
	double dEarliestArr=999;
	if(PR.ListEmpty()==0) // when list is not empty
    {
	    while (!PR.EndOfList()) 
		{
			if (PR.Data().iPhaseCycle<=iERP)
			{
				iERP=PR.Data().iPhaseCycle;
				iTemp=PR.CurrentPosition();
			}
			PR.Next();
		}
		PR.Reset();
		while (!PR.EndOfList()) 
		{
			if (PR.Data().iPhaseCycle==iERP)
			{
				if (PR.Data().dRl <= dEarliestArr)
				{
					dEarliestArr=PR.Data().dRl;
					iTemp=PR.CurrentPosition();
				}
			}
			PR.Next();
		}	
	}
	return iTemp;	
}

void creatFeasibleRegionRing1(double CP[3][5], LinkedList<PriorityRequest> PrioReqListOfRing1, int phaseInRing, double rl, double ru, double delay, int type, double *EndOfPhase,int *phaseSeq ,int t, RSU_Config ConfigIS, double globalgmax[8], double initialGreen, double ElapsedGreen, int isthereReqInotherRing)
{
	// this part just check whether there is another request in same phase as the ERP(Earliest Request Respect to Ring and Phase) !! this is helpfull when the requested phase in the current phase and we have several other request in that phase after the ERP
	PrioReqListOfRing1.Reset();
	if(PrioReqListOfRing1.ListEmpty()==0) // when list is not empty
    {
	    while (!PrioReqListOfRing1.EndOfList()) 
		{
			if (PrioReqListOfRing1.Data().iPhaseCycle==phaseInRing) 
			{
				rl=PrioReqListOfRing1.Data().dRl;
				ru=PrioReqListOfRing1.Data().dRu;
			}
			PrioReqListOfRing1.Next();
		}
	}
	
	int iRequestedPhaseInSeq=-1;
	int iTotalPlanningPhase=0;
	int iPhase; // gets 0 or 1 or 2 or 3
	int iPhaseTemp; // gets 0 or 1 or 2 or 3
	int iPhaseTemp2; // gets 0 or 1 or 2 or 3
	int iPhaseTemp3; // gets 0 or 1 or 2 or 3
	int iPhaseTemp4; // gets 0 or 1 or 2 or 3
	int iPhaseIndexInConfiguration=0;
	int iPhaseIndexInConfigurationTemp=0;
	int iPhaseIndexInConfigurationTemp2=0;
	int iPhaseIndexInConfigurationTemp3=0;
	int iPhaseIndexInConfigurationTemp4=0;
	int iTemp=0;
	int iTemp1=0;
	int iTemp2=0;
	int iTemp3=0;
	int iTemp4=0;
	for (int i=0; i<t ; i++)
	{
		if (phaseSeq[i]==phaseInRing)
		{
			iRequestedPhaseInSeq=i;
			
		}
	}
	iTemp=phaseSeq[0];
	iPhase=iTemp%10;   // current phase in formate of 0 to 3
	
	////////DEBATE HERE!!!!! about elaps time!!!!
			
	// --- Adjusting the elpase green time of coordinated phase for coordination request in order to to consider early return to green and its consequesces.
	if (type==6 && rl==1 )	 // during the split time of coordination
	{	
		ElapsedGreen=ConfigIS.iCoordinationSplit-ru;
	}		
	else if (type==6 && rl>1 && ru<ConfigIS.iCoordinationCycle-ConfigIS.iCoordinationSplit - 10 && phaseSeq[0]==phaseInRing) // during other time of the cycle (not split or yield ) , I assumed 10 is the difference between max green and split of the coordinated phase   .   Also, " phaseSeq[0]==phaseInRing  " means the requested coordination phase is the same as current phase. in fact we are in the early return to green 
	{
		ElapsedGreen=0;
	}
	
	iTotalPlanningPhase=iRequestedPhaseInSeq+1;
	cout <<" Number of Planned Phases" <<iTotalPlanningPhase<<endl;
	double dBackwardRightPoints[iTotalPlanningPhase];
	double dBackwardLeftPoints[iTotalPlanningPhase];
	double dForwardRightPoints[iTotalPlanningPhase];
	double dForwardLeftPoints[iTotalPlanningPhase];
	if (iTotalPlanningPhase==1) // if the request is in  the current phase
	{
		
		iPhaseIndexInConfiguration=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhase);
		//dBackwardRightPoints[0]=max(EndOfPhase[0],ru); //999.0;  
		cout<<"ru"<<ru<<endl;
		cout<<"EndOfPhase[1]"<<EndOfPhase[1]<<endl;
		dBackwardLeftPoints[0]=ru;
		//dForwardRightPoints[0]=ConfigIS.Gmax[iPhaseIndexInConfiguration]-ElapsedGreen+initialGreen;
		if (type==6) // for early return to green 
		{
			dForwardRightPoints[0]=globalgmax[iPhaseIndexInConfiguration];//ConfigIS.Gmax[iPhaseIndexInConfiguration];
			dBackwardRightPoints[0]=max(EndOfPhase[0],ru); //99.0;  
		}
		else
		{
			if (isthereReqInotherRing>-1)
				dBackwardRightPoints[0]=EndOfPhase[0];   // if there is request in other ring, we should follow the solution and forceoff 
			else 
				dBackwardRightPoints[0]=99.0;   // should not force off immidiately, bc there maybe regulare vehicle behind the priority vehicle
			dForwardRightPoints[0]=globalgmax[iPhaseIndexInConfiguration]-ElapsedGreen+initialGreen;   //ConfigIS.Gmax[iPhaseIndexInConfiguration]-ElapsedGreen+initialGreen;
		}
		dForwardLeftPoints[0]=max(0.0,ConfigIS.Gmin[iPhaseIndexInConfiguration]+initialGreen-ElapsedGreen);
		CP[2][0]=iPhase; 
		CP[0][0]=max(dBackwardLeftPoints[0],dForwardLeftPoints[0]);
		if (dBackwardRightPoints[0]<0)
			dBackwardRightPoints[0]=dForwardRightPoints[0];
		CP[1][0]=min(dBackwardRightPoints[0],dForwardRightPoints[0]);
		if (CP[1][0] < CP[0][0])  // Avoid the case than left point is greater than right points , this happens in case that elapse time of the current phase is more than maximum time, then the right point would be negative. so we should make it at lease thesame as left point
		{
			CP[1][0]=CP[0][0];
		}
	}else if (iTotalPlanningPhase==2)   // if the request is in the next phase
	{
		// do forward points
		
		iPhaseIndexInConfiguration=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhase);
		dForwardRightPoints[0]= globalgmax[iPhaseIndexInConfiguration]-ElapsedGreen+initialGreen;
		dForwardLeftPoints[0] = max(0.0,ConfigIS.Gmin[iPhaseIndexInConfiguration]+initialGreen-ElapsedGreen);
		iTemp1=phaseSeq[1];
		iPhaseTemp=iTemp1%10;
		iPhaseIndexInConfigurationTemp=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhaseTemp);
		dForwardRightPoints[1]= dForwardRightPoints[0]+globalgmax[iPhaseIndexInConfigurationTemp]+ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration];
		dForwardLeftPoints[1] = dForwardLeftPoints[0] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp]+ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration];
		
		// do backward points
		dBackwardRightPoints[1]=max(EndOfPhase[1],ru);//999.0;  
		if (delay!=0)
			dBackwardLeftPoints[1]=EndOfPhase[1];
		else
			dBackwardLeftPoints[1]=ru;
		if (delay==0)
			dBackwardRightPoints[0]=rl-(ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration]);
		else
			dBackwardRightPoints[0]=EndOfPhase[0];
		
		if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			dBackwardLeftPoints[0]=-1;
		else
		{
			if (delay!=0)
				dBackwardLeftPoints[0]=EndOfPhase[0]; 
			else
				dBackwardLeftPoints[0]=dBackwardLeftPoints[1]-(globalgmax[iPhaseIndexInConfigurationTemp]+ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration]);
		}
		
		// Do phases	
		CP[2][0]=iPhase;
		CP[2][1]=iPhaseTemp;
		
		// Do CPs
		CP[0][0]=max(dBackwardLeftPoints[0],dForwardLeftPoints[0]);  //dBackwardLeftPoints[0]; 
		if (dBackwardRightPoints[0]<0)
			dBackwardRightPoints[0]=dForwardRightPoints[0];
		CP[1][0]=min(dBackwardRightPoints[0],dForwardRightPoints[0]);
		if (CP[1][0] < CP[0][0])  // in case that elapse time of the current phase is more than maximum time, then the right point would be negative. so we should make it at lease thesame as left point
		{
			double iTemp=CP[0][0]-dForwardRightPoints[0];
			CP[1][0]=CP[0][0];
			dForwardRightPoints[0]=CP[0][0];
			dForwardRightPoints[1]=dForwardRightPoints[1]+iTemp;
		}
		
		CP[0][1]=max(dBackwardLeftPoints[1],dForwardLeftPoints[1]);
		if (dBackwardRightPoints[1]<0)
			dBackwardRightPoints[1]=dForwardRightPoints[1];
		CP[1][1]=min(dBackwardRightPoints[1],dForwardRightPoints[1]);
	}else if (iTotalPlanningPhase==3)   // if the request is in the next next phase
	{
		// do forward points
		
		iPhaseIndexInConfiguration=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhase);
		dForwardRightPoints[0]= globalgmax[iPhaseIndexInConfiguration]-ElapsedGreen+initialGreen;
		cout<<"dForwardRightPoints[0]"<<dForwardRightPoints[0]<<endl;
		cout<<"ElapsedGreen+initialGreen"<<ElapsedGreen+initialGreen<<endl;
		cout<<"max"<<globalgmax[iPhaseIndexInConfiguration]<<endl;
		dForwardLeftPoints[0] = max(0.0,ConfigIS.Gmin[iPhaseIndexInConfiguration]+initialGreen-ElapsedGreen);
		iTemp1=phaseSeq[1];
		iPhaseTemp=iTemp1%10;
		iPhaseIndexInConfigurationTemp=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhaseTemp);
		dForwardRightPoints[1]= dForwardRightPoints[0]+globalgmax[iPhaseIndexInConfigurationTemp]+ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration]; //gmax_{P-1} +Cl_{P-2}
		dForwardLeftPoints[1] = dForwardLeftPoints[0] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp]+ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration];
		iTemp2=phaseSeq[2];
		iPhaseTemp2=iTemp2%10;
		iPhaseIndexInConfigurationTemp2=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhaseTemp2);
		dForwardRightPoints[2]= dForwardRightPoints[1]+globalgmax[iPhaseIndexInConfigurationTemp2]+ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp];//gmax_{P} +Cl_{P-1}
		dForwardLeftPoints[2] = dForwardLeftPoints[1] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp2]+ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp];
		// do backward points
		dBackwardRightPoints[2]=max(EndOfPhase[2],ru);//999.0;  
		if (delay!=0)
			dBackwardLeftPoints[2]=EndOfPhase[2];
		else
			dBackwardLeftPoints[2]=ru;
		
		//dBackwardLeftPoints[2]=EndOfPhase[2];
		if (delay==0)
			dBackwardRightPoints[1]=rl-(ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp]); //rl-cl_{p-1}
		else
			dBackwardRightPoints[1]=EndOfPhase[1];
		
		dBackwardRightPoints[0]=dBackwardRightPoints[1]-(ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[0]=dBackwardRightPoints[1]-globalgmax[iPhaseIndexInConfigurationTemp]-(ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration]);
		// Do phases
		CP[2][0]=iPhase;
		CP[2][1]=iPhaseTemp;
		CP[2][2]=iPhaseTemp2;
		// Do CPs
		CP[0][0]=max(dBackwardLeftPoints[0],dForwardLeftPoints[0]);
		if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			dBackwardLeftPoints[1]=CP[0][0]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2]+ConfigIS.Gmin[iPhaseIndexInConfigurationTemp];
		else
		{
			if (delay!=0)
				dBackwardLeftPoints[1]=EndOfPhase[1]; 
			else
				dBackwardLeftPoints[1]=dBackwardLeftPoints[2]-globalgmax[iPhaseIndexInConfigurationTemp2]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp]);
		}
						
		if (dBackwardRightPoints[0]<0)
			dBackwardRightPoints[0]=dForwardRightPoints[0];
		CP[1][0]=min(dBackwardRightPoints[0],dForwardRightPoints[0]);
		if (CP[1][0] < CP[0][0])  // in case that elapse time of the current phase is more than maximum time, then the right point would be negative. so we should make it at lease thesame as left point
		{
			double iTemp=CP[0][0]-dForwardRightPoints[0];
			CP[1][0]=CP[0][0];
			dForwardRightPoints[0]=CP[0][0];
			dForwardRightPoints[1]=dForwardRightPoints[1]+iTemp;
			dForwardRightPoints[2]=dForwardRightPoints[2]+iTemp;
		}
		CP[0][1]=max(dBackwardLeftPoints[1],dForwardLeftPoints[1]);
		if (dBackwardRightPoints[1]<0)
			dBackwardRightPoints[1]=dForwardRightPoints[1];
		CP[1][1]=min(dBackwardRightPoints[1],dForwardRightPoints[1]);
		CP[0][2]=max(dBackwardLeftPoints[2],dForwardLeftPoints[2]);
		if (dBackwardRightPoints[2]<0)
			dBackwardRightPoints[2]=dForwardRightPoints[2];
		CP[1][2]=min(dBackwardRightPoints[2],dForwardRightPoints[2]);
	}else if (iTotalPlanningPhase==4)   // if the request is in the next next next phase
	{
		// do forward points
		
		iPhaseIndexInConfiguration=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhase);
		dForwardRightPoints[0]= globalgmax[iPhaseIndexInConfiguration]-ElapsedGreen+initialGreen;
		dForwardLeftPoints[0] = max(0.0,ConfigIS.Gmin[iPhaseIndexInConfiguration]+initialGreen-ElapsedGreen);
		iTemp1=phaseSeq[1];
		iPhaseTemp=iTemp1%10;
		iPhaseIndexInConfigurationTemp=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhaseTemp);
		dForwardRightPoints[1]= dForwardRightPoints[0]+globalgmax[iPhaseIndexInConfigurationTemp]+ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration]; //gmax_{P-2} +Cl_{P-3}
		dForwardLeftPoints[1] = dForwardLeftPoints[0] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp]+ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration];
		iTemp2=phaseSeq[2];
		iPhaseTemp2=iTemp2%10;
		iPhaseIndexInConfigurationTemp2=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhaseTemp2);
		dForwardRightPoints[2]= dForwardRightPoints[1]+globalgmax[iPhaseIndexInConfigurationTemp2]+ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp];//gmax_{P-1} +Cl_{P-2}
		dForwardLeftPoints[2] = dForwardLeftPoints[1] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp2]+ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp];
		iTemp3=phaseSeq[3];
		iPhaseTemp3=iTemp3%10;
		iPhaseIndexInConfigurationTemp3=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhaseTemp3);
		dForwardRightPoints[3]= dForwardRightPoints[2]+globalgmax[iPhaseIndexInConfigurationTemp3]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2];//gmax_{P} +Cl_{P-1}
		dForwardLeftPoints[3] = dForwardLeftPoints[2] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp3]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2];
		
		// do backward points
		dBackwardRightPoints[3]=max(EndOfPhase[3],ru);//999.0;  
		dBackwardLeftPoints[3]=EndOfPhase[3];
		if (delay==0)
			dBackwardRightPoints[2]=rl-(ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2]); //rl-cl_{p-1}
		else
			dBackwardRightPoints[2]=EndOfPhase[2];
		//~ if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			//~ dBackwardLeftPoints[2]=-1;
		//~ else
			//~ dBackwardLeftPoints[2]=dBackwardLeftPoints[3]-ConfigIS.Gmax[iPhaseIndexInConfigurationTemp3]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2]);
		dBackwardRightPoints[1]=dBackwardRightPoints[2]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp2];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[1]=dBackwardRightPoints[2]-globalgmax[iPhaseIndexInConfigurationTemp2]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp]);
		dBackwardRightPoints[0]=dBackwardRightPoints[1]-(ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[0]=dBackwardRightPoints[1]-globalgmax[iPhaseIndexInConfigurationTemp]-(ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration]);
		//Do phases
		CP[2][0]=iPhase;
		CP[2][1]=iPhaseTemp;
		CP[2][2]=iPhaseTemp2;
		CP[2][3]=iPhaseTemp3;
		// Do CPs
		CP[0][0]=max(dBackwardLeftPoints[0],dForwardLeftPoints[0]);
		if (dBackwardRightPoints[0]<0)
			dBackwardRightPoints[0]=dForwardRightPoints[0];
		CP[1][0]=min(dBackwardRightPoints[0],dForwardRightPoints[0]);
		if (CP[1][0]< CP[0][0])  // in case that elapse time of the current phase is more than maximum time, then the right point would be negative. so we should make it at lease thesame as left point
		{
			double iTemp=CP[0][0]-dForwardRightPoints[0];
			CP[1][0]=CP[0][0];
			dForwardRightPoints[0]=CP[0][0];
			dForwardRightPoints[1]=dForwardRightPoints[1]+iTemp;
			dForwardRightPoints[2]=dForwardRightPoints[2]+iTemp;
			dForwardRightPoints[3]=dForwardRightPoints[3]+iTemp;
		}
		CP[0][1]=max(dBackwardLeftPoints[1],dForwardLeftPoints[1]);
		if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			dBackwardLeftPoints[2]=CP[0][1]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2]+ConfigIS.Gmin[iPhaseIndexInConfigurationTemp];
		else
			dBackwardLeftPoints[2]=dBackwardLeftPoints[3]-globalgmax[iPhaseIndexInConfigurationTemp3]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2]);
	
		if (dBackwardRightPoints[1]<0)
			dBackwardRightPoints[1]=dForwardRightPoints[1];
		CP[1][1]=min(dBackwardRightPoints[1],dForwardRightPoints[1]);
		CP[0][2]=max(dBackwardLeftPoints[2],dForwardLeftPoints[2]);
		if (dBackwardRightPoints[2]<0)
			dBackwardRightPoints[2]=dForwardRightPoints[2];
		CP[1][2]=min(dBackwardRightPoints[2],dForwardRightPoints[2]);
		CP[0][3]=max(dBackwardLeftPoints[3],dForwardLeftPoints[3]);
		if (dBackwardRightPoints[3]<0)
			dBackwardRightPoints[3]=dForwardRightPoints[3];
		CP[1][3]=min(dBackwardRightPoints[3],dForwardRightPoints[3]);
	}else if (iTotalPlanningPhase==5)   // if the request is in the next 4 phase
	{
		// do forward points
		
		iPhaseIndexInConfiguration=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhase);
		dForwardRightPoints[0]= globalgmax[iPhaseIndexInConfiguration]-ElapsedGreen+initialGreen;
		dForwardLeftPoints[0] = max(0.0,ConfigIS.Gmin[iPhaseIndexInConfiguration]+initialGreen-ElapsedGreen);
		iTemp1=phaseSeq[1];
		iPhaseTemp=iTemp1%10;
		iPhaseIndexInConfigurationTemp=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhaseTemp);
		dForwardRightPoints[1]= dForwardRightPoints[0]+globalgmax[iPhaseIndexInConfigurationTemp]+ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration]; //gmax_{P-2} +Cl_{P-3}
		dForwardLeftPoints[1] = dForwardLeftPoints[0] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp]+ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration];
		iTemp2=phaseSeq[2];
		iPhaseTemp2=iTemp2%10;
		iPhaseIndexInConfigurationTemp2=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhaseTemp2);
		dForwardRightPoints[2]= dForwardRightPoints[1]+globalgmax[iPhaseIndexInConfigurationTemp2]+ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp];//gmax_{P-1} +Cl_{P-2}
		dForwardLeftPoints[2] = dForwardLeftPoints[1] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp2]+ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp];
		iTemp3=phaseSeq[3];
		iPhaseTemp3=iTemp3%10;
		iPhaseIndexInConfigurationTemp3=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhaseTemp3);
		dForwardRightPoints[3]= dForwardRightPoints[2]+globalgmax[iPhaseIndexInConfigurationTemp3]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2];//gmax_{P} +Cl_{P-1}
		dForwardLeftPoints[3] = dForwardLeftPoints[2] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp3]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2];
		iTemp4=phaseSeq[4];
		iPhaseTemp4=iTemp4%10;
		iPhaseIndexInConfigurationTemp4=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,ConfigIS.Ring1No,iPhaseTemp4);
		dForwardRightPoints[4]= dForwardRightPoints[3]+globalgmax[iPhaseIndexInConfigurationTemp4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp3]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp3];//gmax_{P} +Cl_{P-1}
		dForwardLeftPoints[4] = dForwardLeftPoints[3] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp3]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp3];
		
		// do backward points
		dBackwardRightPoints[4]=max(EndOfPhase[4],ru);//999.0;  
		dBackwardLeftPoints[4]=EndOfPhase[4];
		if (delay==0)
			dBackwardRightPoints[3]=rl-(ConfigIS.Red[iPhaseIndexInConfigurationTemp3]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp3]); //rl-cl_{p-1}
		else
			dBackwardRightPoints[3]=EndOfPhase[3];
		//~ if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			//~ dBackwardLeftPoints[3]=-1;
		//~ else
			//~ dBackwardLeftPoints[3]=dBackwardLeftPoints[4]-ConfigIS.Gmax[iPhaseIndexInConfigurationTemp4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp3]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp3]);
		dBackwardRightPoints[2]=dBackwardRightPoints[3]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp3];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[2]=dBackwardRightPoints[3]-globalgmax[iPhaseIndexInConfigurationTemp3]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2]);
		dBackwardRightPoints[1]=dBackwardRightPoints[2]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp2];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[1]=dBackwardRightPoints[2]-globalgmax[iPhaseIndexInConfigurationTemp2]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp]);
		dBackwardRightPoints[0]=dBackwardRightPoints[1]-(ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[0]=dBackwardRightPoints[1]-globalgmax[iPhaseIndexInConfigurationTemp]-(ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration]);
		
		// Do phases
		CP[2][0]=iPhase;
		CP[2][1]=iPhaseTemp;
		CP[2][2]=iPhaseTemp2;
		CP[2][3]=iPhaseTemp3;
		CP[2][4]=iPhaseTemp4;
		// Do CPs
		CP[0][0]=max(dBackwardLeftPoints[0],dForwardLeftPoints[0]);
		//~ cout<<"EndOfPhase[4]"<<EndOfPhase[0]<<endl;
		//~ cout<<"dBackwardLeftPoints[0]"<<dBackwardLeftPoints[0]<<endl;
		//~ cout<<"dBackwardRightPoints[1]   "<< dBackwardRightPoints[1]<<endl; 
		//~ cout<<"ConfigIS.Gmax[iPhaseIndexInConfigurationTemp]  "<<ConfigIS.Gmax[iPhaseIndexInConfigurationTemp]<<endl;
		//~ cout<<"dForwardLeftPoints[0] "<<dForwardLeftPoints[0]<<endl;
		//~ cout<<"dForwardRightPoints[0] "<<dForwardRightPoints[0]<<endl;
		//~ cout<<"dForwardRightPoints[1] "<<dForwardRightPoints[1]<<endl;
		//~ cout<<"dBackwardLeftPoints[4]"<<dBackwardLeftPoints[4]<<endl;
		//~ cout<<"dForwardLeftPoints[4] "<<dForwardLeftPoints[4]<<endl;
		
		if (dBackwardRightPoints[0]<0)
			dBackwardRightPoints[0]=dForwardRightPoints[0];
		CP[1][0]=min(dBackwardRightPoints[0],dForwardRightPoints[0]);
		if (CP[1][0] < CP[0][0])  // in case that elapse time of the current phase is more than maximum time, then the right point would be negative. so we should make it at lease thesame as left point
		{
			double iTemp=CP[0][0]-dForwardRightPoints[0];
			CP[1][0]=CP[0][0];
			dForwardRightPoints[0]=CP[0][0];
			dForwardRightPoints[1]=dForwardRightPoints[1]+iTemp;
			dForwardRightPoints[2]=dForwardRightPoints[2]+iTemp;
			dForwardRightPoints[3]=dForwardRightPoints[3]+iTemp;
			dForwardRightPoints[4]=dForwardRightPoints[4]+iTemp;
		}
		CP[0][1]=max(dBackwardLeftPoints[1],dForwardLeftPoints[1]);
		if (dBackwardRightPoints[1]<0)
			dBackwardRightPoints[1]=dForwardRightPoints[1];
		CP[1][1]=min(dBackwardRightPoints[1],dForwardRightPoints[1]);
		CP[0][2]=max(dBackwardLeftPoints[2],dForwardLeftPoints[2]);
		if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			dBackwardLeftPoints[3]=CP[0][2]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2]+ConfigIS.Gmin[iPhaseIndexInConfigurationTemp];
		else
			dBackwardLeftPoints[3]=dBackwardLeftPoints[4]-globalgmax[iPhaseIndexInConfigurationTemp4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp3]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp3]);
	
		if (dBackwardRightPoints[2]<0)
			dBackwardRightPoints[2]=dForwardRightPoints[2];
		CP[1][2]=min(dBackwardRightPoints[2],dForwardRightPoints[2]);
		CP[0][3]=max(dBackwardLeftPoints[3],dForwardLeftPoints[3]);
		if (dBackwardRightPoints[3]<0)
			dBackwardRightPoints[3]=dForwardRightPoints[3];
		CP[1][3]=min(dBackwardRightPoints[3],dForwardRightPoints[3]);
		CP[0][4]=max(dBackwardLeftPoints[4],dForwardLeftPoints[4]);
		if (dBackwardRightPoints[4]<0)
			dBackwardRightPoints[4]=dForwardRightPoints[4];
		CP[1][4]=min(dBackwardRightPoints[4],dForwardRightPoints[4]);
	}
	 cout<<"curent Phase"<<iPhase<<endl;
	//~ cout<<"iPhaseTemp"<<iPhaseTemp<<endl;
	for (int ii=0;ii<iTotalPlanningPhase;ii++)
	{
		cout<<"dForwardLeftPoints[  "<< ii << " "<< dForwardLeftPoints[ii]<<endl; 
		cout<<"dBackwardLeftPoints[ "<< ii << " "<< dBackwardLeftPoints[ii]<<endl;
		cout<<"CP left              "<< ii << " "<< CP[0][ii]<<endl;
		cout<<"dForwardRightPoints[ "<< ii << " "<< dForwardRightPoints[ii]<<endl; 
		cout<<"dBackwardRightPoints["<< ii << " "<< dBackwardRightPoints[ii]<<endl;
		cout<<"CP right             "<< ii << " "<< CP[1][ii]<<endl;
		cout<< "phase "<< CP[2][ii]<<endl;
	}
		
}

void creatFeasibleRegionRing2(double  CP[3][5], LinkedList<PriorityRequest> PrioReqListOfRing2, int phaseInRing, double rl, double ru, double delay, int type, double *EndOfPhase,int *phaseSeq ,int t, RSU_Config ConfigIS,double globalgmax[8], double initialGreen, double ElapsedGreen, int isThereReqInOtherRing)
{
	// this part just check whether there is another request in same phase as the ERP(Earliest Request Respect to Ring and Phase) !! this is helpfull when the requested phase in the current phase and we have several other request in that phase after the ERP
	PrioReqListOfRing2.Reset();
	if(PrioReqListOfRing2.ListEmpty()==0) // when list is not empty
    {
	    while (!PrioReqListOfRing2.EndOfList()) 
		{
			if (PrioReqListOfRing2.Data().iPhaseCycle==phaseInRing) 
			{
				rl=PrioReqListOfRing2.Data().dRl;
				ru=PrioReqListOfRing2.Data().dRu;
			}
			PrioReqListOfRing2.Next();
		}
	}
	
	
	int iRequestedPhaseInSeq=-1;
	int iTotalPlanningPhase=0;
	int iPhase; // gets 0 or 1 or 2 or 3
	int iPhaseTemp; // gets 0 or 1 or 2 or 3
	int iPhaseTemp2; // gets 0 or 1 or 2 or 3
	int iPhaseTemp3; // gets 0 or 1 or 2 or 3
	int iPhaseTemp4; // gets 0 or 1 or 2 or 3
	int iPhaseIndexInConfiguration=0;
	int iPhaseIndexInConfigurationTemp=0;
	int iPhaseIndexInConfigurationTemp2=0;
	int iPhaseIndexInConfigurationTemp3=0;
	int iPhaseIndexInConfigurationTemp4=0;
	int iTemp=0;
	int iTemp1=0;
	int iTemp2=0;
	int iTemp3=0;
	int iTemp4=0;
	for (int i=0; i<t ; i++)
	{
		if (phaseSeq[i]==phaseInRing)
		{
			iRequestedPhaseInSeq=i;
			
		}
	}
	
	
	iTemp=phaseSeq[0];
	iPhase=iTemp%10;   // current phase in formate of 0 to 3
	
	////////DEBATE HERE!!!!! about elaps time!!!!
			
	// --- Adjusting the elpase green time of coordinated phase for coordination request in order to to consider early return to green and its consequesces.
	if (type==6 && rl==1 )	 // during the split time of coordination
	{	
		ElapsedGreen=ConfigIS.iCoordinationSplit-ru;
	}		
	else if (type==6 && rl>1 && ru<ConfigIS.iCoordinationCycle-ConfigIS.iCoordinationSplit - 10 && phaseSeq[0]==phaseInRing) // during other time of the cycle (not split or yield ) , I assumed 10 is the difference between max green and split of the coordinated phase   .   Also, " phaseSeq[0]==phaseInRing  " means the requested coordination phase is the same as current phase. in fact we are in the early return to green 
	{
		ElapsedGreen=0;
	}
	
	iTotalPlanningPhase=iRequestedPhaseInSeq+1;
	cout <<" Number of Planned Phases" <<iTotalPlanningPhase<<endl;
	double dBackwardRightPoints[iTotalPlanningPhase];
	double dBackwardLeftPoints[iTotalPlanningPhase];
	double dForwardRightPoints[iTotalPlanningPhase];
	double dForwardLeftPoints[iTotalPlanningPhase];
	if (iTotalPlanningPhase==1) // if the request is in  the current phase
	{
		
		iPhaseIndexInConfiguration=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhase);
		//dBackwardRightPoints[0]=max(EndOfPhase[0],ru); //999.0;  
		dBackwardLeftPoints[0]=ru;
		//dForwardRightPoints[0]=ConfigIS.Gmax[iPhaseIndexInConfiguration+4]-ElapsedGreen+initialGreen;
		if (type==6) // for early return to green 
		{
			dForwardRightPoints[0]=globalgmax[iPhaseIndexInConfiguration]; //ConfigIS.Gmax[iPhaseIndexInConfiguration];
			dBackwardRightPoints[0]=max(EndOfPhase[0],ru); //99.0;  
		}
		else
		{
			if (isThereReqInOtherRing>-1)
				dBackwardRightPoints[0]=EndOfPhase[0];   // if there is request in other ring, we should follow the solution and forceoff 
			else 
				dBackwardRightPoints[0]=99.0;   // should not force off immidiately, bc there maybe regulare vehicle behind the priority vehicle
			dForwardRightPoints[0]=globalgmax[iPhaseIndexInConfiguration]-ElapsedGreen+initialGreen;   //ConfigIS.Gmax[iPhaseIndexInConfiguration]-ElapsedGreen+initialGreen;
		}
		dForwardLeftPoints[0]=max(0.0,ConfigIS.Gmin[iPhaseIndexInConfiguration+4]+initialGreen-ElapsedGreen);
		CP[2][0]=iPhase;
		CP[0][0]=max(dBackwardLeftPoints[0],dForwardLeftPoints[0]);
		if (dBackwardRightPoints[0]<0)
			dBackwardRightPoints[0]=dForwardRightPoints[0];
		CP[1][0]=min(dBackwardRightPoints[0],dForwardRightPoints[0]);
		if (CP[1][0] < CP[0][0])  // Avoid the case than left point is greater than right points , this happens in case that elapse time of the current phase is more than maximum time, then the right point would be negative. so we should make it at lease thesame as left point
		{
			CP[1][0]=CP[0][0];
		}
	}else if (iTotalPlanningPhase==2)   // if the request is in the next phase
	{
		// do forward points
		
		iPhaseIndexInConfiguration=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhase);
		dForwardRightPoints[0]= globalgmax[iPhaseIndexInConfiguration+4]-ElapsedGreen+initialGreen;
		dForwardLeftPoints[0] = max(0.0,ConfigIS.Gmin[iPhaseIndexInConfiguration+4]+initialGreen-ElapsedGreen);
		iTemp1=phaseSeq[1];
		iPhaseTemp=iTemp1%10;
		iPhaseIndexInConfigurationTemp=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhaseTemp);
		dForwardRightPoints[1]= dForwardRightPoints[0]+globalgmax[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4];
		dForwardLeftPoints[1] = dForwardLeftPoints[0] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4];
		
		// do backward points
		if (delay!=0)
			dBackwardLeftPoints[1]=EndOfPhase[1];
		else
			dBackwardLeftPoints[1]=ru;
		dBackwardRightPoints[1]=max(EndOfPhase[1],ru); //999.0;  
		//dBackwardLeftPoints[1]=EndOfPhase[1];
		if (delay==0)
			dBackwardRightPoints[0]=rl-(ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4]);
		else
			dBackwardRightPoints[0] = EndOfPhase[0];
		if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			dBackwardLeftPoints[0]=-1;
		else
		{
			if (delay!=0)
				dBackwardLeftPoints[0]=EndOfPhase[0]; 
			else
				dBackwardLeftPoints[0]=dBackwardLeftPoints[1]-(globalgmax[iPhaseIndexInConfigurationTemp]+ConfigIS.Red[iPhaseIndexInConfiguration]+ConfigIS.Yellow[iPhaseIndexInConfiguration]);
		}
		
		// Do Phases
		CP[2][0]=iPhase;
		CP[2][1]=iPhaseTemp;
		// Do CPS
		CP[0][0]=max(dBackwardLeftPoints[0],dForwardLeftPoints[0]);  //dBackwardLeftPoints[0]; 
		if (dBackwardRightPoints[0]<0)
			dBackwardRightPoints[0]=dForwardRightPoints[0];
		CP[1][0]=min(dBackwardRightPoints[0],dForwardRightPoints[0]);
		
		if (CP[1][0] < CP[0][0])  // in case that elapse time of the current phase is more than maximum time, then the right point would be negative. so we should make it at lease thesame as left point
		{
			double iTemp=CP[0][0]-dForwardRightPoints[0];
			CP[1][0]=CP[0][0];
			dForwardRightPoints[0]=CP[0][0];
			dForwardRightPoints[1]=dForwardRightPoints[1]+iTemp;
		}
		
		CP[0][1]=max(dBackwardLeftPoints[1],dForwardLeftPoints[1]);
		if (dBackwardRightPoints[1]<0)
			dBackwardRightPoints[1]=dForwardRightPoints[1];
		CP[1][1]=min(dBackwardRightPoints[1],dForwardRightPoints[1]);
	}else if (iTotalPlanningPhase==3)   // if the request is in the next next phase
	{
		// do forward points
		
		iPhaseIndexInConfiguration=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhase);
		dForwardRightPoints[0]= globalgmax[iPhaseIndexInConfiguration+4]-ElapsedGreen+initialGreen;
		dForwardLeftPoints[0] = max(0.0,ConfigIS.Gmin[iPhaseIndexInConfiguration+4]+initialGreen-ElapsedGreen);
		iTemp1=phaseSeq[1];
		iPhaseTemp=iTemp1%10;
		iPhaseIndexInConfigurationTemp=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhaseTemp);
		dForwardRightPoints[1]= dForwardRightPoints[0]+globalgmax[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4]; //gmax_{P-1} +Cl_{P-2}
		dForwardLeftPoints[1] = dForwardLeftPoints[0] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4];
		iTemp2=phaseSeq[2];
		iPhaseTemp2=iTemp2%10;
		iPhaseIndexInConfigurationTemp2=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhaseTemp2);
		dForwardRightPoints[2]= dForwardRightPoints[1]+globalgmax[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4];//gmax_{P} +Cl_{P-1}
		dForwardLeftPoints[2] = dForwardLeftPoints[1] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4];
		// do backward points
		dBackwardRightPoints[2]=max(EndOfPhase[2],ru);//999.0;  
		
		if (delay!=0)
			dBackwardLeftPoints[2]=EndOfPhase[2];
		else
			dBackwardLeftPoints[2]=ru;
		if (delay==0)
			dBackwardRightPoints[1]=rl-(ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4]); //rl-cl_{p-1}
		else
			dBackwardRightPoints[1]=EndOfPhase[1];
		//~ if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			//~ dBackwardLeftPoints[1]=-1;
		//~ else
			//~ dBackwardLeftPoints[1]=dBackwardLeftPoints[2]-ConfigIS.Gmax[iPhaseIndexInConfigurationTemp2+4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4]);
		dBackwardRightPoints[0]=dBackwardRightPoints[1]-(ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp+4];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[0]=dBackwardRightPoints[1]-globalgmax[iPhaseIndexInConfigurationTemp+4]-(ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4]);;
		
		// Do Phases
		CP[2][0]=iPhase;
		CP[2][1]=iPhaseTemp;
		CP[2][2]=iPhaseTemp2;
		
		// Do CPs
		CP[0][0]=max(dBackwardLeftPoints[0],dForwardLeftPoints[0]);
		if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			dBackwardLeftPoints[1]=CP[0][0]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Gmin[iPhaseIndexInConfigurationTemp+4];
		else
		{
			if (delay!=0)
				dBackwardLeftPoints[1]=EndOfPhase[1]; 
			else
				dBackwardLeftPoints[1]=dBackwardLeftPoints[2]-globalgmax[iPhaseIndexInConfigurationTemp2+4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4]);
		}
		if (dBackwardRightPoints[0]<0)
			dBackwardRightPoints[0]=dForwardRightPoints[0];
		CP[1][0]=min(dBackwardRightPoints[0],dForwardRightPoints[0]);
		if (CP[1][0] < CP[0][0])  // in case that elapse time of the current phase is more than maximum time, then the right point would be negative. so we should make it at lease thesame as left point
		{
			double iTemp=CP[0][0]-dForwardRightPoints[0];
			CP[1][0]=CP[0][0];
			dForwardRightPoints[0]=CP[0][0];
			dForwardRightPoints[1]=dForwardRightPoints[1]+iTemp;
			dForwardRightPoints[2]=dForwardRightPoints[2]+iTemp;
		}
		CP[0][1]=max(dBackwardLeftPoints[1],dForwardLeftPoints[1]);
		if (dBackwardRightPoints[1]<0)
			dBackwardRightPoints[1]=dForwardRightPoints[1];
		CP[1][1]=min(dBackwardRightPoints[1],dForwardRightPoints[1]);
		CP[0][2]=max(dBackwardLeftPoints[2],dForwardLeftPoints[2]);
		if (dBackwardRightPoints[2]<0)
			dBackwardRightPoints[2]=dForwardRightPoints[2];
		CP[1][2]=min(dBackwardRightPoints[2],dForwardRightPoints[2]);
		
	}else if (iTotalPlanningPhase==4)   // if the request is in the next next next phase
	{
		// do forward points
		
		iPhaseIndexInConfiguration=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhase);
		dForwardRightPoints[0]= globalgmax[iPhaseIndexInConfiguration+4]-ElapsedGreen+initialGreen;
		dForwardLeftPoints[0] = max(0.0,ConfigIS.Gmin[iPhaseIndexInConfiguration+4]+initialGreen-ElapsedGreen);
		iTemp1=phaseSeq[1];
		iPhaseTemp=iTemp1%10;
		iPhaseIndexInConfigurationTemp=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhaseTemp);
		dForwardRightPoints[1]= dForwardRightPoints[0]+globalgmax[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4]; //gmax_{P-2} +Cl_{P-3}
		dForwardLeftPoints[1] = dForwardLeftPoints[0] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4];
		iTemp2=phaseSeq[2];
		iPhaseTemp2=iTemp2%10;
		iPhaseIndexInConfigurationTemp2=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhaseTemp2);
		dForwardRightPoints[2]= dForwardRightPoints[1]+globalgmax[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4];//gmax_{P-1} +Cl_{P-2}
		dForwardLeftPoints[2] = dForwardLeftPoints[1] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4];
		iTemp3=phaseSeq[3];
		iPhaseTemp3=iTemp3%10;
		iPhaseIndexInConfigurationTemp3=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhaseTemp3);
		dForwardRightPoints[3]= dForwardRightPoints[2]+globalgmax[iPhaseIndexInConfigurationTemp3+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4];//gmax_{P} +Cl_{P-1}
		dForwardLeftPoints[3] = dForwardLeftPoints[2] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp3+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4];
		
		// do backward points
		dBackwardRightPoints[3]=max(EndOfPhase[3],ru);//999.0;  
		dBackwardLeftPoints[3]=EndOfPhase[3];
		if (delay==0)
			dBackwardRightPoints[2]=rl-(ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4]); //rl-cl_{p-1}
		else
			dBackwardRightPoints[2]=EndOfPhase[2];
		//~ if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			//~ dBackwardLeftPoints[2]=-1;
		//~ else
			//~ dBackwardLeftPoints[2]=dBackwardLeftPoints[3]-ConfigIS.Gmax[iPhaseIndexInConfigurationTemp3+4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4]);
		dBackwardRightPoints[1]=dBackwardRightPoints[2]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp2+4];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[1]=dBackwardRightPoints[2]-globalgmax[iPhaseIndexInConfigurationTemp2+4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4]);
		dBackwardRightPoints[0]=dBackwardRightPoints[1]-(ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp+4];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[0]=dBackwardRightPoints[1]-globalgmax[iPhaseIndexInConfigurationTemp+4]-(ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4]);
		
		// Do Phases
		CP[2][0]=iPhase;
		CP[2][1]=iPhaseTemp;
		CP[2][2]=iPhaseTemp2;
		CP[2][3]=iPhaseTemp3;
		// Do CPs
		CP[0][0]=max(dBackwardLeftPoints[0],dForwardLeftPoints[0]);
		if (dBackwardRightPoints[0]<0)
			dBackwardRightPoints[0]=dForwardRightPoints[0];
		CP[1][0]=min(dBackwardRightPoints[0],dForwardRightPoints[0]);
		if (CP[1][0] < CP[0][0])  // in case that elapse time of the current phase is more than maximum time, then the right point would be negative. so we should make it at lease thesame as left point
		{
			double iTemp=CP[0][0]-dForwardRightPoints[0];
			CP[1][0]=CP[0][0];
			dForwardRightPoints[0]=CP[0][0];
			dForwardRightPoints[1]=dForwardRightPoints[1]+iTemp;
			dForwardRightPoints[2]=dForwardRightPoints[2]+iTemp;
			dForwardRightPoints[3]=dForwardRightPoints[3]+iTemp;
		}
		
		CP[0][1]=max(dBackwardLeftPoints[1],dForwardLeftPoints[1]);
		if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			dBackwardLeftPoints[2]=CP[0][1]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Gmin[iPhaseIndexInConfigurationTemp+4];
		else
			dBackwardLeftPoints[2]=dBackwardLeftPoints[3]-globalgmax[iPhaseIndexInConfigurationTemp3+4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4]);
	
		if (dBackwardRightPoints[1]<0)
			dBackwardRightPoints[1]=dForwardRightPoints[1];
		CP[1][1]=min(dBackwardRightPoints[1],dForwardRightPoints[1]);
		CP[0][2]=max(dBackwardLeftPoints[2],dForwardLeftPoints[2]);
		if (dBackwardRightPoints[2]<0)
			dBackwardRightPoints[2]=dForwardRightPoints[2];
		CP[1][2]=min(dBackwardRightPoints[2],dForwardRightPoints[2]);
		CP[0][3]=max(dBackwardLeftPoints[3],dForwardLeftPoints[3]);
		if (dBackwardRightPoints[3]<0)
			dBackwardRightPoints[3]=dForwardRightPoints[3];
		CP[1][3]=min(dBackwardRightPoints[3],dForwardRightPoints[3]);
		
	}else if (iTotalPlanningPhase==5)   // if the request is in the next 4 phase
	{
		// do forward points
		
		iPhaseIndexInConfiguration=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhase);
		dForwardRightPoints[0]= globalgmax[iPhaseIndexInConfiguration+4]-ElapsedGreen+initialGreen;
		dForwardLeftPoints[0] = max(0.0,ConfigIS.Gmin[iPhaseIndexInConfiguration+4]+initialGreen-ElapsedGreen);
		iTemp1=phaseSeq[1];
		iPhaseTemp=iTemp1%10;
		iPhaseIndexInConfigurationTemp=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhaseTemp);
		dForwardRightPoints[1]= dForwardRightPoints[0]+globalgmax[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4]; //gmax_{P-2} +Cl_{P-3}
		dForwardLeftPoints[1] = dForwardLeftPoints[0] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4];
		iTemp2=phaseSeq[2];
		iPhaseTemp2=iTemp2%10;
		iPhaseIndexInConfigurationTemp2=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhaseTemp2);
		dForwardRightPoints[2]= dForwardRightPoints[1]+globalgmax[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4];//gmax_{P-1} +Cl_{P-2}
		dForwardLeftPoints[2] = dForwardLeftPoints[1] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4];
		iTemp3=phaseSeq[3];
		iPhaseTemp3=iTemp3%10;
		iPhaseIndexInConfigurationTemp3=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhaseTemp3);
		dForwardRightPoints[3]= dForwardRightPoints[2]+globalgmax[iPhaseIndexInConfigurationTemp3+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4];//gmax_{P} +Cl_{P-1}
		dForwardLeftPoints[3] = dForwardLeftPoints[2] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp3+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4];
		iTemp4=phaseSeq[4];
		iPhaseTemp4=iTemp4%10;
		iPhaseIndexInConfigurationTemp4=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,ConfigIS.Ring2No,iPhaseTemp4);
		dForwardRightPoints[4]= dForwardRightPoints[3]+globalgmax[iPhaseIndexInConfigurationTemp4+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp3+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp3+4];//gmax_{P} +Cl_{P-1}
		dForwardLeftPoints[4] = dForwardLeftPoints[3] +ConfigIS.Gmin[iPhaseIndexInConfigurationTemp4+4]+ConfigIS.Red[iPhaseIndexInConfigurationTemp3+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp3+4];
		
		// do backward points
		dBackwardRightPoints[4]=max(EndOfPhase[4],ru);//999.0;  
		dBackwardLeftPoints[4]=EndOfPhase[4];
		if (delay==0)
			dBackwardRightPoints[3]=rl-(ConfigIS.Red[iPhaseIndexInConfigurationTemp3+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp3+4]); //rl-cl_{p-1}
		else
			dBackwardRightPoints[3]=EndOfPhase[3];
		//~ if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			//~ dBackwardLeftPoints[3]=-1;
		//~ else
			//~ dBackwardLeftPoints[3]=dBackwardLeftPoints[4]-ConfigIS.Gmax[iPhaseIndexInConfigurationTemp4+4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp3+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp3+4]);
		dBackwardRightPoints[2]=dBackwardRightPoints[3]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp3+4];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[2]=dBackwardRightPoints[3]-globalgmax[iPhaseIndexInConfigurationTemp3+4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4]);
		dBackwardRightPoints[1]=dBackwardRightPoints[2]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp2+4];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[1]=dBackwardRightPoints[2]-globalgmax[iPhaseIndexInConfigurationTemp2+4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp+4]);
		dBackwardRightPoints[0]=dBackwardRightPoints[1]-(ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4])-ConfigIS.Gmin[iPhaseIndexInConfigurationTemp+4];//-cl_{p-2}-gmin_{p-1}
		dBackwardLeftPoints[0]=dBackwardRightPoints[1]-globalgmax[iPhaseIndexInConfigurationTemp+4]-(ConfigIS.Red[iPhaseIndexInConfiguration+4]+ConfigIS.Yellow[iPhaseIndexInConfiguration+4]);
		
		//Do Phases
		CP[2][0]=iPhase;
		CP[2][1]=iPhaseTemp;
		CP[2][2]=iPhaseTemp2;
		CP[2][3]=iPhaseTemp3;
		CP[2][4]=iPhaseTemp4;
		
		// Do CPs
		CP[0][0]=max(dBackwardLeftPoints[0],dForwardLeftPoints[0]);
		if (dBackwardRightPoints[0]<0)
			dBackwardRightPoints[0]=dForwardRightPoints[0];
		CP[1][0]=min(dBackwardRightPoints[0],dForwardRightPoints[0]);
		if (CP[1][0]< CP[0][0])  // in case that elapse time of the current phase is more than maximum time, then the right point would be negative. so we should make it at lease thesame as left point
		{
			double iTemp=CP[0][0]-dForwardRightPoints[0];
			CP[1][0]=CP[0][0];
			dForwardRightPoints[0]=CP[0][0];
			dForwardRightPoints[1]=dForwardRightPoints[1]+iTemp;
			dForwardRightPoints[2]=dForwardRightPoints[2]+iTemp;
			dForwardRightPoints[3]=dForwardRightPoints[3]+iTemp;
			dForwardRightPoints[4]=dForwardRightPoints[4]+iTemp;
		}
		CP[0][1]=max(dBackwardLeftPoints[1],dForwardLeftPoints[1]);
		if (dBackwardRightPoints[1]<0)
			dBackwardRightPoints[1]=dForwardRightPoints[1];
		CP[1][1]=min(dBackwardRightPoints[1],dForwardRightPoints[1]);
		CP[0][2]=max(dBackwardLeftPoints[2],dForwardLeftPoints[2]);
		if (type==6) // if the type of request is coordinatuion (type=6) we should let early return to green
			dBackwardLeftPoints[3]=CP[0][2]+ConfigIS.Red[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp2+4]+ConfigIS.Gmin[iPhaseIndexInConfigurationTemp+4];
		else
			dBackwardLeftPoints[3]=dBackwardLeftPoints[4]-globalgmax[iPhaseIndexInConfigurationTemp4+4]-(ConfigIS.Red[iPhaseIndexInConfigurationTemp3+4]+ConfigIS.Yellow[iPhaseIndexInConfigurationTemp3+4]);
	
		if (dBackwardRightPoints[2]<0)
			dBackwardRightPoints[2]=dForwardRightPoints[2];
		CP[1][2]=min(dBackwardRightPoints[2],dForwardRightPoints[2]);
		CP[0][3]=max(dBackwardLeftPoints[3],dForwardLeftPoints[3]);
		if (dBackwardRightPoints[3]<0)
			dBackwardRightPoints[3]=dForwardRightPoints[3];
		CP[1][3]=min(dBackwardRightPoints[3],dForwardRightPoints[3]);
		CP[0][4]=max(dBackwardLeftPoints[4],dForwardLeftPoints[4]);
		if (dBackwardRightPoints[4]<0)
			dBackwardRightPoints[4]=dForwardRightPoints[4];
		CP[1][4]=min(dBackwardRightPoints[4],dForwardRightPoints[4]);
	}
	//cout<<" Curent phase "<<iPhase<<endl;
	//~ cout<<"iPhaseTemp"<<iPhaseTemp<<endl;
	for (int ii=0;ii<iTotalPlanningPhase;ii++)
	{
		cout<<"dForwardLeftPoints[  "<< ii << " "<< dForwardLeftPoints[ii]<<endl; 
		cout<<"dBackwardLeftPoints[ "<< ii << " "<< dBackwardLeftPoints[ii]<<endl;
		cout<<"CP left              "<< ii << " "<< CP[0][ii]<<endl;
		cout<<"dForwardRightPoints[ "<< ii << " "<< dForwardRightPoints[ii]<<endl; 
		cout<<"dBackwardRightPoints["<< ii << " "<< dBackwardRightPoints[ii]<<endl;
		cout<<"CP right             "<< ii << " "<< CP[1][ii]<<endl;
		cout<< "phase "<< CP[2][ii]<<endl;
	}
}

void creatFeasibleRegionRing1_EV(double  CP[3][5], int phaseInRing, double ru,double delay,double *EndOfPhase,int *phaseSeq ,int t)
{
	int iTemp=0;
	int iPhase=0;
	iTemp=phaseSeq[0];
	iPhase=iTemp%10;
	int iTotalPlanningPhase=0;
	int iRequestedPhaseInSeq=-1;
	if (t==1) // in case we have only one phase in this ring, the request can not be passed to the next cycle ( multiple optimal solution in solver may choose the request to be passed to next cycle
		phaseInRing=phaseInRing%10;
		
	for (int i=0; i<t ; i++)
	{
		if (phaseSeq[i]==phaseInRing)
		{
			iRequestedPhaseInSeq=i;
		}
	}
	
	iTotalPlanningPhase=iRequestedPhaseInSeq+1;
	cout<<"iTotalPlanningPhase"<<iTotalPlanningPhase<<endl;
	if (iTotalPlanningPhase==1)// if we have just one phase in this ring! 
	{
		CP[0][0]=ru;
		CP[1][0]=CP[0][0];
		iTemp=phaseSeq[0];
		iPhase=iTemp%10;
		CP[2][0]=iPhase;		
	}
	else
	{
		for (int j=0; j<iTotalPlanningPhase; j++)
		{
			cout<<"Here"<<endl;
			CP[0][j]=EndOfPhase[j];
			CP[1][j]=CP[0][j];
			iTemp=phaseSeq[j];
			iPhase=iTemp%10;
			CP[2][j]=iPhase;	
		}
	}
}

void creatFeasibleRegionRing2_EV(double  CP[3][5], int phaseInRing, double ru, double delay, double *EndOfPhase,int *phaseSeq ,int t)
{
	int iTemp=0;
	int iPhase=0;
	iTemp=phaseSeq[0];
	iPhase=iTemp%10;
	int iTotalPlanningPhase=0;
	int iRequestedPhaseInSeq=-1;
	if (t==1) // in case we have only one phase in this ring, the request can not be passed to the next cycle ( multiple optimal solution in solver may choose the request to be passed to next cycle
		phaseInRing=phaseInRing%10;
		
	for (int i=0; i<t ; i++)
	{
		if (phaseSeq[i]==phaseInRing)
		{
			iRequestedPhaseInSeq=i;
		}
	}
	iTotalPlanningPhase=iRequestedPhaseInSeq+1;
	if (iTotalPlanningPhase==1)
	{
		CP[0][0]=ru;
		CP[1][0]=CP[0][0];
		iTemp=phaseSeq[0];
		iPhase=iTemp%10;
		CP[2][0]=iPhase;		
	}
	else
	{
		for (int j=0; j<iTotalPlanningPhase; j++)
		{
			CP[0][j]=EndOfPhase[j];
			CP[1][j]=CP[0][j];
			iTemp=phaseSeq[j];
			iPhase=iTemp%10;
			CP[2][j]=iPhase;	
		}
	}
}

void creatCPexactlyAsOptSol(double  CP[2][3][5], double *EndOfPhaseRing1,int *phaseSeqRing1 ,int t1, double *EndOfPhaseRing2,int *phaseSeqRing2 ,int t2)
{
	int iTemp=0;
	int iTotalPlanningPhase=0;
	iTotalPlanningPhase= min(t1,t2); // at most we plan 5 phase ahead in each ring (this is to pass to COP). also, the number of the planned phase in first and second ring should be the same.
	iTotalPlanningPhase= min(iTotalPlanningPhase,5);
	cout<< "iTotalPlanningPhase"<<iTotalPlanningPhase<<endl;
	// Ring 1
	for (int j=0; j<iTotalPlanningPhase; j++)
	{
		CP[0][0][j]=EndOfPhaseRing1[j];
		CP[0][1][j]=CP[0][0][j];
		iTemp=phaseSeqRing1[j];
		CP[0][2][j]=iTemp%10;
	}
	//Ring 2
	for (int j=0; j<iTotalPlanningPhase; j++)
	{
		CP[1][0][j]=EndOfPhaseRing2[j];
		CP[1][1][j]=CP[1][0][j];
		iTemp=phaseSeqRing2[j];
		CP[1][2][j]=iTemp%10;
	}
}

void readOptPlanFromFile(char *filename,double  aCriticalPoints[2][3][5])
{
	LinkedList<PriorityRequest> PrioReqList[2];
	for (int k=0;k<2;k++)
		for (int kk=0;kk<3;kk++)
			for (int kkk=0;kkk<5;kkk++)
				aCriticalPoints[k][kk][kkk]=0.0;
	memset(aCriticalPoints,0,sizeof(aCriticalPoints));
	fstream fss;
    fss.open(filename,fstream::in);
    if (!fss)
    {
        cout<<"***********Error opening the plan file!\n";
        sprintf(temp_log,"Error opening the plan file!");
        outputlog(temp_log);
		exit(1); 
    }
    string lineread;
    int SP[2];
    double InitTime1[2],InitGrn[2];
    double V[2][8],g[2][8];
    int ReqNo=0;
    int ReqPhaseNo=0;
    double ReqRl=0.0;
    double ReqRu=0.0;
    double ReqDelay=0.0;
    double dTotalDelay=0.0;
    int ReqType=0;
    //-------------------------------Begin of Read Plan----------------------------//
    getline(fss,lineread);
    sscanf(lineread.c_str(),"%d %d",&SP[0],&SP[1]);
    getline(fss,lineread);
    sscanf(lineread.c_str(),"%lf %lf %lf %lf",&InitTime1[0],&InitTime1[1],&InitGrn[0],&InitGrn[1]);
    for(int i=0;i<2;i++)
    {
        getline(fss,lineread);
        sscanf(lineread.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf",
            &V[i][0],&V[i][1],&V[i][2],&V[i][3],
            &V[i][4],&V[i][5],&V[i][6],&V[i][7]);
    }
    for(int i=0;i<2;i++)
    {
        getline(fss,lineread);
        sscanf(lineread.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf",
            &g[i][0],&g[i][1],&g[i][2],&g[i][3],
            &g[i][4],&g[i][5],&g[i][6],&g[i][7]);
    }
	getline(fss,lineread);
    sscanf(lineread.c_str(),"%d",&ReqNo);
    cout<<"Number of Requests" <<ReqNo<<endl;
    for(int i=0;i<ReqNo;i++)
    {
        getline(fss,lineread);
        sscanf(lineread.c_str(),"%d %lf %lf %lf %d ",&ReqPhaseNo,&ReqRl,&ReqRu,&ReqDelay,&ReqType);
		ReqPhaseNo=ReqPhaseNo-1;
        int CurRing=FindRingNo(ReqPhaseNo);
        if (ReqPhaseNo<=10) // Double check if the req in next cycle is not being process in current cycl!!! 
		{
			if ( ReqPhaseNo<SP[CurRing]-1)
				ReqPhaseNo=ReqPhaseNo*10;			 
		}
        int phase_in_ring=ReqPhaseNo-4*CurRing; // If in ring 2, phase should -4.
        cout<<"ReqPhaseNo "<<ReqPhaseNo<<endl;
        cout<<"phase_in_ring "<<phase_in_ring<<endl;
        PriorityRequest PriorityRequest_t=PriorityRequest(phase_in_ring,ReqRl,ReqRu,ReqDelay,ReqType);
        PrioReqList[CurRing].InsertRear(PriorityRequest_t);
	}
	getline(fss,lineread);
    sscanf(lineread.c_str(),"%lf ",&dTotalDelay);
    cout<<"Total Delay of Requests: "<< dTotalDelay<<endl;
    fss.close();
    
    int R1No=ConfigIS.Ring1No;
    int R2No=ConfigIS.Ring2No;
    SP[0]=SP[0]-1;
    SP[1]=SP[1]-5;
    int SPIdx1=FindIndexArray<int>(ConfigIS.Phase_Seq_R1,R1No,SP[0]);
    int SPIdx2=FindIndexArray<int>(ConfigIS.Phase_Seq_R2,R2No,SP[1]);
    cout <<"SP index 1 and 2:\t"<<(SPIdx1+1)<<"  "<<(SPIdx2+1)<<endl;
    int t1=R1No*2-SPIdx1;
    int t2=R2No*2-SPIdx2;
    double *Split1=new double[t1];
    double *Split2=new double[t2];
    double *mustChangeTime1=new double[t1];
    double *mustChangeTime2=new double[t2];
	int *Phase1=GeneratePhaseArray(SP[0],ConfigIS.Phase_Seq_R1,R1No,t1,1); // TOTAL t1 & t2 "Array.h"
    int *Phase2=GeneratePhaseArray(SP[1],ConfigIS.Phase_Seq_R2,R2No,t2,1); // Phase1 including the cycle information
    int ii=0;
    for (int i=0;i<2;i++)  // Cycle
    {
        for (int k=0;k<R1No;k++)  // Phase
        {
            int RP=ConfigIS.Phase_Seq_R1[k];
            if(V[i][RP]!=0)
            {
                Split1[ii]=V[i][RP];
                ii++;
            }
        }
    }
    for(int i=0;i<t1;i++)
    {
        int phase_no=Phase1[i];
        int ring_no=phase_no/10;
        int real_phase=phase_no%10;
        mustChangeTime1[i]=SumArray<double>(Split1,t1,0,i)+g[ring_no][real_phase];
        mustChangeTime1[i]=mustChangeTime1[i]+InitTime1[0];
    }
    PrintArray<int>(Phase1,t1);
    PrintArray<double>(Split1,t1);
    PrintArray<double>(mustChangeTime1,t1);
    
	ii=0;
    for (int i=0;i<2;i++)  // Cycle
    {
        for (int k=0;k<R2No;k++)  // Phase
        {
            int RP=ConfigIS.Phase_Seq_R2[k]+4; //Ring 2 shuold +4; (should be real phase -1)
            if(V[i][RP]!=0)
            {
                Split2[ii]=V[i][RP];
                ii++;
            }
        }
    }
    for(int i=0;i<t2;i++)
    {
        int phase_no=Phase2[i];
        int ring_no=phase_no/10;
        int real_phase=phase_no%10+4; // Ring 2 shuold +4; (should be real phase -1)
        mustChangeTime2[i]=SumArray<double>(Split2,t2,0,i)+g[ring_no][real_phase];
        mustChangeTime2[i]=mustChangeTime2[i]+InitTime1[1];
    }
    
	PrintArray<int>(Phase2,t2);
    PrintArray<double>(Split2,t2);
    PrintArray<double>(mustChangeTime2,t2);
 
       
    PrioReqList[0].Reset();
    PrioReqList[1].Reset();
	//~ if ( (HaveCoordInList==1 && HaveTransitInList==1)|| (HaveCoordInList==1 && HaveTruckInList==1) ) // if there is coordination priority request AND trnansit or truck priority request, we shuold exactly follow the solution of optimizer to creat Critical points
	//~ {
		//~ creatFeasibleRegionForCoordAndPri(aCriticalPoints,mustChangeTime1, Phase1,t1,mustChangeTime2, Phase2,t2);
	//~ }
	//~ else  // if we have just coordination re in table, or just truck and transit request in table
	
	if (congested==1) // if we face very congested situation , follow the optimal solution from solver
		creatCPexactlyAsOptSol(aCriticalPoints,mustChangeTime1, Phase1,t1,mustChangeTime2, Phase2,t2);
	else
	{
		int iPosOfEarliestReqInPrioReqList1=findTheEarliestReqInERP(PrioReqList[0]); // in ring 1
		//cout<<"iPosOfEarliestReqInPrioReqList1"<<iPosOfEarliestReqInPrioReqList1<<endl;
		int iPosOfEarliestReqInPrioReqList2=findTheEarliestReqInERP(PrioReqList[1]); // in ring 2
		//cout<<"iPosOfEarliestReqInPrioReqList2"<<iPosOfEarliestReqInPrioReqList2<<endl;
		
		if (codeUsage==1) //  if priority and actuation logic is applied (Send to Interface )
		{
			if (dTotalDelay>0) // if the total delay of requests is positive, we have to follow exactly the solution of the optimizer and therefore there is no flexibility 
				creatCPexactlyAsOptSol(aCriticalPoints,mustChangeTime1, Phase1,t1,mustChangeTime2, Phase2,t2);
			else
			{
				if (iPosOfEarliestReqInPrioReqList1>-1) 
				{
					PrioReqList[0].Reset(iPosOfEarliestReqInPrioReqList1);
					creatFeasibleRegionRing1(aCriticalPoints[0],PrioReqList[0],PrioReqList[0].Data().iPhaseCycle,PrioReqList[0].Data().dRl,PrioReqList[0].Data().dRu, PrioReqList[0].Data().dReqDelay, PrioReqList[0].Data().iType,mustChangeTime1, Phase1,t1, ConfigIS, dGlobalGmax ,InitTime1[0],InitGrn[0],iPosOfEarliestReqInPrioReqList2); 
				}
				if (iPosOfEarliestReqInPrioReqList2>-1) 
				{
					PrioReqList[1].Reset(iPosOfEarliestReqInPrioReqList2);
					creatFeasibleRegionRing2(aCriticalPoints[1],PrioReqList[1],PrioReqList[1].Data().iPhaseCycle,PrioReqList[1].Data().dRl,PrioReqList[1].Data().dRu, PrioReqList[1].Data().dReqDelay, PrioReqList[1].Data().iType,mustChangeTime2, Phase2,t2, ConfigIS, dGlobalGmax, InitTime1[1],InitGrn[1],iPosOfEarliestReqInPrioReqList1); 
				}
			}
		}
		else //  if priority + intelligent phase allocation alg applied (Send to COP ) // in this case, if we have request in one ring, and coordination in the second ring, we focuse on priority request !!!
		{
			int tempTypeR1=0;
			int tempTypeR2=0;
			if (iPosOfEarliestReqInPrioReqList1>-1) 
			{
				PrioReqList[0].Reset(iPosOfEarliestReqInPrioReqList1);
				tempTypeR1 = PrioReqList[0].Data().iType;
			}
			if (iPosOfEarliestReqInPrioReqList2>-1) 	
			{
				PrioReqList[1].Reset(iPosOfEarliestReqInPrioReqList2);
				tempTypeR2 = PrioReqList[1].Data().iType;
			}
			if (tempTypeR1==COORDINATION && tempTypeR2!=COORDINATION)
				creatFeasibleRegionRing2(aCriticalPoints[1],PrioReqList[1],PrioReqList[1].Data().iPhaseCycle,PrioReqList[1].Data().dRl,PrioReqList[1].Data().dRu, PrioReqList[1].Data().dReqDelay, PrioReqList[1].Data().iType,mustChangeTime2, Phase2,t2, ConfigIS, dGlobalGmax, InitTime1[1],InitGrn[1],iPosOfEarliestReqInPrioReqList1); 
			else if (tempTypeR2==COORDINATION && tempTypeR1!=COORDINATION)
				creatFeasibleRegionRing1(aCriticalPoints[0],PrioReqList[0],PrioReqList[0].Data().iPhaseCycle,PrioReqList[0].Data().dRl,PrioReqList[0].Data().dRu, PrioReqList[0].Data().dReqDelay, PrioReqList[0].Data().iType,mustChangeTime1, Phase1,t1, ConfigIS, dGlobalGmax ,InitTime1[0],InitGrn[0],iPosOfEarliestReqInPrioReqList2); 
			else if ( (tempTypeR2!=COORDINATION && tempTypeR1!=COORDINATION) || (tempTypeR2==COORDINATION && tempTypeR1==COORDINATION) )
			{
				creatFeasibleRegionRing1(aCriticalPoints[0],PrioReqList[0],PrioReqList[0].Data().iPhaseCycle,PrioReqList[0].Data().dRl,PrioReqList[0].Data().dRu, PrioReqList[0].Data().dReqDelay, PrioReqList[0].Data().iType,mustChangeTime1, Phase1,t1, ConfigIS, dGlobalGmax ,InitTime1[0],InitGrn[0],iPosOfEarliestReqInPrioReqList2); 
				creatFeasibleRegionRing2(aCriticalPoints[1],PrioReqList[1],PrioReqList[1].Data().iPhaseCycle,PrioReqList[1].Data().dRl,PrioReqList[1].Data().dRu, PrioReqList[1].Data().dReqDelay, PrioReqList[1].Data().iType,mustChangeTime2, Phase2,t2, ConfigIS, dGlobalGmax, InitTime1[1],InitGrn[1],iPosOfEarliestReqInPrioReqList1); 
			}
		}
	}
    delete []mustChangeTime1;
    delete []mustChangeTime2;
    delete []Split1;
    delete []Split2;
}


void readOptPlanFromFileForEV(char *filename,double aCriticalPoints[2][3][5], int omitPhases[8])
{
	LinkedList<PriorityRequest> PrioReqList[2];
	// Figuring out which phases are missing
	for (int j=0;j<8;j++)
		omitPhases[j]=0;
	for (int j=0; j<8;j++)
		if (ConfigIS_EV.Gmax[j]==0 && ConfigIS.Gmax[j]>0 )
			omitPhases[j]=j+1;
			
	for (int k=0;k<2;k++)
		for (int kk=0;kk<3;kk++)
			for (int kkk=0;kkk<5;kkk++)
				aCriticalPoints[k][kk][kkk]=0.0;
	
	fstream fss;
    fss.open(filename,fstream::in);
    if (!fss)
    {
        cout<<"***********Error opening the plan file!\n";
        sprintf(temp_log,"Error opening the plan file!");
        outputlog(temp_log);
		exit(1); 
    }
    string lineread;
    int SP[2];
    double InitTime1[2],InitGrn[2];
    double V[2][8],g[2][8];
    int ReqNo=0;
    int ReqPhaseNo=0;
    double ReqRl=0.0;
    double ReqRu=0.0;
    double ReqDelay=0.0;
    int ReqType=0;
    //-------------------------------Begin of Read Plan----------------------------//
    getline(fss,lineread);
    sscanf(lineread.c_str(),"%d %d",&SP[0],&SP[1]);
    getline(fss,lineread);
    sscanf(lineread.c_str(),"%lf %lf %lf %lf",&InitTime1[0],&InitTime1[1],&InitGrn[0],&InitGrn[1]);
    for(int i=0;i<2;i++)
    {
        getline(fss,lineread);
        sscanf(lineread.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf",
            &V[i][0],&V[i][1],&V[i][2],&V[i][3],
            &V[i][4],&V[i][5],&V[i][6],&V[i][7]);
    }
    for(int i=0;i<2;i++)
    {
        getline(fss,lineread);
        sscanf(lineread.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf",
            &g[i][0],&g[i][1],&g[i][2],&g[i][3],
            &g[i][4],&g[i][5],&g[i][6],&g[i][7]);
    }
	getline(fss,lineread);
    sscanf(lineread.c_str(),"%d",&ReqNo);
    for(int i=0;i<ReqNo;i++)
    {
        getline(fss,lineread);
        sscanf(lineread.c_str(),"%d %lf %lf %lf %d ",&ReqPhaseNo,&ReqRl,&ReqRu,&ReqDelay,&ReqType);
		ReqPhaseNo=ReqPhaseNo-1;
        int CurRing=FindRingNo(ReqPhaseNo);
        if (ReqPhaseNo<=10) // Double check if the req in next cycle is not being process in current cycl!!! 
		{
			if ( ReqPhaseNo<SP[CurRing]-1)
				ReqPhaseNo=ReqPhaseNo*10;			 
		}
        int phase_in_ring=ReqPhaseNo-4*CurRing; // If in ring 2, phase should -4.
        PriorityRequest PriorityRequest_t=PriorityRequest(phase_in_ring,ReqRl,ReqRu,ReqDelay,ReqType);
        PrioReqList[CurRing].InsertRear(PriorityRequest_t);
	}
    fss.close();
    
    int R1No=ConfigIS_EV.Ring1No;
    int R2No=ConfigIS_EV.Ring2No;
    SP[0]=SP[0]-1;
    SP[1]=SP[1]-5;
    int SPIdx1=FindIndexArray<int>(ConfigIS_EV.Phase_Seq_R1,R1No,SP[0]);
    int SPIdx2=FindIndexArray<int>(ConfigIS_EV.Phase_Seq_R2,R2No,SP[1]);
    cout <<"SP index 1 and 2:\t"<<(SPIdx1+1)<<"  "<<(SPIdx2+1)<<endl;
    
    
     int t1=-1;//R1No*2-SPIdx1;
     int t2=-1;//R2No*2-SPIdx2;
     double *Split1;
     double *Split2;
     double *mustChangeTime1;
     double *mustChangeTime2;
	 int *Phase1;//=GeneratePhaseArray(SP[0],ConfigIS_EV.Phase_Seq_R1,R1No,t1,1); // TOTAL t1 & t2 "Array.h"
     int *Phase2;//=GeneratePhaseArray(SP[1],ConfigIS_EV.Phase_Seq_R2,R2No,t2,1); // Phase1 including the cycle information
     
   
    cout<<"R1No "<<R1No<<endl;
    cout<<"R2No "<<R2No<<endl;
    int ii=0;
	if (R1No==1)
	{
		t1=1;    // only one phase
		Phase1=new int[t1];
		Split1=new double[t1];
        mustChangeTime1=new double[t1];
        Split1[0]=0.0;
        int *Phase11=GeneratePhaseArray(SP[0],ConfigIS_EV.Phase_Seq_R1,R1No,t1,1); // TOTAL t1=1
        Phase1[0]=Phase11[0];
        //------ IF only have one phase in a ring, we need to add 2 cycles' result up.
        ii=0;
        int RP1=ConfigIS_EV.Phase_Seq_R1[0]%10; // IS (Real phase -1)
        for (int i=0;i<2;i++)  // Cycle
        {
            Split1[ii]+=V[i][RP1];
        }
        int CurPhase=Phase1[ii]%10;  // Ring 2 shuold +4;
        mustChangeTime1[ii]=Split1[ii]-ConfigIS_EV.Yellow[CurPhase]-ConfigIS_EV.Red[CurPhase];
        PrintArray<int>(Phase1,t1);
        PrintArray<double>(Split1,t1);
        PrintArray<double>(mustChangeTime1,t1);
	}else
	{
		t1=R1No*2-SPIdx1;
		Phase1=new int[t1];
		Split1=new double[t1];
        mustChangeTime1=new double[t1];
        int *Phase11=GeneratePhaseArray(SP[0],ConfigIS_EV.Phase_Seq_R1,R1No,t1,1);
        //~ cout<<Phase11[0]<<endl;
        //~ cout<<Phase11[1]<<endl;
     //~ 
        for(int jj=0;jj<t1;jj++)
        {
			Phase1[jj]=Phase11[jj];
		}
        ii=0;
        for (int i=0;i<2;i++)  // Cycle
        {
            for (int k=0;k<R1No;k++)  // Phase
            {
                int RP=ConfigIS_EV.Phase_Seq_R1[k];
                if(V[i][RP]!=0)
                {
                    Split1[ii]=V[i][RP];
                    ii++;
                }
            }
        }
        for(int i=0;i<t1;i++)
        {
            int phase_no=Phase1[i];
            int ring_no=phase_no/10; //Cycle
            int real_phase=phase_no%10;
            mustChangeTime1[i]=SumArray<double>(Split1,t1,0,i)+g[ring_no][real_phase];
            mustChangeTime1[i]=mustChangeTime1[i]+InitTime1[0];
        }
        PrintArray<int>(Phase1,t1);
        PrintArray<double>(Split1,t1);
        PrintArray<double>(mustChangeTime1,t1);
	}

	if (R2No==1)
	{
		if(SPIdx2<0)
        {
            cout<<"***********Starting Phase is not in Ring 2 Sequence.***********\n";
            sprintf(temp_log,"***********Starting Phase is not in Ring 2 Sequence.***********");
            outputlog(temp_log);            exit(1);
        }
		t2=1;    // only one phase
		Phase2=new int[t2];
		Split2=new double[t2];
        mustChangeTime2=new double[t2];
		Split2[0]=0.0;
        int *Phase22=GeneratePhaseArray(SP[1],ConfigIS_EV.Phase_Seq_R2,R2No,t2,1); // TOTAL t1=1
        Phase2[0]=Phase22[0];
        //------ IF only have one phase in a ring, we need to add 2 cycles' result up.
        ii=0;
        int RP2=ConfigIS_EV.Phase_Seq_R2[0]%10+4; // IS (Real phase -1)
        for (int i=0;i<2;i++)  // Cycle
        {
            Split2[ii]+=V[i][RP2];
        }
        int CurPhase=Phase2[ii]%10+4;  // Ring 2 shuold +4;
        mustChangeTime2[ii]=Split2[ii]-ConfigIS_EV.Yellow[CurPhase]-ConfigIS_EV.Red[CurPhase];
        PrintArray<int>(Phase2,t2);
        PrintArray<double>(Split2,t2);
         PrintArray<double>(mustChangeTime2,t2);
  	}else
	{
		
		t2=R2No*2-SPIdx2;
		Phase2=new int[t2];
		Split2=new double[t2];
        mustChangeTime2=new double[t2];
        int *Phase22=GeneratePhaseArray(SP[1],ConfigIS_EV.Phase_Seq_R2,R2No,t2,1);
        for(int jj=0;jj<t2;jj++)
			Phase2[jj]=Phase22[jj];
        ii=0;
        for (int i=0;i<2;i++)  // Cycle
        {
            for (int k=0;k<R2No;k++)  // Phase
            {
                int RP=ConfigIS_EV.Phase_Seq_R2[k]%10+4;
                if(V[i][RP]!=0)
                {
                    Split2[ii]=V[i][RP];
                    cout<<" Split2[ii]"<< Split2[ii]<<endl;
                    ii++;
                }
            }
        }
        for(int i=0;i<t2;i++)
        {
            int phase_no=Phase2[i];
            int ring_no=phase_no/10; //Cycle
            int real_phase=phase_no%10+4;
            mustChangeTime2[i]=SumArray<double>(Split2,t2,0,i)+g[ring_no][real_phase];
            mustChangeTime2[i]=mustChangeTime2[i]+InitTime1[1];
        }
        PrintArray<int>(Phase2,t2);
        PrintArray<double>(Split2,t2);
        PrintArray<double>(mustChangeTime2,t2);
	}
	int iPosOfEarliestReqInPrioReqList1=findTheEarliestReqInERP(PrioReqList[0]); // in ring 1
		//cout<<"iPosOfEarliestReqInPrioReqList1"<<iPosOfEarliestReqInPrioReqList1<<endl;
	int iPosOfEarliestReqInPrioReqList2=findTheEarliestReqInERP(PrioReqList[1]); // in ring 2
		//cout<<"iPosOfEarliestReqInPrioReqList2"<<iPosOfEarliestReqInPrioReqList2<<endl;
    if (iPosOfEarliestReqInPrioReqList1>-1) 
    {
		PrioReqList[0].Reset(iPosOfEarliestReqInPrioReqList1);
		creatFeasibleRegionRing1_EV(aCriticalPoints[0],PrioReqList[0].Data().iPhaseCycle,PrioReqList[0].Data().dRu,PrioReqList[0].Data().dReqDelay,mustChangeTime1, Phase1,t1); 
	}
	if (iPosOfEarliestReqInPrioReqList2>-1) 
    {
		PrioReqList[1].Reset(iPosOfEarliestReqInPrioReqList2);
		creatFeasibleRegionRing2_EV(aCriticalPoints[1],PrioReqList[1].Data().iPhaseCycle,PrioReqList[1].Data().dRu,PrioReqList[1].Data().dReqDelay,mustChangeTime2, Phase2,t2); 
	}
 
	delete []mustChangeTime1;
    delete []mustChangeTime2;
    delete []Split1;
    delete []Split2;
    delete []Phase1;
    delete []Phase2;

}





void PrintList2File(char *Filename,LinkedList<ReqEntry> &ReqList,int BroadCast)
{
    if(BroadCast==0)
    {
        ofstream fs;
        //iostream fs;
        fs.open(Filename);
        if(fs.good())
        {
            fs<<"Num_req  "<<ReqList.ListSize()<<" "<<0<<endl;  //*** IMPORTANT: UpdateFlag changed to "0": means solved ***//
            if (!ReqList.ListEmpty())
            {
                ReqList.Reset();
                while(!ReqList.EndOfList())
                {
                    fs<<RSUID<<" "<<ReqList.Data().VehID<<"  "<<ReqList.Data().VehClass<<"  "<<ReqList.Data().ETA<<"  "<<ReqList.Data().Phase                        
                        <<" "<<ReqList.Data().MinGreen<<" "<<ReqList.Data().AbsTime<<" "<<ReqList.Data().Split_Phase<<" "<<ReqList.Data().iInLane<<" "<<ReqList.Data().iOutLane
                        <<" "<<ReqList.Data().iStrHour<<" "<<ReqList.Data().iStrMinute<<" "<<ReqList.Data().iStrSecond<<" "<<ReqList.Data().iEndHour
                        <<" "<<ReqList.Data().iEndMinute<<" "<<ReqList.Data().iEndSecond<<" "<<ReqList.Data().iVehState<<" "<<ReqList.Data().iMsgCnt<< " " << endl;
                    ReqList.Next();
                }
            }
        }
        else
        {
            perror("Error occurs when open a file to write the linked list");
        }
        cout<<"Print to file successfully.\n";
        fs.close();
    }
    else // if(BroadCast==0)
    {
        ofstream fs;
        fs.open(Filename);
        if(fs.good())
        {
            fs<<"Num_req  "<<ReqList.ListSize()<<" "<<0<<endl;  //*** IMPORTANT: UpdateFlag changed to "0": means solved ***//
            if (!ReqList.ListEmpty())
            {
                ReqList.Reset();
                while(!ReqList.EndOfList())
                {
                    fs<<RSUID<<" "<<ReqList.Data().VehID<<"  "<<ReqList.Data().VehClass<<"  "<<ReqList.Data().ETA<<"  "<<ReqList.Data().Phase                        
                        <<" "<<ReqList.Data().MinGreen<<" "<<ReqList.Data().AbsTime<<" "<<ReqList.Data().iInLane<<" "<<ReqList.Data().iOutLane
                        <<" "<<ReqList.Data().iStrHour<<" "<<ReqList.Data().iStrMinute<<" "<<ReqList.Data().iStrSecond<<" "<<ReqList.Data().iEndHour
                        <<" "<<ReqList.Data().iEndMinute<<" "<<ReqList.Data().iEndSecond<<" "<<ReqList.Data().iVehState<<" "<<ReqList.Data().iMsgCnt<< " " << endl;
                    ReqList.Next();
                }
            }
        }
        else
        {
            perror("Error occurs when open a file to write the linked list");
        }
        cout<<"Print to file successfully.\n";
        fs.close();

    } //end of "if(BroadCast==0)"

}


void PrintList2File_EV(char *Filename,LinkedList<ReqEntry> &ReqList,int BroadCast)
{
    if(BroadCast==0)
    {
        ofstream fs;
        //iostream fs;
        fs.open(Filename);
        if(fs.good())
        {
            fs<<"Num_req  "<<ReqList.ListSize()<<" "<<0<<endl;  //*** IMPORTANT: UpdateFlag changed to "0": means solved ***//
            if (!ReqList.ListEmpty())
            {
                ReqList.Reset();
                while(!ReqList.EndOfList())
                {
                    fs<<RSUID<<" "<<ReqList.Data().VehID<<"  "<<ReqList.Data().VehClass<<"  "<<ReqList.Data().ETA<<"  "<<ReqList.Data().Phase                        
                        <<" "<<ReqList.Data().MinGreen<<" "<<ReqList.Data().AbsTime<<" "<<ReqList.Data().iInLane<<" "<<ReqList.Data().iOutLane
                        <<" "<<ReqList.Data().iStrHour<<" "<<ReqList.Data().iStrMinute<<" "<<ReqList.Data().iStrSecond<<" "<<ReqList.Data().iEndHour
                        <<" "<<ReqList.Data().iEndMinute<<" "<<ReqList.Data().iEndSecond<<" "<<ReqList.Data().iVehState<<" "<<ReqList.Data().iMsgCnt<< " " << endl;
                    ReqList.Next();

                }
            }
        }
        else
        {
            perror("Error occurs when open a file to write the linked list");
        }
        //cout<<"Print to file successfully.\n";

        fs.close();

    }
    else
    {
        ofstream fs;
        fs.open(Filename);
        if(fs.good())
        {
            fs<<"Num_req  "<<ReqList.ListSize()<<" "<<0<<endl;  //*** IMPORTANT: UpdateFlag changed to "0": means solved ***//
            if (!ReqList.ListEmpty())
            {
                ReqList.Reset();
                while(!ReqList.EndOfList())
                {
                    fs<<RSUID<<" "<<ReqList.Data().VehID<<"  "<<ReqList.Data().VehClass<<"  "<<ReqList.Data().ETA<<"  "<<ReqList.Data().Phase                        
                        <<" "<<ReqList.Data().MinGreen<<" "<<ReqList.Data().AbsTime<<" "<<ReqList.Data().iInLane<<" "<<ReqList.Data().iOutLane
                        <<" "<<ReqList.Data().iStrHour<<" "<<ReqList.Data().iStrMinute<<" "<<ReqList.Data().iStrSecond<<" "<<ReqList.Data().iEndHour
                        <<" "<<ReqList.Data().iEndMinute<<" "<<ReqList.Data().iEndSecond<<" "<<ReqList.Data().iVehState<<" "<<ReqList.Data().iMsgCnt<< " " << endl;
                    ReqList.Next();
                }
            }
        }
        else
        {
            perror("Error occurs when open a file to write the linked list");
        }
        cout<<"Print to file successfully.\n";
        fs.close();

    }

}

void ReqListFromFile(char *filename,LinkedList<ReqEntry>& Req_List)
{
    fstream fss;

    fss.open(filename,fstream::in);

    ReqEntry req_temp;
    int ReqNo;
    char RSU_ID[128],OBU_ID[128];
    int Veh_Class,Req_Phase,Req_SplitPhase;
    int abs_time;
    float ETA,MinGrn;
	int iInLane,iOutLane,iStrHour,iStrMinute,iStrSecond,iEndHour,iEndMinute,iEndSecond,iVehState, iMsgCnt;	 
	
    string lineread;
    Req_List.ClearList();

    getline(fss,lineread);
    sscanf(lineread.c_str(),"%*s %d %d",&ReqNo,&ReqListUpdateFlag);

    //cout<<"The total Requests is:"<<ReqNo<<endl;

    sprintf(temp_log,"request number is: %d, Flag is{%d}\n",ReqNo,ReqListUpdateFlag);
    outputlog(temp_log);


    while(!fss.eof())
    {
        getline(fss,lineread);
        if(lineread.size()!=0)
        {
            sscanf(lineread.c_str(),"%s %s %d %f %d %f %d %d %d %d %d %d %d %d %d %d %d %d ",RSU_ID,OBU_ID,&Veh_Class,
                &ETA,&Req_Phase,&MinGrn,&abs_time,&Req_SplitPhase,&iInLane,&iOutLane,&iStrHour,&iStrMinute,&iStrSecond,&iEndHour,&iEndMinute,&iEndSecond,&iVehState,&iMsgCnt);

            ReqEntry req_temp(OBU_ID,Veh_Class,ETA,Req_Phase,MinGrn,abs_time,Req_SplitPhase,iInLane,iOutLane,iStrHour,iStrMinute,iStrSecond,iEndHour,iEndMinute,iEndSecond,iVehState, iMsgCnt);

            Req_List.InsertAfter(req_temp);

            cout<<lineread<<endl;

            //outputlog(lineread.c_str());
        }
    }
    fss.close();
}




void ReqListFromFile_EV(char *filename,LinkedList<ReqEntry>& Req_List)
{
    fstream fss;

    fss.open(filename,fstream::in);

    ReqEntry req_temp;
    int ReqNo;
    char RSU_ID[128],OBU_ID[128];
    int Veh_Class,Req_Phase,Req_SplitPhase;
    int abs_time;
    float ETA,MinGrn;
	int iInLane,iOutLane,iStrHour,iStrMinute,iStrSecond,iEndHour,iEndMinute,iEndSecond,iVehState, iMsgCnt;	 
	
    string lineread;
    Req_List.ClearList();

    getline(fss,lineread);
    sscanf(lineread.c_str(),"%*s %d %d",&ReqNo,&ReqListUpdateFlag);

    sprintf(temp_log,"request Combined number is: %d, Flag is{%d}\n",ReqNo,ReqListUpdateFlag);
    outputlog(temp_log);

    while(!fss.eof())
    {
        getline(fss,lineread);
        if(lineread.size()!=0)
        {
             sscanf(lineread.c_str(),"%s %s %d %f %d %f %d %d %d %d %d %d %d %d %d %d %d %d ",RSU_ID,OBU_ID,&Veh_Class,
                &ETA,&Req_Phase,&MinGrn,&abs_time,&iInLane,&iOutLane,&iStrHour,&iStrMinute,&iStrSecond,&iEndHour,&iEndMinute,&iEndSecond,&iVehState,&iMsgCnt);
            ReqEntry req_temp(OBU_ID,Veh_Class,ETA,Req_Phase,MinGrn,abs_time,0,iInLane,iOutLane,iStrHour,iStrMinute,iStrSecond,iEndHour,iEndMinute,iEndSecond,iVehState, iMsgCnt);
            Req_List.InsertAfter(req_temp);
            cout<<lineread<<endl;
            //outputlog(lineread.c_str());
        }
    }
    fss.close();
}



// Without EV case: will extend the MaxGrn to some portion for Transit requested phases
void LinkList2DatFile(LinkedList<ReqEntry> Req_List,char *filename,double InitTime[2],int InitPhase[2],double GrnElapse[2],double transitWeigth, double truckWeigth,double coordinationWeigth)
{
    //-- Convert request linkedlist to the NewModel.dat file for GLPK solver.
    //-- Init1 and Init2 are the initial time for the two rings while current phases are in R or Y.
    //-- MinGrn1 and MinGrn2 are the elapsed time when current phase is in Green

    ofstream fs;
    fs.open(filename,ios::out);

    int R1No=ConfigIS.Ring1No;
    int R2No=ConfigIS.Ring2No;

    fs<<"data;\n";
    fs<<"param current:=0;\n";
    fs<<"param SP1:="<<InitPhase[0]<<";\n";  // This is the real phase [1-4]
    fs<<"param SP2:="<<InitPhase[1]<<";\n";  // This is the real phase [5-8]

    for(int i=0;i<2;i++)
    {
        if (InitTime[i]<0)
        {
            InitTime[i]=0;
        }
    }

    fs<<"param init1:="<<InitTime[0]<<";\n";
    fs<<"param init2:="<<InitTime[1]<<";\n";
    fs<<"param Grn1 :="<<GrnElapse[0]<<";\n";
    fs<<"param Grn2 :="<<GrnElapse[1]<<";\n";

    //=================Add the information for Yellow, Red======//
    fs<<"param y       \t:=";
    for(int i=0;i<R1No;i++)
    {
        int k=ConfigIS.Phase_Seq_R1[i];
        fs<<"\t"<<(k+1)<<"  "<<ConfigIS.Yellow[k];
    }
    for(int i=0;i<R2No;i++)
    {
        int k=ConfigIS.Phase_Seq_R2[i];
        fs<<"\t"<<(k+5)<<"  "<<ConfigIS.Yellow[k+4];
    }
    fs<<";\n";

    fs<<"param red       \t:=";
    for(int i=0;i<R1No;i++)
    {
        int k=ConfigIS.Phase_Seq_R1[i];
        fs<<"\t"<<(k+1)<<"  "<<ConfigIS.Red[k];
    }
    for(int i=0;i<R2No;i++)
    {
        int k=ConfigIS.Phase_Seq_R2[i];
        fs<<"\t"<<(k+5)<<"  "<<ConfigIS.Red[k+4];
    }
    fs<<";\n";

    fs<<"param gmin      \t:=";
    for(int i=0;i<R1No;i++)
    {
        int k=ConfigIS.Phase_Seq_R1[i];
        fs<<"\t"<<(k+1)<<"  "<<ConfigIS.Gmin[k];
    }
    for(int i=0;i<R2No;i++)
    {
        int k=ConfigIS.Phase_Seq_R2[i];
        fs<<"\t"<<(k+5)<<"  "<<ConfigIS.Gmin[k+4];
    }
    fs<<";\n";

    /*
    fs<<"param gmax      \t:=";
    for(int i=0;i<R1No;i++)
    {
        int k=ConfigIS.Phase_Seq_R1[i];

        fs<<"\t"<<(k+1)<<"  "<<ConfigIS.Gmax[k];
    }
    for(int i=0;i<R2No;i++)
    {
        int k=ConfigIS.Phase_Seq_R2[i];

        fs<<"\t"<<(k+5)<<"  "<<ConfigIS.Gmax[k+4];
    }
    fs<<";\n\n";
    //*/

	// ReqList2Log(Req_List);
	Req_List.Reset();
	int iGmax[2][4]={}; // max green time of each phase in ring 1
	for(int i=0;i<R1No;i++)
    {
        int k=ConfigIS.Phase_Seq_R1[i];
		if(RequestPhaseInList(Req_List,(k+1))>0)
        {
			dGlobalGmax[i]=(ConfigIS.Gmax[k]*(1+MaxGrnExtPortion));
			dGlobalGmax[i+4]=dGlobalGmax[i];
			iGmax[0][i]=(int) dGlobalGmax[i];
        }
        else
        {
			dGlobalGmax[i]=ConfigIS.Gmax[k];
			iGmax[0][i]=(int)dGlobalGmax[i];
        }
    }
    for(int i=0;i<R2No;i++)
    {
        int k=ConfigIS.Phase_Seq_R2[i];
		if(RequestPhaseInList(Req_List,(k+5))>0)
        {
			dGlobalGmax[i+4]=(ConfigIS.Gmax[k+4]*(1+MaxGrnExtPortion));
			dGlobalGmax[i]=dGlobalGmax[i+4];
			iGmax[1][i]=(int) dGlobalGmax[i+4];
        }
        else
        {
			dGlobalGmax[i+4]=max(ConfigIS.Gmax[k+4],dGlobalGmax[i+4]);
			iGmax[1][i]=(int) dGlobalGmax[i+4];
        }
    }

	Req_List.Reset();
    fs<<"param gmax      \t:=";
    for(int i=0;i<R1No;i++)
    {
        int k=ConfigIS.Phase_Seq_R1[i];
        fs<<"\t"<<(k+1)<<"  "<<dGlobalGmax[i];
		//~ if(RequestPhaseInList(Req_List,(k+1))>0)
        //~ {
			//~ //iGmax[0][i]=int(ConfigIS.Gmax[k]*(1+MaxGrnExtPortion));
            //~ fs<<"\t"<<(k+1)<<"  "<<iGmax[0][i];
        //~ }
        //~ else
        //~ {
			//~ iGmax[0][i]=(int) ConfigIS.Gmax[k];
            //~ fs<<"\t"<<(k+1)<<"  "<<ConfigIS.Gmax[k];
        //~ }
    }
    for(int i=0;i<R2No;i++)
    {
        int k=ConfigIS.Phase_Seq_R2[i];
        fs<<"\t"<<(k+5)<<"  "<< dGlobalGmax[i+4];
		//~ if(RequestPhaseInList(Req_List,(k+5))>0)
        //~ {
			//~ iGmax[1][i]=int(ConfigIS.Gmax[k+4]*(1+MaxGrnExtPortion));
            //~ fs<<"\t"<<(k+5)<<"  "<< iGmax[1][i];
        //~ }
        //~ else
        //~ {
			//~ iGmax[1][i]=(int) ConfigIS.Gmax[k+4];
            //~ fs<<"\t"<<(k+5)<<"  "<< ConfigIS.Gmax[k+4];
        //~ }
    }

    fs<<";\n\n";


	Req_List.Reset();
	int NumberofRequests=0;
	int iNumberofTransitInList=1;
	int iNumberofTruckInList=1;
	char tempID[64]=" ";
	fs<<"param priorityType:= ";
	if(Req_List.ListEmpty()==0)
    {
		while(!Req_List.EndOfList())
		{
			//cout<<"Req_List.Data().VehID" << Req_List.Data().VehID<< endl;
			if (strcmp(tempID,Req_List.Data().VehID)!=0);
			{
				
				NumberofRequests++;
				if (Req_List.Data().VehClass==2)
					iNumberofTransitInList++;
				if (Req_List.Data().VehClass==3)
					iNumberofTruckInList++;
				//cout<< "NumberofRequests"<<NumberofRequests<<endl;
				fs<<NumberofRequests;
				fs<<" ";
				fs<<Req_List.Data().VehClass;			
				fs<<" ";
				//cout<<"fs<<Req_List.Data().VehClass;			"<<Req_List.Data().VehClass<<endl;
				strcpy(tempID,Req_List.Data().VehID);
			}
			Req_List.Next();
		}
		while (NumberofRequests<10)
		{
			NumberofRequests++;
			fs<<NumberofRequests;
			fs<<" ";
			fs<<0;			
			fs<<" ";
		}
		fs<<" ;  \n";
	}
	
	if (iNumberofTransitInList>1)
		iNumberofTransitInList=iNumberofTransitInList-1;
	if (iNumberofTruckInList>1)
		iNumberofTruckInList=iNumberofTruckInList-1;
		
		
	fs<<"param PrioWeigth:=  1 1 2 ";
	fs<< transitWeigth/iNumberofTransitInList;
	fs<< " 3 " ;
	fs<< truckWeigth/iNumberofTruckInList;
	fs<< " 4 0 5 0 ";
	fs<< " 6 " ;
	fs<< coordinationWeigth;
	fs<< " 7 0 8 0 9 0 10 0 ; \n" ;
	
	
    //================End of the information for Yellow, Red======//

    //---------------According to priorityconfiguration file, some priority eligible vehicle may have wight equal to zero. we shoudl remove them from the list
	LinkedList<ReqEntry> Req_List_New;
	if(Req_List.ListEmpty()==0)
    {
        removeZeroWeightReq(Req_List, transitWeigth, truckWeigth,Req_List_New);
    }

    fs<<"param Rl (tr): 1 2 3 4 5 6 7 8:=\n";
    Req_List_New.Reset();
    int ReqSeq=1;
    while(!Req_List_New.EndOfList())
    {
        fs<<ReqSeq<<"  ";
        for(int j=1;j<=8;j++)
        {			
            if(Req_List_New.Data().Phase==j)
            {
				if (Req_List_New.Data().MinGreen>0) // in this case the vhicle is in the queue and we should set the Rl as less as possible!!!  // MZ Added to hedge against the worst case that may happen when the vehicle is in the queue
				{
					int iRingOfTheRequest=FindRingNo(j-1); 
					
					if (Req_List_New.Data().VehClass==6)// if it is a coordination request
					{
						fs<< 1 << "  " ;					
					}else
					{
						if ( iGmax[iRingOfTheRequest][(j-1)%4]<=0 )
						{
							if (j%2==0) 
								iGmax[iRingOfTheRequest][(j-1)%4]=iGmax[iRingOfTheRequest][(j-1)%4-1];
							else
								iGmax[iRingOfTheRequest][(j-1)%4]=iGmax[iRingOfTheRequest][(j-1)%4+1];
						}
						if ( (InitPhase[iRingOfTheRequest]==j) )
							fs<< max( (Req_List_New.Data().MinGreen+ROBUSTTIME_UP) -(iGmax[iRingOfTheRequest][(j-1)%4]-GrnElapse[iRingOfTheRequest]), double (1) ) <<"  "; 
						else
						fs<< max( (Req_List_New.Data().MinGreen+ROBUSTTIME_UP)-(iGmax[iRingOfTheRequest][(j-1)%4]), float(1) ) <<"  ";   // Mehdi Added to hedge against the worst case that may happen when the vehicle is in the queue
					}
				}
				else
				{
					if (Req_List_New.Data().VehClass==6) // if it is a coordination request
					{
						fs<< Req_List_New.Data().ETA << "  " ;					
					}else
					{
						fs<<max(Req_List_New.Data().ETA-ROBUSTTIME_LOW,float(1.0))<<"  "; 
					}
				}
			}
            else
                fs<<".  ";
        }
        if(ReqSeq<Req_List_New.ListSize())
            fs<<"\n";
        Req_List_New.Next();
        ReqSeq=ReqSeq+1;
    }
    fs<<";\n";
    fs<<"param Ru (tr): 1 2 3 4 5 6 7 8:=\n";
    Req_List_New.Reset();
    ReqSeq=1;
    while(!Req_List_New.EndOfList())
    {
        fs<<ReqSeq<<"  ";
        for(int j=1;j<=8;j++)
        {
            if(Req_List_New.Data().Phase==j)
            {
				if (Req_List_New.Data().MinGreen>0 && Req_List_New.Data().VehClass!=6) 
				{
					fs<< ROBUSTTIME_UP+Req_List_New.Data().MinGreen<<"  "; 
				}
				else if (Req_List_New.Data().MinGreen<=0 && Req_List_New.Data().VehClass!=6) 
				{
					fs<<Req_List_New.Data().ETA+ROBUSTTIME_UP <<"  "; 
				}
				else if (Req_List_New.Data().MinGreen<=0 && Req_List_New.Data().VehClass==6) 
				{
					fs<< Req_List_New.Data().ETA + ConfigIS.iCoordinationSplit<< "  " ;
				}
				else if (Req_List_New.Data().MinGreen>0 && Req_List_New.Data().VehClass==6) 
				{
					fs<< Req_List_New.Data().MinGreen << "  " ;
				}
			}
            else
                fs<<".  ";
        }
        if(ReqSeq<Req_List_New.ListSize())
            fs<<"\n";
        Req_List_New.Next();
        ReqSeq=ReqSeq+1;
    }
    fs<<";\n";
    fs<<"end;";
    fs.close();
}

// With EV case: will change the MaxGrn to a very large number for EV requested phases
void LinkList2DatFileForEV(LinkedList<ReqEntry> Req_List,char *filename,double InitTime[2],int InitPhase[2],double GrnElapse[2],RSU_Config configIS,int ChangeMaxGrn)
{
    //-- Convert request linkedlist to the NewModel.dat file for GLPK solver.
    //-- Init1 and Init2 are the initial time for the two rings while current phases are in R or Y.
    //-- MinGrn1 and MinGrn2 are the elapsed time when current phase is in Green
    //-- Add new dumy argument: ChangeMaxGrn: default value=0, no need to change the green time;
    //---otherwise, will change the max green time to a big number MaxGrnTime
	ofstream fs;

    fs.open(filename,ios::out);

    int R1No=configIS.Ring1No;
    int R2No=configIS.Ring2No;

    fs<<"data;\n";
    fs<<"param current:=0;\n";
    fs<<"param SP1:="<<InitPhase[0]<<";\n";  // This is the real phase [1-4]
    fs<<"param SP2:="<<InitPhase[1]<<";\n";  // This is the real phase [5-8]

    for(int i=0;i<2;i++)
    {
        if (InitTime[i]<0)
        {
            InitTime[i]=0;
        }
    }

    fs<<"param init1:="<<InitTime[0]<<";\n";
    fs<<"param init2:="<<InitTime[1]<<";\n";
    fs<<"param Grn1 :="<<GrnElapse[0]<<";\n";
    fs<<"param Grn2 :="<<GrnElapse[1]<<";\n";


	int MP[2];//=ConfigIS.MissPhase[i];// Missing phase
	int RlP[2]={-1,-1};//=ConfigIS.MP_Relate[i];// Missing Phase related
	int Found;

	MP[0]=ConfigIS.MissPhase[0];
	if(MP[0]>=0)
	{
		Found=FindIndexArray<int>(configIS.Phase_Seq_R1,configIS.Ring1No,MP[0]);
		if(Found<0)
			RlP[0]=ConfigIS.MP_Relate[0];
	}

	MP[1]=ConfigIS.MissPhase[1];
	if(MP[1]>=0)
	{
		Found=FindIndexArray<int>(configIS.Phase_Seq_R2,configIS.Ring2No,MP[1]-4);
		if(Found<0)
			RlP[1]=ConfigIS.MP_Relate[1];
	}



    //=================Add the information for Yellow, Red, GrnMin======//
    fs<<"param y       \t:=";
    for(int i=0;i<R1No;i++)
    {
        int k=configIS.Phase_Seq_R1[i];
        fs<<"\t"<<(k+1)<<"  "<<configIS.Yellow[k];
    }

    for(int i=0;i<R2No;i++)
    {
        int k=configIS.Phase_Seq_R2[i];
        fs<<"\t"<<(k+5)<<"  "<<configIS.Yellow[k+4];
    }
	//========================================================//
	for(int i=0;i<2;i++)
	{
		if(RlP[i]>=0)
		{
			int MP1=ConfigIS.MissPhase[i];// Missing phase
			int RlP1=ConfigIS.MP_Relate[i];// Missing Phase related
			fs<<"\t"<<(MP1+1)<<"  "<<configIS.Yellow[RlP1];
		}
	}
	//========================================================//
    fs<<";\n";

    fs<<"param red       \t:=";
    for(int i=0;i<R1No;i++)
    {
        int k=configIS.Phase_Seq_R1[i];
        fs<<"\t"<<(k+1)<<"  "<<configIS.Red[k];
    }
    for(int i=0;i<R2No;i++)
    {
        int k=configIS.Phase_Seq_R2[i];
        fs<<"\t"<<(k+5)<<"  "<<configIS.Red[k+4];
    }
	//========================================================//
	for(int i=0;i<2;i++)
	{
		if(RlP[i]>=0)
		{
			int MP1=ConfigIS.MissPhase[i];// Missing phase
			int RlP1=ConfigIS.MP_Relate[i];// Missing Phase related
			fs<<"\t"<<(MP1+1)<<"  "<<configIS.Red[RlP1];
		}
	}
	//========================================================//
    fs<<";\n";

    fs<<"param gmin      \t:=";
    for(int i=0;i<R1No;i++)
    {
        int k=configIS.Phase_Seq_R1[i];
        fs<<"\t"<<(k+1)<<"  "<<configIS.Gmin[k];
    }
    for(int i=0;i<R2No;i++)
    {
        int k=configIS.Phase_Seq_R2[i];
        fs<<"\t"<<(k+5)<<"  "<<configIS.Gmin[k+4];
    }
	//========================================================//
	for(int i=0;i<2;i++)
	{
		if(RlP[i]>=0)
		{
			int MP1=ConfigIS.MissPhase[i];// Missing phase
			int RlP1=ConfigIS.MP_Relate[i];// Missing Phase related
			fs<<"\t"<<(MP1+1)<<"  "<<configIS.Gmin[RlP1];
		}
	}
	//========================================================//
    fs<<";\n";

    //================End of the information for Yellow, Red, GrnMin======//


    // NEED to change the green max of the requested phases to a large number
    fs<<"param gmax      \t:=";

    for(int i=0;i<R1No;i++)
    {
        int k=configIS.Phase_Seq_R1[i];

        double temp_grnMax=configIS.Gmax[k]; //RequestPhaseInList
		//cout<< "temp_grnMax "<<temp_grnMax<<endl;
		//cout<< "MaxGrnTime  "<<MaxGrnTime<<endl;
        if(temp_grnMax>0)    // Phase
        {
            if(ChangeMaxGrn!=0) // EV: not only change phase
            {
                fs<<"\t"<<(k+1)<<"  "<<MaxGrnTime;
            }
            else
            {
                if(RequestPhaseInList(Req_List,(k+1)))
                {
                    fs<<"\t"<<(k+1)<<"  "<<int(configIS.Gmax[k]*(1+MaxGrnExtPortion));
                }
                else
                {
                    fs<<"\t"<<(k+1)<<"  "<<configIS.Gmax[k];
                }
            }
        }
    }
    for(int i=0;i<R2No;i++)
    {
        int k=configIS.Phase_Seq_R2[i];   // k belongs to {0~3}
        double temp_grnMax=configIS.Gmax[k+4];

        if(temp_grnMax>0)    // Phase is used.
        {
            if(ChangeMaxGrn!=0) // EV: not only change phase
            {
                fs<<"\t"<<(k+5)<<"  "<<MaxGrnTime;
            }
            else
            {
                if(RequestPhaseInList(Req_List,(k+5)))
                {
                    fs<<"\t"<<(k+5)<<"  "<<int(configIS.Gmax[k+4]*(1+MaxGrnExtPortion));
                }
                else
                {
                    fs<<"\t"<<(k+5)<<"  "<<configIS.Gmax[k+4];
                }
            }
        }
    }
	//====================MaxGreen=240======================//
	for(int i=0;i<2;i++)
	{
		//cout<< " RlP[i]   "<<RlP[i]<<endl;
		if(RlP[i]>=0)
		{
			int MP1=ConfigIS.MissPhase[i];// Missing phase
			int RlP1=ConfigIS.MP_Relate[i];// Missing Phase related
			fs<<"\t"<<(MP1+1)<<"  "<<MaxGrnTime;
		}
	}
	//========================================================//

    fs<<";\n\n";



	
	Req_List.Reset();
	int NumberofRequests=0;
	char tempID[64]=" ";
	fs<<"param priorityType:= ";
	if(Req_List.ListEmpty()==0)
    {
		while(!Req_List.EndOfList())
		{
			//cout<<"Req_List.Data().VehID" << Req_List.Data().VehID<< endl;
			if (strcmp(tempID,Req_List.Data().VehID)!=0);
			{
				
				NumberofRequests++;
				//cout<< "NumberofRequests"<<NumberofRequests<<endl;
				fs<<NumberofRequests;
				fs<<" ";
				fs<<Req_List.Data().VehClass;			
				fs<<" ";
				//cout<<"fs<<Req_List.Data().VehClass;			"<<Req_List.Data().VehClass<<endl;
				strcpy(tempID,Req_List.Data().VehID);
			}
			Req_List.Next();
		}
		while (NumberofRequests<10)
		{
			NumberofRequests++;
			fs<<NumberofRequests;
			fs<<" ";
			fs<<0;			
			fs<<" ";
		}
		fs<<" ;  \n";
	}
	fs<<"param PrioWeigth:=  1 1 2 0 3 0 4 0 5 0 6 0 7 0 8 0 9 0 10 0 ; \n";
	LinkedList<ReqEntry> Req_List_New;
    int priority;
	if(Req_List.ListEmpty()==0)
    {
        //------Here EV=1, TRANSIT=2. EV>TRANSIT-----//
        priority=FindListHighestPriority(Req_List);  //*** Req list should be not empty.***//
        //cout<<"priority"<<priority<<endl;
        FindReqWithSamePriority(Req_List,priority,Req_List_New);
    }

    fs<<"param Rl (tr): 1 2 3 4 5 6 7 8:=\n";
    Req_List_New.Reset();
    int ReqSeq=1;
    while(!Req_List_New.EndOfList())
    {
		//cout<<"Req_List_New.Data().VehClass"<<Req_List_New.Data().VehClass<<endl;
		fs<<ReqSeq<<"  ";
		for(int j=1;j<=8;j++)
		{
			if(Req_List_New.Data().Phase==j)
				fs<<max(Req_List_New.Data().ETA-ROBUSTTIME_LOW_EV,float(1.0))<<"  ";
			else
				fs<<".  ";
		}
		if(ReqSeq<Req_List_New.ListSize())
			fs<<"\n";
		ReqSeq=ReqSeq+1;
		Req_List_New.Next();
    }

    fs<<";\n";

    fs<<"param Ru (tr): 1 2 3 4 5 6 7 8:=\n";
    Req_List_New.Reset();
    ReqSeq=1;
    while(!Req_List_New.EndOfList())
    {
		fs<<ReqSeq<<"  ";
		for(int j=1;j<=8;j++)
		{
			if(Req_List_New.Data().Phase==j)
				if (Req_List_New.Data().MinGreen>0) 
				{
					fs<< ROBUSTTIME_UP_EV+Req_List_New.Data().MinGreen<<"  "; 
				}
				else
				fs<<Req_List_New.Data().ETA+ROBUSTTIME_UP_EV <<"  "; 
			else
				fs<<".  ";
		}
		if(ReqSeq<Req_List_New.ListSize())
			fs<<"\n";
		ReqSeq=ReqSeq+1;
		Req_List_New.Next();
    }
    fs<<";\n";

    fs<<"end;";
    fs.close();
}


void removeZeroWeightReq(LinkedList<ReqEntry> Req_List_,double dTrnWeight,double dTrkWeight, LinkedList<ReqEntry> &Req_List_New)
{
	Req_List_.Reset();
    Req_List_New.ClearList();
    if(Req_List_.ListEmpty()==0)
    {
        while(!Req_List_.EndOfList())
        {
			if ( Req_List_.Data().VehClass==TRANSIT )
			{
				if (dTrnWeight>0)
					Req_List_New.InsertAfter(Req_List_.Data());
			}
            else if (Req_List_.Data().VehClass==TRUCK )
            {
				if (dTrkWeight>0)
					Req_List_New.InsertAfter(Req_List_.Data());
			}else
					Req_List_New.InsertAfter(Req_List_.Data());
            Req_List_.Next();
        }
    }
}




void FindReqWithSamePriority(LinkedList<ReqEntry> Req_List_,int priority,LinkedList<ReqEntry> &Req_List_New)
{
    Req_List_.Reset();
    Req_List_New.ClearList();

    if(Req_List_.ListEmpty()==0)
    {
        while(!Req_List_.EndOfList())
        {
            if(Req_List_.Data().VehClass==priority)
            {
                Req_List_New.InsertAfter(Req_List_.Data());
            }

            Req_List_.Next();
        }
    }
}

int FindListHighestPriority(LinkedList<ReqEntry> ReqList)
{
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



int FindVehClassInList(LinkedList<ReqEntry> Req_List,int VehClass)
{
    Req_List.Reset();

    int Have=0;
	if(Req_List.ListEmpty()==0)
    {
		while(!Req_List.EndOfList())
		{
			if(Req_List.Data().VehClass==VehClass)
			{
				return (Have=1);
			}
			else
			{
				Req_List.Next();
			}
		}
	}
    return Have;
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

void PrintPlan2Log(char *resultsfile)
{
    fstream fss;
    fss.open(resultsfile,fstream::in);

    if (!fss)
    {
        cout<<"***********Error opening the plan file in order to print to the log file!\n";
        sprintf(tmp_log,"***********Error opening the plan file in order to print to a log file!\n");   
        outputlog(tmp_log);
        exit(1);
    }

    string lineread;
    
    while(!fss.eof())
    {
        getline(fss,lineread);
        strcpy(tmp_log,lineread.c_str());	
        strcat(tmp_log,"\n"); cout<<tmp_log<<endl;outputlog(tmp_log);		
    }   

    fss.close();

}

void deductSolvingTimeFromCPs(double aCriticalPoints[2][3][5],double tt2,double tt1)
{
	for (int k=0;k<2;k++)
	{
		for (int ii=0;ii<5;ii++)
		{
			adCriticalPoints[k][0][ii] = max(0.0, adCriticalPoints[k][0][ii]-(tt2-tt1));
			adCriticalPoints[k][1][ii] = max(0.0, adCriticalPoints[k][1][ii]-(tt2-tt1));
		}
	}
}
void GLPKSolver()
{
	double ttt2;
	double t1=GetSeconds();
	// The argument should be the real phase 1-8.
	struct timeval start, end;
	long mtime, seconds, useconds;
	gettimeofday(&start, NULL);

	char modFile[128]="/nojournal/bin/NewModel.mod";

	if(HaveEVInList==1)
	{
		strcpy(modFile,"/nojournal/bin/NewModel_EV.mod");
	}
	outputlog(modFile);
	outputlog("\n");

	glp_prob *mip;
	glp_tran *tran;
	int ret;
	mip = glp_create_prob();
	tran = glp_mpl_alloc_wksp();



	//ret = glp_mpl_read_model(tran, "./Mod/PriReq_26.mod", 1);
	ret = glp_mpl_read_model(tran, modFile, 1);

	if (ret != 0)
		{  fprintf(stderr, "Error on translating model!\n");
	sprintf(tmp_log,"Error on translating model!: [%.2f].\n",GetSeconds());
					outputlog(tmp_log);
	goto skip;
		}
	ret = glp_mpl_read_data(tran, "/nojournal/bin/NewModelData.dat");

	if (ret != 0)
		{  fprintf(stderr, "Error on translating data\n");
	sprintf(tmp_log,"Error on translating data: [%.2f].\n",GetSeconds());
					outputlog(tmp_log);
	goto skip;
		}
	ret = glp_mpl_generate(tran, NULL);

	if (ret != 0)
		{  fprintf(stderr, "Error on generating model\n");
	 sprintf(tmp_log,"Error on generating model: [%.2f].\n",GetSeconds());
					outputlog(tmp_log);
	goto skip;
		}

	ttt2=GetSeconds();  
	cout<<"TIME TO GENERATE THE OPTIMIZATION MODEL   "<<ttt2-t1<<endl;  
		
	glp_mpl_build_prob(tran, mip);
	glp_simplex(mip, NULL);

	glp_intopt(mip, NULL);  //modified by YF: returns 0 if solve successfully! 11142013

	ret = glp_mpl_postsolve(tran, mip, GLP_MIP);

	if (ret != 0){
		fprintf(stderr, "Error on postsolving model\n");
		sprintf(tmp_log,"Error on postsolving model: [%.2f].\n",GetSeconds());
					outputlog(tmp_log);

				}
	skip: glp_mpl_free_wksp(tran);
	glp_delete_prob(mip);
}
