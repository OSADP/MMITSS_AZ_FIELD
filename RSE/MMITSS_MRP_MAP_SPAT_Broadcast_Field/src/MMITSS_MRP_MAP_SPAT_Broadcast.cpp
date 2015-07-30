/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   



/* MMITSS_MRP_MAP_SPAT_Broadcast.cpp
*  Created by Yiheng Feng
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.

*/

#include <getopt.h>
#include <unistd.h>
#include "SignalController.h"
#include "GetInfo.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <getopt.h>
#include <unistd.h>
#include <vector>
#include <libgps.h>


#include "j2735spat.h"
#include "j2735common.h"
#include "NMAP.h"

#define LOCAL_HOST_ADDR "127.0.0.1"

#define PORT_MAP 15030
#define PORT_ART 15040

#define MAX_BUFLEN_SPAT 400
#define MAX_BUFLEN_MAP  2048

#define FLOAT2INT 10.00001


#define NUM_MM_STS 8       // Number of Maximum phases
#define NUM_LANES 10        // 6 lanes maximum per phase

#ifndef byte
    #define byte  char  // needed if byte not defined
#endif


#define CONTROLLER_IP "10.254.56.23"

char INTip[64];
char INTport[16];

char tmp_log[512];

char BROADCAST_ADDR_ATH1[64]="192.168.101.255";
char BROADCAST_ADDR_ATH0[64]="192.168.1.255";



char spat_buf[MAX_BUFLEN_SPAT];  //buf used to receive SPAT data from controller



uint8_t j2735_spat_buf[MAX_BUFLEN_SPAT]; //buf used to encode J2735 SPAT message
char j2735_map_buf[MAX_BUFLEN_MAP]; //buf used to encode J2735 MAP message
unsigned char j2735_map_buf_recv[MAX_BUFLEN_MAP]; //buf used to decode J2735 MAP message

//MAP parameters
char MAP_File_Name[64]  = "/nojournal/bin/nmap_name.txt";
char Lane_Phase_Mapping_File_Name[64]  = "/nojournal/bin/Lane_Phase_Mapping.txt";
char Inlane_Outlane_Phase_Mapping_File_Name[64] = "/nojournal/bin/InLane_OutLane_Phase_Mapping.txt";

char ART_File_Name[64]= "/nojournal/bin/requests_combined.txt";

string nmap_name;
int phase_mapping[8];    //the corresponding phases sequence: approach 1 3 5 7 through left
int appr_phase_mapping [8];  //map each phase to the approach order: appr_phase_mapping[0] -> phase1
int LaneNode_Phase_Mapping[8][8][20];           //8 approaches, at most 8 lanes each approach, at most 20 lane nodes each lane
												// the value is just the requested phase of this lane node;
MAP NewMap;
vector<LaneNodes> MAP_Nodes;

void get_map_name(); //Yiheng add 07/18/2014
void get_lane_phase_mapping();

void get_appr_phase_mapping();

void write_inlane_outlane_phase_mapping_to_file(char *filename);

int msleep(unsigned long milisec);
void PrintPhases(PhaseStatus currentPhase);
void xTimeStamp( char * pc_TimeStamp_ );


int fillspat(J2735SPATMsg_t *spat, J2735MovementState_t  *mm_st, uint8_t lane_sets[][NUM_LANES]);
int printspat(J2735SPATMsg_t *spat);
int fillmap(J2735MapData_t *map);
void printmap(J2735MapData_t *mapdata);

int gettimestamp(uint64_t *seconds, uint64_t *microsecs);


//Read ART from file requested_combined.txt and form a octet stream to be sent
void Pack_ART(byte* ART_data, char * filename, int &size);

char Ped_Status_File_Name[64]  = "/nojournal/bin/Ped_status.txt";
char Signal_status_File_Name[64] = "/nojournal/bin/signal_status.txt";
int print_Ped_Status(J2735SPATMsg_t *spat, char *filename);

int print_Signal_Status(J2735SPATMsg_t *spat, char *filename);

SignalController Daisy_Mnt;

//define log file name
char logfilename[256]   = "/nojournal/bin/log/MMITSS_MRP_MAP_SPAT_BROADCAST_";
char IPInfo[64]			= "/nojournal/bin/ntcipIP.txt";

int field_flag=0; //0: in the lab  1: in the field

//~ int gps_init ();

static void usage(char *name)
{
    printf("%s [-hofc] [Number] Y\n"
           "   -d(0): Green HOLD on number\n"
           "   -o(2): Phase OMIT on number\n"
           "   -f(1): FORCEOFF ring on number\n"
           "   -c(3): phase CALL on number\n"
		   "The [Number] should be the binary value of the phase is controlled in(xxxx xxxx)\n "
           "\n", name);
}

//Savari GPS data structure
//~ gps_data_t *gps_handle;
//~ savari_gps_data_t gps;


int main ( int argc, char* argv[] )
    {
	int i,j,k;
		int ret;

    unsigned int type = 0;
    

    
    if (argc>1)
	{
		sscanf(argv[1],"%s",BROADCAST_ADDR_ATH1);   // Obtain the broadcasting address from user. If Communication is through WME the address should be 127.0.0.1 
		sscanf(argv[2],"%s",BROADCAST_ADDR_ATH0);
		sscanf(argv[3],"%d",&field_flag);
	}
	
	
	//gps_init ();


    //------log file name with Time stamp---------------------------
    char timestamp[128];
    //char tmp_log[64];
    xTimeStamp(timestamp);
    strcat(logfilename,timestamp);strcat(logfilename,".log");

	std::fstream fs;
	fs.open(logfilename, fstream::out | fstream::trunc);

    //------end log file name-----------------------------------
    //-----------------Beginning of Wireless communication------------//
    sprintf(tmp_log,"Now to Read the Signal Controller IP information:\n");
	cout<<tmp_log;  outputlog(tmp_log);
    
	get_ip_address();


    int SleepTime=1000; // in ms

    for(int i=4;i>0;i--) //warm up...
        {
        Daisy_Mnt.PhaseRead();//IntersectionPhaseRead();
        Daisy_Mnt.UpdatePhase();//Phases.UpdatePhase(phase_read);
        Daisy_Mnt.Phases.Display();//Phases.Display();
        msleep(1000);
        }
	
	
	//Set controller to broadcast SPAT
	int Setvalue=6;   //with pedestrain data
	Daisy_Mnt.SPATSet(Setvalue);  //Set to broadcast the SPAT with a value
	
	//------------init: Begin of Network connection------------------------------------
	int sockfd;
	int broadcast=1;

	struct sockaddr_in sendaddr;
	struct sockaddr_in recvaddr;
	int numbytes, addr_len;

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
	sendaddr.sin_port = htons(6053);  //*** IMPORTANT: the vissim,signal control and performance observer should also have this port. ***//
	sendaddr.sin_addr.s_addr = INADDR_ANY;//inet_addr(LOCAL_HOST_ADDR);//inet_addr(OBU_ADDR);//INADDR_ANY;

	memset(sendaddr.sin_zero,'\0',sizeof sendaddr.sin_zero);

	if(bind(sockfd, (struct sockaddr*) &sendaddr, sizeof sendaddr) == -1)
	{
		perror("bind");        exit(1);
	}

	recvaddr.sin_family = AF_INET;
	recvaddr.sin_port = htons(PORT_MAP);
	recvaddr.sin_addr.s_addr = inet_addr(BROADCAST_ADDR_ATH1) ; //INADDR_BROADCAST;
	memset(recvaddr.sin_zero,'\0',sizeof recvaddr.sin_zero);
	int addr_length = sizeof ( recvaddr );
	int recv_data_len;
	//-----------------------End of Network Connection------------------//
	
	int sockfd_ART;
	struct sockaddr_in recvaddr_ART;
	int addr_len_ART;
	
	if((sockfd_ART = socket(AF_INET,SOCK_DGRAM,0)) == -1)
	{
		perror("sockfd_ART");
		exit(1);
	}
	
	if((setsockopt(sockfd_ART,SOL_SOCKET,SO_BROADCAST,
		&broadcast,sizeof broadcast)) == -1)
	{
		perror("setsockopt - SO_SOCKET ");
		exit(1);
	}
		
	recvaddr_ART.sin_family = AF_INET;
	recvaddr_ART.sin_port = htons(PORT_ART);
	recvaddr_ART.sin_addr.s_addr = inet_addr(BROADCAST_ADDR_ATH0) ; //INADDR_BROADCAST;
	memset(recvaddr_ART.sin_zero,'\0',sizeof recvaddr_ART.sin_zero);
	
	
/////////////////////////////////////////////Read the MAP description file and do the lane-phase mapping

	NewMap.ID=1;
	NewMap.Version=1;
	// Parse the MAP file
	get_map_name();
	char mapname[128];
	sprintf(mapname,"%s",nmap_name.c_str());
	printf("%s",mapname);
	NewMap.ParseIntersection(mapname);

	sprintf(tmp_log,"Read the map successfully At (%d).\n",time(NULL));
	outputlog(tmp_log); cout<<tmp_log;

	
for(i=0;i<8;i++)
{
	for(j=0;j<8;j++)
	{
		for(k=0;k<20;k++)
		{
			LaneNode_Phase_Mapping[i][j][k]=0;
		}
	}
}	


//store all nodes information to MAP_Nodes after parsing the message
	//This is used for calculating vehicle positions in the MAP
	for (i=0;i<NewMap.intersection.Approaches.size();i++)
		for(j=0;j<NewMap.intersection.Approaches[i].Lanes.size();j++)
			for(k=0;k<NewMap.intersection.Approaches[i].Lanes[j].Nodes.size();k++)
			{	
				LaneNodes temp_node;
				temp_node.index.Approach=NewMap.intersection.Approaches[i].Lanes[j].Nodes[k].index.Approach;
				temp_node.index.Lane=NewMap.intersection.Approaches[i].Lanes[j].Nodes[k].index.Lane;
				temp_node.index.Node=NewMap.intersection.Approaches[i].Lanes[j].Nodes[k].index.Node;
				MAP_Nodes.push_back(temp_node);
												
				//sprintf(tmp_log,"%d %d %d\n",temp_node.index.Approach,temp_node.index.Lane,temp_node.index.Node);
				//outputlog(tmp_log); cout<<tmp_log;
			}

//Construct the lane node phase mapping matrix
get_lane_phase_mapping();

for(int iii=0;iii<MAP_Nodes.size();iii++)
{
	int flag=0;
	for (i=0;i<NewMap.intersection.Approaches.size();i++)
	{
		for(j=0;j<NewMap.intersection.Approaches[i].Lanes.size();j++)
		{
			for(k=0;k<NewMap.intersection.Approaches[i].Lanes[j].Nodes.size();k++)
			{
				if(NewMap.intersection.Approaches[i].Lanes[j].Nodes[k].index.Approach==MAP_Nodes[iii].index.Approach &&
					NewMap.intersection.Approaches[i].Lanes[j].Nodes[k].index.Lane==MAP_Nodes[iii].index.Lane)
				{
					//determine requesting phase
					if (MAP_Nodes[iii].index.Approach==1)  //south bound
					{
						if (NewMap.intersection.Approaches[i].Lanes[j].Attributes[1]==1)  //through
							LaneNode_Phase_Mapping[MAP_Nodes[iii].index.Approach][MAP_Nodes[iii].index.Lane][MAP_Nodes[iii].index.Node]=phase_mapping[0];
						if (NewMap.intersection.Approaches[i].Lanes[j].Attributes[2]==1)  //left turn
							LaneNode_Phase_Mapping[MAP_Nodes[iii].index.Approach][MAP_Nodes[iii].index.Lane][MAP_Nodes[iii].index.Node]=phase_mapping[1];
					}
					if (MAP_Nodes[iii].index.Approach==3)
					{
						if (NewMap.intersection.Approaches[i].Lanes[j].Attributes[1]==1)  //through
							LaneNode_Phase_Mapping[MAP_Nodes[iii].index.Approach][MAP_Nodes[iii].index.Lane][MAP_Nodes[iii].index.Node]=phase_mapping[2];
						if (NewMap.intersection.Approaches[i].Lanes[j].Attributes[2]==1)  //left turn
							LaneNode_Phase_Mapping[MAP_Nodes[iii].index.Approach][MAP_Nodes[iii].index.Lane][MAP_Nodes[iii].index.Node]=phase_mapping[3];
					}
					if (MAP_Nodes[iii].index.Approach==5)
					{
						if (NewMap.intersection.Approaches[i].Lanes[j].Attributes[1]==1)  //through
							LaneNode_Phase_Mapping[MAP_Nodes[iii].index.Approach][MAP_Nodes[iii].index.Lane][MAP_Nodes[iii].index.Node]=phase_mapping[4];
						if (NewMap.intersection.Approaches[i].Lanes[j].Attributes[2]==1)  //left turn
							LaneNode_Phase_Mapping[MAP_Nodes[iii].index.Approach][MAP_Nodes[iii].index.Lane][MAP_Nodes[iii].index.Node]=phase_mapping[5];
					}
					if (MAP_Nodes[iii].index.Approach==7)
					{
						if (NewMap.intersection.Approaches[i].Lanes[j].Attributes[1]==1)  //through
							LaneNode_Phase_Mapping[MAP_Nodes[iii].index.Approach][MAP_Nodes[iii].index.Lane][MAP_Nodes[iii].index.Node]=phase_mapping[6];
						if (NewMap.intersection.Approaches[i].Lanes[j].Attributes[2]==1)  //left turn
							LaneNode_Phase_Mapping[MAP_Nodes[iii].index.Approach][MAP_Nodes[iii].index.Lane][MAP_Nodes[iii].index.Node]=phase_mapping[7];
					}
					flag=1;
					break;
				}
			}
			if(flag==1)
				break;
		}
		if(flag==1)
			break;
	}
}

//Read Map data and contruct the J2735MAP Message
J2735MapData_t j2735_map,j2735_map_de;
memset(&j2735_map, 0, sizeof(j2735_map));
fillmap(&j2735_map);

//j2735_free_mapdata(&j2735_map);
int enc_len;
enc_len = j2735_encode_mapdata(&j2735_map,j2735_map_buf, MAX_BUFLEN_MAP);
    if (enc_len < 0) {
        printf("MAP: Encode Failure\n");
    } else {
        printf("MAP: Encode Success\n");
    }

printf("The MAP size is is: %d\n",enc_len);

j2735_dump_hex("MAP:", (unsigned char*)j2735_map_buf, enc_len);
    memset(&j2735_map_de, 0, sizeof(j2735_map_de));
    ret = j2735_decode_mapdata(&j2735_map_de,(unsigned char*)j2735_map_buf, enc_len);
    if (ret < 0) {
        printf("MAP: Decode Failure\n");
    } else {
        printf("MAP: Decode Success\n");
    }
printmap(&j2735_map_de);
j2735_free_mapdata(&j2735_map_de);


//map each phase to approach number
get_appr_phase_mapping();


//Construct the inlane-outlane-phase mapping file for PRS
write_inlane_outlane_phase_mapping_to_file(Inlane_Outlane_Phase_Mapping_File_Name);

////////////////////////End of reading map and lane phase mapping

sprintf(tmp_log,"Signal Status Data: \n");
outputlog(tmp_log);


int count=0;
    while ( true )
    {

        sprintf(tmp_log,"\n..........In the While LOOP........:\n");
        cout<<tmp_log;       // outputlog(tmp_log);

		//Get SPAT from UDP port 6053	
		recv_data_len = recvfrom(sockfd, spat_buf, sizeof(spat_buf), 0,
                        (struct sockaddr *)&sendaddr, (socklen_t *)&addr_length);
                                            	
		//cout<<"The Received SPAT data bytes are: "<<recv_data_len<<endl;
		cout<<"Message Count: "<<count<<endl;
		
		//Read the SPAT Data and MAP Data to Create SPAT Message
		J2735SPATMsg_t spat;
		J2735MovementState_t  mm_st[NUM_MM_STS];
		uint8_t lane_sets[NUM_MM_STS][NUM_LANES];
		
		
		fillspat(&spat, mm_st, lane_sets);
		
		//Write signal status to log file
		print_Signal_Status(&spat,Signal_status_File_Name);	

		
		//printspat(&spat);
		
		print_Ped_Status(&spat,Ped_Status_File_Name);
		
		
		
		ret = j2735_encode_spat(&spat,j2735_spat_buf, MAX_BUFLEN_SPAT);
		if (ret < 0) {
        printf("SPAT: Encode Failure\n");
		} else {
        printf("SPAT: Encode Success, the length of SPAT message is: %d\n",ret);
		}
			
		//j2735_dump_hex("SPAT:", (unsigned char*)j2735_spat_buf, ret);
		
		sendto(sockfd,j2735_spat_buf,ret, 0,(struct sockaddr *)&recvaddr, addr_length);
		cout<<"Broadcast J2735 SPAT Message!"<<endl;		
	
		count++;
		
		if(count%10==0)
		{
			//j2735_dump_hex("MAP:", (unsigned char*)j2735_map_buf, enc_len);
			
			//Send MAP every 1 second			
			sendto(sockfd,j2735_map_buf,enc_len, 0,(struct sockaddr *)&recvaddr, addr_length);
			cout<<"Broadcast J2735 MAP Message!"<<endl;
						
			//Send ART every 1 second			
			//Read ART from file requested_combined.txt and form a octet stream to be sent
			int ART_size=0;
			byte ART_buf[1024];
			Pack_ART(ART_buf, ART_File_Name,ART_size);
			char *ART_buf_send;
			ART_buf_send=new char[ART_size];
			for (i=0;i<ART_size;i++)
			{
				ART_buf_send[i]=ART_buf[i];
			}
			sendto(sockfd_ART,ART_buf_send,ART_size, 0,(struct sockaddr *)&recvaddr_ART, addr_length);
			cout<<"Broadcast Active Request Table with size "<<ART_size<<endl;;
			
		}
		
	
		memset(&spat, 0, sizeof(spat));
		
		ret = j2735_decode_spat(&spat, j2735_spat_buf, MAX_BUFLEN_SPAT);
		if ( ret < 0 ) {
			printf("SPAT: Decode Failure\n");
		} else {
			printf("SPAT: Decode Sucess\n");
			//printspat(&spat);
        j2735_free_spat_contents_only(&spat);
		}
		
		
		if(count>=10000)
		count=0;
    }

    fs.close();
   
    return 0;
}



void PrintPhases(PhaseStatus currentPhase)
    {
    char tmp_log[128]="Phase Information: ";
    char tmp_phase[16];
    for(int i=0;i<numPhases;i++)
        {
        sprintf(tmp_phase," %d",currentPhase.phaseColor[i]);
        strcat(tmp_log,tmp_phase);
        }
    outputlog(tmp_log);
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

void get_map_name() //Yiheng add 07/18/2014
{
    fstream fs;

    fs.open(MAP_File_Name);

    char temp[128];

    getline(fs,nmap_name);

    if(nmap_name.size()!=0)
    {
        //std::cout<< "Current Vehicle ID:" << RSUID <<std::endl;
        sprintf(temp,"Current MAP is %s\n",nmap_name.c_str());
        cout<<temp<<endl;
        outputlog(temp);
    }
    else
    {
        sprintf(temp,"Reading MAP_File problem");
        cout<<temp<<endl;
        outputlog(temp);
        exit(0);
    }

    fs.close();
}

void get_lane_phase_mapping() //Yiheng add 07/18/2014
{
    fstream fs;

    fs.open(Lane_Phase_Mapping_File_Name);

    char temp[128];
	
	string temp_string;

    getline(fs,temp_string);  //First line is comment
	getline(fs,temp_string);  //Second line contains information


    if(temp_string.size()!=0)
    {
		char tmp[128];
		strcpy(tmp,temp_string.c_str());		
		sscanf(tmp,"%d %d %d %d %d %d %d %d",&phase_mapping[0],&phase_mapping[1],&phase_mapping[2],&phase_mapping[3],&phase_mapping[4],&phase_mapping[5],&phase_mapping[6],&phase_mapping[7]);
    }
    else
    {
        sprintf(temp,"Reading Lane_Phase_Mapping_File problem");
        cout<<temp<<endl;
        outputlog(temp);
        exit(0);
    }

    fs.close();
}

int gettimestamp(uint64_t *seconds, uint64_t *microsecs)
{
    struct timeval tv;

    gettimeofday(&tv, 0);

    *seconds = tv.tv_sec;
    *microsecs = tv.tv_usec;

    return 0;
}

void get_appr_phase_mapping()
{
	int i;
	for (i=0;i<8;i++)
	{	
		if(phase_mapping[i]!=0)
		{
			if(i%2==0)
				appr_phase_mapping[phase_mapping[i]-1]= (i+1);
			if(i%2==1)
				appr_phase_mapping[phase_mapping[i]-1]= i;
		}	
	}
	
	cout<<"appr_phase_mapping:"<<endl;
	for(i=0;i<8;i++)
	cout<<appr_phase_mapping[i]<<" ";
	cout<<endl;

}



int fillspat(J2735SPATMsg_t *spat, J2735MovementState_t  *mm_st,
                        uint8_t lane_sets[][NUM_LANES])
{
	unsigned char byteA;
	unsigned char byteB;
	unsigned short tempUShort;
	long tempLong;
	int Status_red;
	int Status_green;
	int Status_yellow;
	int Ped_status_walk;
	int Ped_status_flash;
	int Ped_status_dontwalk;
	int time_remaining;
	
    int i,j,k;
    uint64_t secs, usecs;
    
    int lane_no;  //the number of lanes of one phase
    int tmp_lane_id;

    memset(spat, 0, sizeof(*spat));

    spat->intersection_id = NewMap.intersection.ID;
    spat->intersection_status = J2735SPAT_STATUS_MANUAL_CONTROL;
    gettimestamp(&secs, &usecs);
    spat->timestamp_ms = (secs*1000) + (usecs/1000);
    spat->num_movement_states = NUM_MM_STS;
    spat->movement_states = mm_st;
    //Movement state is defined as phases e.g. mm_st[0] -> phase 1   mm_st[7] -> phase 8      
    for(i=0; i <  NUM_MM_STS; i++)
    {
		if (appr_phase_mapping[i]!=0)   //Phase exist!!!!!!!!!!
		{
			///////////////////////Number of lanes and Lane Set -> from MAP
			lane_no=0;
			mm_st[i].lane_set = lane_sets[i];
			
			k=0;
			for (j=0;j<NewMap.intersection.Approaches[appr_phase_mapping[i]-1].Lane_No;j++)
			{
				
				if (i==0 ||i==2 ||i==4 ||i==6)   //phase 1 3 5 7, left turn phases
				{
					if (NewMap.intersection.Approaches[appr_phase_mapping[i]-1].Lanes[j].Attributes[2]==1)
					{
						lane_no++;
						tmp_lane_id=NewMap.intersection.Approaches[appr_phase_mapping[i]-1].Lanes[j].ID;
						lane_sets[i][k]=tmp_lane_id;						
						//cout<<"Lane ID for Phase "<<i+1<<" is: "<<tmp_lane_id<<endl;
						k++;			
					}
				}
				else //phase 2 4 6 8, through phases
				{
					if (NewMap.intersection.Approaches[appr_phase_mapping[i]-1].Lanes[j].Attributes[1]==1)
					{
						lane_no++;
						tmp_lane_id=NewMap.intersection.Approaches[appr_phase_mapping[i]-1].Lanes[j].ID;
						lane_sets[i][k]=tmp_lane_id;
						//cout<<"Lane ID for Phase "<<i+1<<" is: "<<tmp_lane_id<<endl;
						k++;
					}
				}			
			}
			mm_st[i].num_lanes = lane_no;
			//cout<<"Total number of lane: "<<mm_st[i].num_lanes<<endl;
			
			///////////////////////Signal Status -> from controller SPAT data	
			
			mm_st[i].signal_state.present = 1;
			mm_st[i].next_signal_state.present = 1;
			
			//do signal status byte: 210-215 of SPAT
			byteA=spat_buf[210];
			byteB=spat_buf[211];
			Status_red=(int)(((byteA << 8) + (byteB)));
			byteA=spat_buf[212];
			byteB=spat_buf[213];
			Status_yellow=(int)(((byteA << 8) + (byteB)));
			byteA=spat_buf[214];
			byteB=spat_buf[215];
			Status_green=(int)(((byteA << 8) + (byteB)));			
			int tmp_sig_status=1<<i;  //get the bit value of this phase
			if(Status_green & tmp_sig_status)
			{
				mm_st[i].signal_state.ball = J2735SPAT_BALL_GREEN;
				mm_st[i].next_signal_state.ball = J2735SPAT_BALL_YELLOW;
				if (i==0 ||i==2 ||i==4 ||i==6)   //phase 1 3 5 7, left turn phases
				{
					mm_st[i].signal_state.arrow = J2735SPAT_ARROW_GREEN_LEFT;
					mm_st[i].next_signal_state.arrow = J2735SPAT_ARROW_YELLOW_LEFT;
				}
				else
				{
					mm_st[i].signal_state.arrow = J2735SPAT_ARROW_GREEN_STRAIGHT;
					mm_st[i].next_signal_state.arrow = J2735SPAT_ARROW_YELLOW_STRAIGHT;
				}
				
			}
			else if (Status_yellow &tmp_sig_status)
			{
				mm_st[i].signal_state.ball = J2735SPAT_BALL_YELLOW;
				mm_st[i].next_signal_state.ball = J2735SPAT_BALL_RED;
				if (i==0 ||i==2 ||i==4 ||i==6)   //phase 1 3 5 7, left turn phases
				{
					mm_st[i].signal_state.arrow = J2735SPAT_ARROW_YELLOW_LEFT;
					mm_st[i].next_signal_state.arrow = J2735SPAT_ARROW_RED_LEFT;
				}
				else
				{
					mm_st[i].signal_state.arrow = J2735SPAT_ARROW_YELLOW_STRAIGHT;
					mm_st[i].next_signal_state.arrow = J2735SPAT_ARROW_RED_STRAIGHT;
				}
			}
			else if (Status_red &tmp_sig_status)
			{
				mm_st[i].signal_state.ball = J2735SPAT_BALL_RED;
				mm_st[i].next_signal_state.ball = J2735SPAT_BALL_GREEN;
				if (i==0 ||i==2 ||i==4 ||i==6)   //phase 1 3 5 7, left turn phases
				{
					mm_st[i].signal_state.arrow = J2735SPAT_ARROW_RED_LEFT;
					mm_st[i].next_signal_state.arrow = J2735SPAT_ARROW_GREEN_LEFT;
				}
				else
				{
					mm_st[i].signal_state.arrow = J2735SPAT_ARROW_RED_STRAIGHT;
					mm_st[i].next_signal_state.arrow = J2735SPAT_ARROW_GREEN_STRAIGHT;
				}
			}
			else
			{
				mm_st[i].signal_state.ball = J2735SPAT_BALL_DARK;
				mm_st[i].signal_state.arrow = J2735SPAT_ARROW_NONE;
			}
			cout<<"status: "<<mm_st[i].signal_state.ball<<" Arrow: "<<mm_st[i].signal_state.arrow<<endl;
			//////////////////////////////End of signal status
			
			//do Ped status byte: 216-221 of SPAT
			byteA=spat_buf[216];
			byteB=spat_buf[217];
			Ped_status_dontwalk=(int)(((byteA << 8) + (byteB)));
			byteA=spat_buf[218];
			byteB=spat_buf[219];
			Ped_status_flash=(int)(((byteA << 8) + (byteB)));
			byteA=spat_buf[220];
			byteB=spat_buf[221];
			Ped_status_walk=(int)(((byteA << 8) + (byteB)));
			
			int tmp_ped_sig_status=1<<i;  //get the bit value of this phase
			if(Ped_status_walk & tmp_ped_sig_status)
			{
				mm_st[i].ped_state = J2735SPAT_PED_WALK;
				mm_st[i].next_ped_state = J2735SPAT_PED_CAUTION;
			}
			else if (Ped_status_flash &tmp_ped_sig_status)
			{
				mm_st[i].ped_state = J2735SPAT_PED_CAUTION;
				mm_st[i].next_ped_state = J2735SPAT_PED_STOP;
			}
			else if (Ped_status_dontwalk &tmp_ped_sig_status)
			{
				mm_st[i].ped_state = J2735SPAT_PED_STOP;
				mm_st[i].next_ped_state = J2735SPAT_PED_WALK;
			}
			else
			{
				mm_st[i].ped_state = J2735SPAT_PED_UNAVAILABLE;
				mm_st[i].next_ped_state = J2735SPAT_PED_UNAVAILABLE;
			}			
			//cout<<"ped status: "<<mm_st[i].ped_state<<endl;
			//////////////////////////////////////////End of Ped signal status
			
			mm_st[i].special_signal_state = J2735SPAT_SPECIAL_STATE_NOT_PRESENT;
			
			//do time remaining and state confidence: always do the minimum time to change (may not be accurate!!!!)
			byteA=spat_buf[3+i*13];   //MinTimeTo Change
			byteB=spat_buf[4+i*13];
			time_remaining=(int)(((byteA << 8) + (byteB)));
			mm_st[i].time_remaining = time_remaining;
			
			cout<<"Signal remaining time is:"<<time_remaining<<endl;
			
			mm_st[i].state_conf = J2735SPAT_CONF_MINTIME;
			
			
			byteA=spat_buf[7+i*13];   //Ped MinTimeTo Change
			byteB=spat_buf[8+i*13];
			time_remaining=(int)(((byteA << 8) + (byteB)));
			mm_st[i].next_time_remaining = time_remaining;   //The next_time_remaining is used as the ped remaining time!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			mm_st[i].next_state_conf = J2735SPAT_CONF_UNKNOWN;
			//cout<<"ped remaining time: "<<time_remaining<<endl;
			//printf("ped remaining time: %d\n",mm_st[i].next_time_remaining);
			///////////////////////////////end of time remaining and state confidence
			
		}	
		else   //Phase doesn't exist!!!!!!!!!!!!!!!!!
		{
			mm_st[i].num_lanes =1;
			mm_st[i].lane_set = lane_sets[i];
			lane_sets[i][0]=0;
			mm_st[i].signal_state.present = 0;
			mm_st[i].next_signal_state.present = 0;
			mm_st[i].signal_state.ball = J2735SPAT_BALL_DARK;
            mm_st[i].signal_state.arrow = J2735SPAT_ARROW_NONE;
            mm_st[i].ped_state = J2735SPAT_PED_UNAVAILABLE;
            mm_st[i].next_signal_state.ball = J2735SPAT_BALL_DARK;
            mm_st[i].next_signal_state.arrow = J2735SPAT_ARROW_NONE;
            mm_st[i].next_ped_state = J2735SPAT_PED_UNAVAILABLE;
            mm_st[i].special_signal_state = J2735SPAT_SPECIAL_STATE_NOT_PRESENT;
			mm_st[i].time_remaining = 12002;
			mm_st[i].state_conf = J2735SPAT_CONF_UNKNOWN;
			mm_st[i].next_time_remaining = 0;
			mm_st[i].next_state_conf = J2735SPAT_CONF_UNKNOWN;
		}
    }
    return 0;
}

int printspat(J2735SPATMsg_t *spat)
{
    int i,j;
    uint64_t secs, usecs;
    J2735MovementState_t  *mm_st;
    uint64_t now;

    printf("intersection id:                    0x%x\n", spat->intersection_id);
    printf("intersectin status:                 %d\n",
                                                    spat->intersection_status);
    printf("gen time:                           %llu\n", spat->timestamp_ms);
    gettimestamp(&secs, &usecs);
    now = (secs*1000) + (usecs/1000);
    printf("curr time:                          %llu\n", now);
    printf("num movement states:                %ld\n",
                                                    spat->num_movement_states);
    mm_st = spat->movement_states;
    for(i=0; i <  NUM_MM_STS; i++){
        printf("movement state                  %d\n", i+1);
        printf("    num_lanes:                  %ld\n", mm_st[i].num_lanes);
        printf("    lane set:\n");
        printf("    ");
        for(j=0; j < mm_st[i].num_lanes; j++)
            printf("%u ", mm_st[i].lane_set[j]);
        printf("\n");
        printf("    signal state present:       %d\n",
                                            mm_st[i].signal_state.present);
        printf("    next signal state present:  %d\n",
                                            mm_st[i].next_signal_state.present);
        printf("    ball:                       0x%x\n",
                                            mm_st[i].signal_state.ball);
        printf("    arrow:                      0x%x\n",
                                            mm_st[i].signal_state.arrow);
        printf("    ped state:                  %d\n",
                                            mm_st[i].ped_state);

        printf("    next ball:                  0x%x\n",
                                            mm_st[i].next_signal_state.ball);
        printf("    next arrow:                 0x%x\n",
                                            mm_st[i].next_signal_state.arrow);
        printf("    next ped state:             %d\n",
                                            mm_st[i].next_ped_state);
        printf("    special state:              %d\n",
                                            mm_st[i].special_signal_state);
        printf("    time remaining:             %ld\n",
                                            mm_st[i].time_remaining);
        printf("    signal state conf:          %d\n",
                                            mm_st[i].state_conf);
        printf("    next time remaining:        %ld\n",
                                            mm_st[i].next_time_remaining);
        printf("    next state conf:            %d\n",
                                            mm_st[i].next_state_conf);
    }
    return 0;
}

int fillmap(J2735MapData_t *map)
{
	int i,j,k;
    J2735Approach_t *ap;
    J2735Lane_t     *ln;
    J2735Position3D_t *wp;

    memset(map, 0, sizeof(*map));

    map->intersection_id = NewMap.intersection.ID;
    map->msg_count = 111;

    // reference point - de la cruz @ reed
    map->reference_point.latitude = NewMap.intersection.Ref_Lat;
    map->reference_point.longitude = NewMap.intersection.Ref_Long;
    map->reference_point.elevation = NewMap.intersection.Ref_Ele;

    map->num_approaches = (int) NewMap.intersection.Appro_No/2;  //The J2735 MAP message consider only 4 approaches
    printf("Total Number of Approaches is: %d \n",map->num_approaches);
    map->approaches= new J2735Approach_t[4];
    for(i=0;i<map->num_approaches;i++) 
    {
		
		map->approaches[i].approach_number=i*2+1;  //Approach number is 1,3,5,7 only the ingress lanes of the Batelle MAP
		//printf("Approach: %d\n",map->approaches[i].approach_number);
		map->approaches[i].num_ingress_lanes=NewMap.intersection.Approaches[i*2].Lane_No;   //Number of lanes of approach 1,3,5,7
		//printf("No Ingress Lane is: %d\n",map->approaches[i].num_ingress_lanes);
		
		map->approaches[i].num_crosswalks=0;  //no crosswalks
		
		//Do ingress lanes
		map->approaches[i].ingress_lanes= new J2735Lane_t[NewMap.intersection.Approaches[i*2].Lane_No];
		for(j=0;j<map->approaches[i].num_ingress_lanes;j++)
		{
			map->approaches[i].ingress_lanes[j].lane_number=NewMap.intersection.Approaches[i*2].Lanes[j].ID;
			map->approaches[i].ingress_lanes[j].lane_width=NewMap.intersection.Approaches[i*2].Lanes[j].Width;
			//printf("Lane ID: %d Lane Width: %d\n",map->approaches[i].ingress_lanes[j].lane_number,map->approaches[i].ingress_lanes[j].lane_width);
			
			//Do lane attributes
			if(NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[1]==1 && NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[3]==0) //straight permitted
			map->approaches[i].ingress_lanes[j].lane_attributes=J2735_LANE_ATTR_STRAIGHT_ALLOWED;
			if(NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[2]==1) //left turn permitted
			map->approaches[i].ingress_lanes[j].lane_attributes=J2735_LANE_ATTR_LEFT_ALLOWED;
			if(NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[3]==1 && NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[1]==0) //right turn permitted
			map->approaches[i].ingress_lanes[j].lane_attributes=J2735_LANE_ATTR_RIGHT_ALLOWED;
			if(NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[3]==1 && NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[1]==1) // through and right turn permitted
			map->approaches[i].ingress_lanes[j].lane_attributes=J2735_LANE_ATTR_STRAIGHT_ALLOWED;
			//printf("Lane Attributes: %d\n",map->approaches[i].ingress_lanes[j].lane_attributes);
			
			//Do Way Points
			map->approaches[i].ingress_lanes[j].num_way_points=NewMap.intersection.Approaches[i*2].Lanes[j].Node_No;
			map->approaches[i].ingress_lanes[j].way_points= new J2735Position3D_t[NewMap.intersection.Approaches[i*2].Lanes[j].Node_No];
			for(k=0;k<map->approaches[i].ingress_lanes[j].num_way_points;k++)
			{
				map->approaches[i].ingress_lanes[j].way_points[k].latitude=NewMap.intersection.Approaches[i*2].Lanes[j].Nodes[k].Latitude;
				map->approaches[i].ingress_lanes[j].way_points[k].longitude=NewMap.intersection.Approaches[i*2].Lanes[j].Nodes[k].Longitude;
				//printf("Waypoints %d is: %lf %lf\n",k+1,map->approaches[i].ingress_lanes[j].way_points[k].latitude,map->approaches[i].ingress_lanes[j].way_points[k].longitude);
			}
			//Do Lane Connection
			map->approaches[i].ingress_lanes[j].num_connects_to_lanes=NewMap.intersection.Approaches[i*2].Lanes[j].Connection_No;
			map->approaches[i].ingress_lanes[j].connects_to_lanes= new uint8_t[map->approaches[i].ingress_lanes[j].num_connects_to_lanes];
			for(k=0;k<map->approaches[i].ingress_lanes[j].num_connects_to_lanes;k++)
			{
				map->approaches[i].ingress_lanes[j].connects_to_lanes[k]=NewMap.intersection.Approaches[i*2].Lanes[j].Connections[k].ConnectedLaneName.Approach*10+
																		NewMap.intersection.Approaches[i*2].Lanes[j].Connections[k].ConnectedLaneName.Lane;    //e.g. lane 1.2 = 12!!!!
																																		//Should apply same rule after decoding the map data
				//printf("Connection lane is: %d\n",map->approaches[i].ingress_lanes[j].connects_to_lanes[k]);
			}
			
		}
		
		//Do Engress lanes
		map->approaches[i].num_egress_lanes=NewMap.intersection.Approaches[i*2+1].Lane_No;   //Number of lanes of approach 2,4,6,8
		printf("No Engress Lane is: %d\n",map->approaches[i].num_egress_lanes);
		map->approaches[i].egress_lanes= new J2735Lane_t[NewMap.intersection.Approaches[i*2+1].Lane_No];
		for(j=0;j<map->approaches[i].num_egress_lanes;j++)
		{
			map->approaches[i].egress_lanes[j].lane_number=NewMap.intersection.Approaches[i*2+1].Lanes[j].ID;
			map->approaches[i].egress_lanes[j].lane_width=NewMap.intersection.Approaches[i*2+1].Lanes[j].Width;
			//printf("Lane ID: %d Lane Width: %d\n",map->approaches[i].egress_lanes[j].lane_number,map->approaches[i].egress_lanes[j].lane_width);
			
			//Do lane attributes
			map->approaches[i].egress_lanes[j].lane_attributes=J2735_LANE_ATTR_STRAIGHT_ALLOWED;  //all egress lanes are straight allowed
			//printf("Lane Attributes: %d\n",map->approaches[i].egress_lanes[j].lane_attributes);
			
			//Do Way Points
			map->approaches[i].egress_lanes[j].num_way_points=NewMap.intersection.Approaches[i*2+1].Lanes[j].Node_No;
			map->approaches[i].egress_lanes[j].way_points= new J2735Position3D_t[NewMap.intersection.Approaches[i*2+1].Lanes[j].Node_No];
			for(k=0;k<map->approaches[i].egress_lanes[j].num_way_points;k++)
			{
				map->approaches[i].egress_lanes[j].way_points[k].latitude=NewMap.intersection.Approaches[i*2+1].Lanes[j].Nodes[k].Latitude;
				map->approaches[i].egress_lanes[j].way_points[k].longitude=NewMap.intersection.Approaches[i*2+1].Lanes[j].Nodes[k].Longitude;
				//printf("Waypoints %d is: %lf %lf\n",k+1,map->approaches[i].egress_lanes[j].way_points[k].latitude,map->approaches[i].egress_lanes[j].way_points[k].longitude);
			}
			//Do Lane Connection
			map->approaches[i].egress_lanes[j].num_connects_to_lanes=0;    //No Connection for egress lane
			map->approaches[i].egress_lanes[j].connects_to_lanes=NULL;
			//printf("No Connection for egress lane \n");			
		}
	}	
    
    return 1;
}

void printlane(char *msg, J2735Lane_t *ln)
{
    int i;
    J2735Position3D_t   *wp;
    printf("  %s # %u, width: %u, attributes: 0x%X\n",
                  msg, ln->lane_number, ln->lane_width, ln->lane_attributes);
    for(i=0; i < ln->num_way_points; i++){
        wp = &ln->way_points[i];
        printf("    Way point # %u, %f,%f\n", i,
               wp->latitude, wp->longitude);
    }
    printf("    Connects to: ");
    for(i=0; i < ln->num_connects_to_lanes; i++){
        printf("%u,", ln->connects_to_lanes[i]);
    }
    printf("\n");
}

void printmap(J2735MapData_t *mapdata)
{
    int i, j;

    printf("Intersection id:        0x%X\n", mapdata->intersection_id);
    printf("Msg count:              %u\n", mapdata->msg_count);
    printf("Ref point:              %f,%f\n",
                            mapdata->reference_point.latitude,
                            mapdata->reference_point.longitude);
    for(i=0; i < mapdata->num_approaches; i++){

        printf("Approach number:        %lu\n",
                                mapdata->approaches[i].approach_number);

        for(j=0; j < mapdata->approaches[i].num_ingress_lanes; j++)
            printlane("Ingress Lane", &mapdata->approaches[i].ingress_lanes[j]);
        for(j=0; j < mapdata->approaches[i].num_egress_lanes; j++)
            printlane("Egress Lane", &mapdata->approaches[i].egress_lanes[j]);
        for(j=0; j < mapdata->approaches[i].num_crosswalks; j++)
            printlane("Crosswalk", &mapdata->approaches[i].crosswalks[j]);
    }
}

void write_inlane_outlane_phase_mapping_to_file(char *filename)
{
	int i,j;
	char log[128];
	fstream fs;
	fs.open(filename, ios::out);
	
	if (!fs || !fs.good())
	{
		cout << "could not open file!\n";
	}
	
	sprintf(log,"IntersectionID\t%d\n",NewMap.intersection.ID);
	fs<<log;
	sprintf(log,"No_Approach\t%d\n",NewMap.intersection.Appro_No);
	fs<<log;
	
	int count=0;
	for(i=0;i<8;i++)
	{
		if(phase_mapping[i]!=0)
		count++;
	}	
	sprintf(log,"No_Phase\t%d\n",count);
	fs<<log;
	int No_Ingress=NewMap.intersection.Appro_No/2;
	sprintf(log,"No_Ingress\t%d\n",No_Ingress);
	fs<<log;
	for(i=0;i<No_Ingress;i++)
	{
		sprintf(log,"Approach\t%d\n",i*2+1);
		fs<<log;
		int lane_no=NewMap.intersection.Approaches[i*2].Lane_No;
		for(j=0;j<NewMap.intersection.Approaches[i*2].Lane_No;j++)
		{
			if(NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[3]==1 && NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[1]==0 && NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[2]==0) //only right turn lane
			lane_no--;
		}
		sprintf(log,"No_Lane\t%d\n",lane_no);
		fs<<log;
		
		
		for(j=0;j<NewMap.intersection.Approaches[i*2].Lane_No;j++)
		{
			int phase;
			if(NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[3]==1) //right turn lane
				phase=0;   //write turn lane doesn't require any phase
			if(NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[1]==1) //through lane
				phase=phase_mapping[i*2];   //put through lane after right turn lane, to make sure a through-right turn lane can be written successfully
			if(NewMap.intersection.Approaches[i*2].Lanes[j].Attributes[2]==1) //left turn lane
				phase=phase_mapping[i*2+1];		
			if(phase!=0)
			{	
				sprintf(log,"%s %d.%d %d\n",NewMap.intersection.Approaches[i*2].Lanes[j].Lane_Name.c_str(),NewMap.intersection.Approaches[i*2].Lanes[j].Connections[0].ConnectedLaneName.Approach,
					NewMap.intersection.Approaches[i*2].Lanes[j].Connections[0].ConnectedLaneName.Lane,phase);
				fs<<log;
			}
		}
		sprintf(log,"end_Approach\n");
		fs<<log;
	}
	sprintf(log,"end_file");
	fs<<log;
	fs.close();
}

void Pack_ART(byte* ART_data, char * filename, int &size)
{
	int i;
	byte* pByte;
	int offset=0;
	
	unsigned short   tempUShort;
    long    tempLong;
    double  temp;
	
	char tempString[20];
	
	fstream fs;
    fs.open(filename);
    char tmp[128];
	string temp_string;
    getline(fs,temp_string);  
    strcpy(tmp,temp_string.c_str());
	int NumReq=0;
	sscanf(tmp,"%*s %d",&NumReq);
	cout<<"NumReq is: "<<NumReq<<endl;
	
	if(NumReq==-1)   //-1 means at the beginning of the program, make it 0 here.
	NumReq=0;
	
	//Do Intersection ID;
	tempLong = (long) NewMap.intersection.ID; 
	pByte = (byte* ) &tempLong;
	ART_data[offset+0] = (byte) *(pByte + 3); 
	ART_data[offset+1] = (byte) *(pByte + 2); 
	ART_data[offset+2] = (byte) *(pByte + 1); 
	ART_data[offset+3] = (byte) *(pByte + 0); 
	offset = offset + 4;
	
	//Pack Number of Requests
	tempUShort = (unsigned short)NumReq;
	pByte = (byte* ) &tempUShort;
	ART_data[offset+0] = (byte) *(pByte + 1); 
    ART_data[offset+1] = (byte) *(pByte + 0); 
	offset = offset + 2;
	//Start packing the active request table
	for(i=0;i<NumReq;i++)
	{
		getline(fs,temp_string);  //First line is comment
		strcpy(tmp,temp_string.c_str());
		long vehid=0;
		int veh_class=0;
		float ETA=0;
		int phase=0;
		float Tq=0;
		long abs_time=0;
		//int split_phase=0;
		int inlane=0;
		int outlane=0;
		int shour=0;
		int smin=0;
		int ssec=0;
		int ehour=0;
		int emin=0;
		int esec=0;
		int veh_state=0;    //1: approaching 2: leaving 3: Inqueue
		int req_sequence=0;
		sscanf(tmp,"%*s %ld %d %f %d %f %ld %d %d %d %d %d %d %d %d %d %d",&vehid,&veh_class,&ETA,&phase,&Tq,&abs_time,&inlane,&outlane,&shour,&smin,&ssec,&ehour,&emin,&esec,&veh_state,&req_sequence);
		//Do Veh ID;
		tempLong = (long) vehid; 
		pByte = (byte* ) &tempLong;
		ART_data[offset+0] = (byte) *(pByte + 3); 
		ART_data[offset+1] = (byte) *(pByte + 2); 
		ART_data[offset+2] = (byte) *(pByte + 1); 
		ART_data[offset+3] = (byte) *(pByte + 0); 
		offset = offset + 4;
		//Do veh_class
		tempUShort = (unsigned short)veh_class;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do ETA
		tempUShort = (unsigned short)(ETA*FLOAT2INT);
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//cout<<"ETA: "<<tempUShort<<endl;
		//Do phase
		tempUShort = (unsigned short)phase;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do Tq
		tempUShort = (unsigned short)(Tq*FLOAT2INT);
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do abs_time
		tempLong = (long) abs_time; 
		pByte = (byte* ) &tempLong;
		ART_data[offset+0] = (byte) *(pByte + 3); 
		ART_data[offset+1] = (byte) *(pByte + 2); 
		ART_data[offset+2] = (byte) *(pByte + 1); 
		ART_data[offset+3] = (byte) *(pByte + 0); 
		offset = offset + 4;
		//Do split phase
		//tempUShort = (unsigned short)split_phase;
		//pByte = (byte* ) &tempUShort;
		//ART_data[offset+0] = (byte) *(pByte + 1); 
		//ART_data[offset+1] = (byte) *(pByte + 0); 
		//offset = offset + 2;
		//Do inlane
		tempUShort = (unsigned short)inlane;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do outlane
		tempUShort = (unsigned short)outlane;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do shour
		tempUShort = (unsigned short)shour;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do smin
		tempUShort = (unsigned short)smin;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do ssec
		tempUShort = (unsigned short)ssec;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do ehour
		tempUShort = (unsigned short)ehour;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do emin
		tempUShort = (unsigned short)emin;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do esec
		tempUShort = (unsigned short)esec;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do veh_state
		//if (strcmp(tempString,"Approaching")==0)
		//veh_state=1;
		//if (strcmp(tempString,"Leaving")==0)
		//veh_state=2;
		//if (strcmp(tempString,"InQueue")==0)
		//veh_state=3;
		
		tempUShort = (unsigned short)veh_state;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		//Do request sequence
		tempUShort = (unsigned short)req_sequence;
		pByte = (byte* ) &tempUShort;
		ART_data[offset+0] = (byte) *(pByte + 1); 
		ART_data[offset+1] = (byte) *(pByte + 0); 
		offset = offset + 2;
		
				
	}
    size=offset;
}

int print_Ped_Status(J2735SPATMsg_t *spat,char *filename)
{
	fstream fs;
	fs.open(filename, ios::out);
	
	if (!fs || !fs.good())
	{
		cout << "could not open file!\n";
		exit(0);
	}
	
	fs<<spat->intersection_id<<" Ped_Status\n";
	
	int i;
	J2735MovementState_t  *mm_st;
	mm_st = spat->movement_states;
	//printf("The Ped status are:\n");
	for(i=0;i<NUM_MM_STS;i++)
	{
		if(mm_st[i].ped_state==J2735SPAT_PED_UNAVAILABLE)
		{
			//printf("N/A ");
			fs<<"0 ";
		}	
		if(mm_st[i].ped_state==J2735SPAT_PED_WALK)
		{
			//printf("G ");
			fs<<"1 ";
		}
		if(mm_st[i].ped_state==J2735SPAT_PED_CAUTION)
		{
			//printf("Y ");
			fs<<"2 ";
		}
		if(mm_st[i].ped_state==J2735SPAT_PED_STOP)
		{
			//printf("R ");
			fs<<"3 ";
		}
	}
	//printf("\n");
	fs<<"\n";
		
	//printf("The Ped Remaining time are:\n");
	for(i=0;i<NUM_MM_STS;i++)
	{
		printf("%d ",mm_st[i].next_time_remaining);
		fs<<mm_st[i].next_time_remaining<<" ";
	}
	printf("\n");
	fs<<"\n";
	
	fs.close();	
	return 0;
}

int print_Signal_Status(J2735SPATMsg_t *spat, char *filename)
{
		//char temporary_log[256];
	
		fstream fs;
		fs.open(filename, ios::out | ios::trunc);
		
		if (!fs || !fs.good())
		{
			cout << "could not open file!\n";
			exit(0);
		}
	
	//fs<<spat->intersection_id<<" Sig_status ";
	
		
	//~ if(field_flag==1)
	//~ {
		//~ //savari_gps_read (&gps, gps_handle);
		//~ sprintf(tmp_log,"At time: %.2lf, Signal Status is: ",GetSeconds());
	//~ }
	//~ else
	//~ {
		//~ sprintf(tmp_log,"At time: %.2lf Signal Status is: ",GetSeconds());
	//~ }
	
	
	int i;
	J2735MovementState_t  *mm_st;
	mm_st = spat->movement_states;
	for(i=0;i<NUM_MM_STS;i++)
	{
		if(mm_st[i].signal_state.ball==J2735SPAT_BALL_DARK)
		{
			//printf("N/A ");
			//strcat(temporary_log,"0 ");
			fs<<"0 ";

		}	
		if(mm_st[i].signal_state.ball==J2735SPAT_BALL_GREEN)
		{
			//printf("G ");
			//strcat(temporary_log,"3 ");
			fs<<"3 ";
		}
		if(mm_st[i].signal_state.ball==J2735SPAT_BALL_YELLOW)
		{
			//printf("Y ");
			//strcat(temporary_log,"4 ");
			fs<<"4 ";
		}
		if(mm_st[i].signal_state.ball==J2735SPAT_BALL_RED)
		{
			//printf("R ");
			//strcat(temporary_log,"1 ");
			fs<<"1 ";
		}
	}
	fs.close();
	//strcat(tmp_log,"\n");
	//outputlog(tmp_log);
}

//~ int gps_init () {
    //~ int is_async = 0;
    //~ int fd;
//~ 
    //~ gps_handle = savari_gps_open(&fd, is_async);
    //~ if (gps_handle == 0) {
        //~ printf("sorry no gpsd running\n");
        //~ return -1;
    //~ }
//~ 
    //~ return 0;
//}
