//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************



/* MMITSS_RSU_SMARTCROSS.cpp
*  Created by :Yiheng Feng
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


#define LOCAL_HOST_ADDR "127.0.0.1"
#define PORT_SPAT 15020
#define PORT_MAP 7890

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

char WIRELESS_ROUTE_ADDR[64]="10.254.56.7";




//MAP parameters
char Ped_Status_File_Name[64]  = "/nojournal/bin/Ped_status.txt";



int msleep(unsigned long milisec);
void PrintPhases(PhaseStatus currentPhase);
void xTimeStamp( char * pc_TimeStamp_ );


int gettimestamp(uint64_t *seconds, uint64_t *microsecs);


//Read ART from file requested_combined.txt and form a octet stream to be sent
void Pack_ART(byte* ART_data, char * filename, int &size);


SignalController Daisy_Mnt;

//define log file name
char logfilename[256]   = "/nojournal/bin/log/MMITSS_MRP_MAP_SPAT_BROADCAST_";
char IPInfo[64]			= "/nojournal/bin/ntcipIP.txt";

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

void flipEndian (char *buf, int size) {
       int start, end;
       for (start = 0, end = size - 1; start < end; start++, end--) {
               buf[start] = buf[start] ^ buf[end];
               buf[end] = buf[start] ^ buf[end];
               buf[start] = buf[start] ^ buf[end];
       }
}

int main ( int argc, char* argv[] )
    {
		
	//Struct for UDP socket timeout: 0.5s
	struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000;	
		
		
	int i,j,k;
		int ret;

    unsigned int type = 0;
    
    if (argc>1)
	{
		sscanf(argv[1],"%s",WIRELESS_ROUTE_ADDR);   // Obtain the broadcasting address from user. If Communication is through WME the address should be 127.0.0.1 otherwise the address is 192.168.1.255
	}

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

    //~ for(int i=4;i>0;i--) //warm up...
        //~ {
        //~ Daisy_Mnt.PhaseRead();//IntersectionPhaseRead();
        //~ Daisy_Mnt.UpdatePhase();//Phases.UpdatePhase(phase_read);
        //~ Daisy_Mnt.Phases.Display();//Phases.Display();
        //~ msleep(1000);
        //~ }
	
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
	
	
	//Setup time out
	if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
      perror("Error");
	}
/*	
	if((setsockopt(sockfd,SOL_SOCKET,SO_BROADCAST,
		&broadcast,sizeof broadcast)) == -1)
	{
		perror("setsockopt - SO_SOCKET ");
		exit(1);
	}
	*/
	

	sendaddr.sin_family = AF_INET;
	sendaddr.sin_port = htons(5678);  //*** IMPORTANT: the vissim,signal control and performance observer should also have this port. ***//
	sendaddr.sin_addr.s_addr = INADDR_ANY;//inet_addr(LOCAL_HOST_ADDR);//inet_addr(OBU_ADDR);//INADDR_ANY;

	memset(sendaddr.sin_zero,'\0',sizeof sendaddr.sin_zero);

	if(bind(sockfd, (struct sockaddr*) &sendaddr, sizeof sendaddr) == -1)
	{
		perror("bind");        exit(1);
	}

	int addr_length = sizeof ( recvaddr );
	int recv_data_len;
	//-----------------------End of Network Connection------------------//
	
	recvaddr.sin_family = AF_INET;
	recvaddr.sin_port = htons(7890);
	recvaddr.sin_addr.s_addr = inet_addr(WIRELESS_ROUTE_ADDR) ; //INADDR_BROADCAST;
	memset(recvaddr.sin_zero,'\0',sizeof recvaddr.sin_zero);

////////////////////////End of reading map and lane phase mapping

int ped_phase_status[8];
int ped_remaining_time[8];
char recv_buf[8];

char temp_log[128];


int count=-1;
int phase;
    while ( true )
    {		
		recv_data_len = recvfrom(sockfd, recv_buf, sizeof(recv_buf), 0,
                        (struct sockaddr *)&sendaddr, (socklen_t *)&addr_length);
        
        if(recv_data_len>=0)
        {                			
			phase= * ((int *) recv_buf);	
			int value=pow(2,phase-1);				
			Daisy_Mnt.PhaseControl(PHASE_PED_CALL,value,'Y');  //call ped phase 2!!!!!!!!!!!!!!!!!!!
			cout<<"Call Ped phase: "<<value<<endl;
			count=3;
		}
		else
		{
			if(count>=0)
			{
				Daisy_Mnt.PhaseControl(PHASE_PED_CALL,0,'Y');  //clear ped call
				cout<<"Clear ped call!"<<endl;
				count--;
			}
			phase=0;
		}
		
		
		fstream fs;
		fs.open(Ped_Status_File_Name);
		char tmp[128];
		string temp_string;
		getline(fs,temp_string); //first line is just the intersection ID
		getline(fs,temp_string); //Second line is the ped phase status information
		strcpy(tmp,temp_string.c_str());
		sscanf(tmp,"%d %d %d %d %d %d %d %d",&ped_phase_status[0],&ped_phase_status[1],&ped_phase_status[2],&ped_phase_status[3],&ped_phase_status[4],&ped_phase_status[5],&ped_phase_status[6],&ped_phase_status[7]);
		getline(fs,temp_string); //Second line is the ped phase status information
		strcpy(tmp,temp_string.c_str());  //Third line is the ped remaining time
		sscanf(tmp,"%d %d %d %d %d %d %d %d",&ped_remaining_time[0],&ped_remaining_time[1],&ped_remaining_time[2],&ped_remaining_time[3],&ped_remaining_time[4],&ped_remaining_time[5],&ped_remaining_time[6],&ped_remaining_time[7]);
		
		
		
		double cur_time=GetSeconds();
		sprintf(temp_log,"Current time is: %lf",cur_time);
		cout<<temp_log;
		
		
		int offset=0;
		
		unsigned char buf[104];
		
		flipEndian ((char *)(&cur_time), sizeof(double));
		
		memcpy(buf,&cur_time,sizeof(cur_time));
		offset+=sizeof(cur_time);
		
		for(i=0;i<8;i++)
		{
			char temp[4];
			//phase No.
			int phase=i+1;
			flipEndian ((char *)(&phase), sizeof(int));
			memcpy(buf+offset,&phase,sizeof(phase));
			offset+=sizeof(phase);
			
			//phase status
			int status=ped_phase_status[i];
			flipEndian ((char *)(&status), sizeof(int));
			memcpy(buf+offset,&status,sizeof(status));
			offset+=sizeof(status);
			
			
			int remaining=ped_remaining_time[i];
			flipEndian ((char *)(&remaining), sizeof(int));
			memcpy(buf+offset,&remaining,sizeof(remaining));
			offset+=sizeof(remaining);
		}
		
		//cout<<buf<<endl;	
		
		for(int i=0;i<104;i++)
		{
			if (!(i % 4)) printf (" ");
			if (!(i % 8)) printf (" ");
			if (!(i % 16)) printf ("\n");
			printf("%02x ",buf[i]);
		}	
		printf("\n");
		
		//Send the buf
		sendto(sockfd,buf,sizeof(buf), 0,(struct sockaddr *)&recvaddr, addr_length);
		
		cout<<"Send Ped Signal Status"<<endl;
		

		msleep(1000);
    }
    

   
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



int gettimestamp(uint64_t *seconds, uint64_t *microsecs)
{
    struct timeval tv;

    gettimeofday(&tv, 0);

    *seconds = tv.tv_sec;
    *microsecs = tv.tv_usec;

    return 0;
}
