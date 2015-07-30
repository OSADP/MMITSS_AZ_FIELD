/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */   



/* MMITSS_PED_MAP_BROADCAST.cpp
*  Created by :Sara Khosravi
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.

*/

#include <getopt.h>
#include <unistd.h>

#include "GetInfo.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <getopt.h>
#include <unistd.h>
#include <vector>
#include <libgps.h>


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

#ifndef DEG2ASNunits
#define DEG2ASNunits  (1000000.0)  // used for ASN 1/10 MICRO deg to unit converts
#endif


#define CONTROLLER_IP "10.254.56.23"
char INTip[64];
char INTport[16];
char IPInfo[64]="/nojournal/bin/ntcipIP.txt"; 
int LaneNode_Phase_Mapping[8][8][20];
string nmap_name;
MAP NewMap;
char tmp_log[512];
byte tmp_traj_data[1000];
char BROADCAST_ADDR_ATH1[64]="192.168.101.255";

int msleep(unsigned long milisec);
void PackTrajData(byte* tmp_traj_data,int &size);
void get_map_name();
int outputlog(char *output);
void xTimeStamp( char * pc_TimeStamp_ ); 

char logfilename[256] = "/nojournal/bin/log/MMITSS_Ped_Map_Broadcaster_";
char MAP_File_Name[64]  = "/nojournal/bin/Ped_MAP.txt";
//int field_flag=0;
int main ( int argc, char* argv[] )
{ 

	if (argc>1)
	{
		sscanf(argv[1],"%s",BROADCAST_ADDR_ATH1);   // Obtain the broadcasting address from user. If Communication is through WME the address should be 127.0.0.1 
		//sscanf(argv[3],"%d",&field_flag);
	}

	//------log file name with Time stamp---------------------------
	char timestamp[128];
	//char tmp_log[64];
	xTimeStamp(timestamp);
	strcat(logfilename,timestamp);strcat(logfilename,".log");

	std::fstream fs;
	fs.open(logfilename, fstream::out | fstream::trunc);


	//------------init: Begin of Network connection------------------------------------
	int sockfd;
	int broadcast=1;

	struct sockaddr_in destaddr;
	int numbytes, addr_len;

	if((sockfd = socket(AF_INET,SOCK_DGRAM,0)) == -1)
	{
		perror("sockfd");
		exit(1);
	}

	destaddr.sin_family = AF_INET;
	destaddr.sin_port = htons(PORT_MAP);
	destaddr.sin_addr.s_addr = inet_addr(BROADCAST_ADDR_ATH1) ; //INADDR_BROADCAST;
	memset(destaddr.sin_zero,'\0',sizeof destaddr.sin_zero);
	int addr_length = sizeof ( destaddr );
	int recv_data_len;
	//-----------------------End of Network Connection------------------//

	NewMap.ID=1;
	NewMap.Version=1;
	// Parse the MAP file
	get_map_name();
	char mapname[128];
	sprintf(mapname,"%s",nmap_name.c_str());
	printf("%s",mapname);
	NewMap.ParseIntersection(mapname);

	int size;
	int i,j,k;
	int ret;

	unsigned int type = 0;

	PackTrajData(tmp_traj_data, size);
	cout << "TrajDataSize : " << size << endl;
	while (true){
		int temp_size = sendto(sockfd,tmp_traj_data, size,0,(struct sockaddr *)&destaddr, addr_length);
		if (size != temp_size)
			cout << "ERROR in broadcasting map: Sent " << temp_size << " bytes" << endl;
		else
			cout<<"Broadcast MAP Message!"<<endl;
		msleep(1000);
	}
}

//Pack the trajectory data to a octet stream

//Pack the data from trackedveh to a octet string

void PackTrajData(byte* tmp_traj_data,int &size)
{
	int i,k;
	int offset=0;
	byte*   pByte;      // pointer used (by cast)to get at each byte 
	// of the shorts, longs, and blobs
	byte    tempByte;   // values to hold data once converted to final format
	unsigned short   tempUShort;
	long    tempLong;
	double  temp;

	//for each approach

	for (i=0;i<NewMap.intersection.Approaches.size();i++)
	{
		for(k=0;k<NewMap.intersection.Approaches[i].Nodes.size();k++)
		{
			//approach no
			tempUShort = (unsigned short) NewMap.intersection.Approaches[i].Nodes[k].index.Approach;
			pByte = (byte* ) &tempUShort;
			tmp_traj_data[offset+0] = (byte) *(pByte + 1); 
			tmp_traj_data[offset+1] = (byte) *(pByte + 0); 
			offset = offset + 2;

			//latitude
			tempLong = (long) (NewMap.intersection.Approaches[i].Nodes[k].Latitude*DEG2ASNunits); 
			pByte = (byte* ) &tempLong;
			tmp_traj_data[offset+0] = (byte) *(pByte + 3); 
			tmp_traj_data[offset+1] = (byte) *(pByte + 2); 
			tmp_traj_data[offset+2] = (byte) *(pByte + 1); 
			tmp_traj_data[offset+3] = (byte) *(pByte + 0); 
			offset = offset + 4;
			//longitude 
			tempLong = (long) (NewMap.intersection.Approaches[i].Nodes[k].Longitude*DEG2ASNunits);
			pByte = (byte* ) &tempLong;
			tmp_traj_data[offset+0] = (byte) *(pByte + 3); 
			tmp_traj_data[offset+1] = (byte) *(pByte + 2); 
			tmp_traj_data[offset+2] = (byte) *(pByte + 1); 
			tmp_traj_data[offset+3] = (byte) *(pByte + 0); 
			offset = offset + 4;
		}
	}
	size=offset;
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

int outputlog(char *output)
{
	FILE * stream = fopen( logfilename, "r" );

	if (stream==NULL)
	{
		perror ("Error opening file");
	}

	fseek( stream, 0L, SEEK_END );
	long endPos = ftell( stream );
	fclose( stream );

	fstream fs;
	if (endPos <10000000)
		fs.open(logfilename, ios::out | ios::app);
	else
		fs.open(logfilename, ios::out | ios::trunc);

	//fstream fs;
	//fs.open("/nojournal/bin/OBU_logfile.txt", ios::out | ios::app);
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
