/* $Id: cgpxlogger.c 4685 2008-02-17 02:57:48Z ckuethe $ */
/*
* Copyright (c) 2005,2006 Chris Kuethe <chris.kuethe@gmail.com>
*
* Permission to use, copy, modify, and distribute this software for any
* purpose with or without fee is hereby granted, provided that the above
* copyright notice and this permission notice appear in all copies.
*
* THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
* WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
* ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
* ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
* OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/




//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  cgpxlogger.cpp 
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
   

#include <sys/types.h>
#include <sys/cdefs.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <ctype.h>
#include <err.h>  
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <string>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ReqEntry.h"
#include "LinkedList.h"
//#include "gpsd_config.h"
//#include "gpsd.h"
#include "libgps.h"
//#include "OutputLog.h"

using namespace std;
#define BS 512
#define NUM 8
#define EV  1
#define TRANSIT 2
#define TRUCK 3

#define ACTIVE 1
#define NOT_ACTIVE -1
#define LEAVE 0

#define RED 1
#define GREEN 3
#define YELLOW 4



int tracking = 0;
time_t theTime = time(NULL);
int Hour;
int Minute;
int Second;
char *pollstr = "SPAMDQTV\n";
char *host = "127.0.0.1";
char *port = "2947";
unsigned int sl = 1;//5; 1 second interval
struct {
    double latitude;
    double longitude;
    float altitude;
    float speed;
    float course;
    float hdop;
    short svs;
    char status;
    char mode;
    char time[32];
} gps_ctx;

char logfilename[256] = "/nojournal/bin/log/webserver_";
char requestfilename[128] = "/nojournal/bin/requests_combined.txt";
char active_rndf_file[128]="/nojournal/bin/ActiveMAP.txt";
string Active_RNDF, Leave_RNDF, Active_Name, Leave_Name;
int TYPE;   // the return value from ActiveRNDF.txt.
char temp_log[256];
bool isconn;
char *progname;
int Have_Request=0;
int iReqSequence=0;
unsigned int track_count=0xffffffff;
unsigned int print_count=0;
int  is_html=0;
int MANUAL=1;  // when -m is off, means to check the "connection.txt" generated from obu_listener
               // when -m is on, MANUAL =0 means always connected.

void usage(void);
bool isconnected();
void bye(int);
void html_header(void);
void html_footer(void);
void header(void);
void footer(void);
int  FindActiveRNDF(char *activeRNDFfile,string& active_name, string& leave_name, int &iReqSeq);
void signal_status();
void request_table();
void priority_status();
int FindListHighestPriority(LinkedList<ReqEntry> Req_List);
struct gps_data_t *gpsdata;
void process(struct gps_data_t *, char *, size_t, int);
void html_write_record(struct gps_data_t *gpsdata);
void html_track_start(void);
void html_track_end(void);
void write_record(struct gps_data_t *gpsdata);
void track_start(void);
void track_end(void);
void gps_update();
int  FindCombinedPhase(int phase);
void FindCombinedReq(LinkedList<ReqEntry> &ReqList, ReqEntry &TestEntry);
void FindCombinedReqList(LinkedList<ReqEntry> &ReqList, LinkedList<ReqEntry> &ReqList_Com);
void PrintReqList(LinkedList<ReqEntry> ReqList);
void PrintReqListCombined(LinkedList<ReqEntry> ReqList);
int  ReqListFromFile(char *filename,LinkedList<ReqEntry>& Req_List);


int main(int argc, char **argv)
{
    int ch;

    int casoc = 0;


    progname = argv[0];
    while ((ch = getopt(argc, argv, "hVmwn:i:j:s:p:")) != -1)
    {
        switch (ch)
        {
        case 'i':
            sl = (unsigned int)atoi(optarg);
            if (sl < 1)
                sl = 1;
            if (sl >= 3600)
                fprintf(stderr,
                "WARNING: polling interval is an hour or more!\n");
            break;
        case 'j':
            casoc = (unsigned int)atoi(optarg);
            casoc = casoc ? 1 : 0;
            break;
        case 's':  
            host = optarg;
            break;
        case 'p':
            port = optarg;
            break;
        case 'w':
            is_html=1;
            break;
        case 'm':
			MANUAL=0;
            break;
        case 'n':
            track_count = (unsigned int)atoi(optarg);
            break;

        case 'V':
            (void)fprintf(stderr, "SVN ID: $Id: cgpxlogger.c 4685 2008-02-17 02:57:48Z ckuethe $ \n");
            exit(0);
        default:
            usage();
            /* NOTREACHED */
        }
    }

    MANUAL=1;


	TYPE=FindActiveRNDF(active_rndf_file,Active_RNDF,Leave_RNDF, iReqSequence); // Return the string of value "xxx.rndf"

	string s2;

	if(TYPE==3)
	{
		Active_Name.assign(Active_RNDF);
		Leave_Name.assign(Leave_RNDF);
	}
	else if(TYPE==2)
	{
		Active_Name.assign(Active_RNDF);
	}
	else if(TYPE==1)
	{
		Active_Name.assign(Leave_RNDF);
	}
	else //TYPE==0
	{
		sprintf(temp_log,"No Active Map, At time: {%ld}\t",time(NULL));
	}
    printf("<H2> <center> <font size=\"25\" color=\"white\" style=\"font-family:Time New Roman\" > <b> Multi Modal Intelligent Traffic Signal System </font> </center></H2>");
    printf("<H2> <center> <font size=\"25\" color=\"white\" style=\"font-family:Time New Roman\" > <b> RSE Priority Page </font> </center></H2>");
 	isconn=isconnected();
	header();
    for(;;)
    {
        footer();
        exit(0);
    }
    

}	// *** main() ***//




void usage()
{
    fprintf(stderr,
        "Usage: %s [-h] [-w ] [-n count] [-s server] [-p port] [-i interval] [-j casoc]\n\t-w => html output\n\t-n count => number of tracks to print\n",
        progname);
    fprintf(stderr,
        "\tdefaults to '%s -s 127.0.0.1 -p 2947 -i 5 -j 0 -n 0xffffffff'\n",
        progname);
    exit(1);
}


bool isconnected()
{
	fprintf (stderr, "in function : %s\n", __func__);
    int tt0;
	FILE * fp=fopen("/nojournal/bin/connection.txt","r");
    fscanf(fp,"%d",&tt0);
    fclose(fp);
	return true;
}


void html_header()
{
	fprintf (stderr, "in function : %s\n", __func__);
    char temp[64];
    char temp2[64];
 
    //bool isconn;
    FILE * fp1=fopen("/nojournal/bin/vehicleid.txt","r");

    fscanf(fp1,"%s", temp);
    fclose(fp1);

    //isconn = isconnected();
     struct tm *aTime = localtime(&theTime);
    Hour = aTime->tm_hour;
    Minute = aTime->tm_min; 
    Second = aTime->tm_sec; 
    
    printf ( "   <font color=\"yellow\" size=\"5\" style=\"font-family:Time New Roman\" >    Current Time is:  &nbsp;  <font color=\"white\" size=\"5\" style=\"font-family:Time New Roman\"  >  %d : %d : %d   <br> \n", Hour, Minute, Second );

    //~ if (isconn)
    //~ {
		//~ if(TYPE>=1)
        //~ sprintf(temp2,"Connected to RSE !");
		//~ else
		//~ {
			//~ sprintf(temp2,"Connected to a RSE! But not on any MAP");
		//~ }
    //~ }
    //~ else
    //~ {
        //~ sprintf(temp2,"SEARCHING FOR ANY RSE......");
    //~ }
    printf("<html>\n\t<head>\n\t\t<title>");

    printf("Arizona MMITSS webpage: %s </title>\n\t        <font color=\"yellow \"  size=\"5\" style=\"font-family:Time New Roman\" > &nbsp;  <font color=\"white \"  size=\"4\" style=\"font-family:Time New Roman\" >  %s  \n\t <link href=\"styles.css\" rel=\"stylesheet\" type=\"text/css\">\n </head>\n", temp,temp2);
	printf("\t<body bgcolor=\"#192F3F\" background=\"Detroit.jpg\" text=\"#FFFFFF\">\n");
 }

void header() 
{
	fprintf (stderr, "in function : %s\n", __func__);
    if (is_html)
    {
        html_header();
        return;
    }

    printf("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
    printf("<gpx version=\"1.1\" creator=\"GPX GPSD client\"\n");
    printf("        xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n");
    printf("        xmlns=\"http://www.topografix.com/GPX/1.1\"\n");
    printf("        xsi:schemaLocation=\"http://www.topografix.com/GPS/1/1\n");
    printf("        http://www.topografix.com/GPX/1/1/gpx.xsd\">\n");
    printf("  <metadata>\n");
    printf("    <name>GPX GPSD client</name>\n");
    printf("    <author>Chris Kuethe (chris.kuethe@gmail.com)</author>\n");
    printf("    <copyright>2-clause BSD License</copyright>\n");
    printf("  </metadata>\n");
    printf("\n");
    printf("\n");
}

void html_footer()
{
	fprintf (stderr, "in function : %s\n", __func__);
    signal_status();
    request_table();
    priority_status();
	printf("<br> <br><br> <br><br> <br><br> <br><br> <br>");		
    printf("\t\t<center><img src = \"pics/LogoBanner.png\" width=\"900\" ></center> \n"); //Import the logos ribbon
    printf("\t</body>\n</html>\n");
}


void footer()
{
	fprintf (stderr, "in function : %s\n", __func__);
    if (is_html)
    {
        html_footer();
        return;
    }
    track_end();
    printf("</gpx>\n");
}

void html_track_end()
{
	fprintf (stderr, "in function : %s\n", __func__);
    if (tracking == 0)
        return;
    printf("\t\t</table>\n");
    //printf("\t\t<IMG SRC=\"DemoSite.jpg\" ALT=\"DemoSite\"> \n");
    tracking = 0;
}

void track_end()
{
    if (is_html)
    {
        html_track_end();
        return;
    }
    if (tracking == 0)
        return;
    printf("    </trkseg>\n  </trk>\n<!-- track end -->\n");
    tracking = 0;
}

void priority_status()
{
	fprintf (stderr, "in function : %s\n", __func__);
    printf("</tr></table>");
	if (isconn)
	{
        switch(TYPE)
        {
		case 2:  // only approaching
			printf("<table bgcolor=\"#66FF00\" border=\"1\" align=\"center\" width=\"800\"><tr>");  // Color Green
			printf("<td align=\"center\"> <font size=\"20\" color=\"black\"> On Map: %s </font>  </td>",Active_Name.c_str());
			break;
        case 3:  // Both approaching and leaving
            printf("<table bgcolor=\"#FF8C00\" border=\"1\" align=\"center\" width=\"800\"><tr>");  // Color Green
            printf("<td align=\"center\"> <font size=\"20\" color=\"black\"> On Map: %s <BR>Leaving %s</font>   </td>",Active_Name.c_str(),Leave_Name.c_str());
            break;
        case 0: // Non-active map
            printf("<table bgcolor=\"#ABFF73\" border=\"1\" align=\"center\" width=\"800\"><tr>");  // Color is Red
            printf("<td align=\"center\"> <font size=\"20\" color=\"black\"> NO Active Map </font></td>");
            break;
        case 1: // only leaving
            printf("<table bgcolor=\"#FFFF00\" border=\"1\" align=\"center\" width=\"800\"><tr>");  // Color is Yellow
            printf("<td align=\"center\"> <font size=\"20\" color=\"black\"> Leaving Map: %s </font></td> ",Active_Name.c_str());
            printf("</tr></table>");
            break;
        default:
            printf("<table bgcolor=\"#000000\" border=\"1\" align=\"center\" width=\"800\"><tr>");  // Color is BLACK
            printf("<td align=\"center\"> <font size=\"20\" color=\"black\"> NO Active Map </font></td>");
        }
    }
    else
    {
        printf("<table bgcolor=\"#000000\" border=\"1\" align=\"center\" width=\"800\"><tr>");  // Color is BLACK
        printf("<td align=\"center\"> <font size=\"20\" color=\"black\"> ~~~NO Active Map~~~ </font></td>");
    }

    printf("</tr></table>");

    if ( Have_Request>0 && isconn )
    {
        printf("<table bgcolor=\"#D7ACAC\" border=\"1\" align=\"center\" width=\"800\"><tr>");
        printf("<td align=\"center\"><font size=\"20\" color=\"black\">  Priority Request is ACTIVE  </font> </td> " );
        printf("</tr></table>");
    }
    else
    {
        printf("<table bgcolor=\"#202020\" border=\"1\" align=\"center\" width=\"800\"><tr>");
        printf("<td align=\"center\"><font size=\"20\" color=\"white\"> Priority Request is NOT ACTIVE </font></td>");
        printf("</tr></table>");
    }
}


int  FindActiveRNDF(char *activeRNDFfile,string& active_name, string& leave_name, int &iReqSequence)
{
	fprintf (stderr, "in function : %s\n", __func__);
	//Return value will be:
	//3: both have active and leave maps
	//2: only have active map
	//1: only have leave map
	//0: no active map
	fstream fss;
	fss.open(activeRNDFfile,fstream::in);
	if (!fss)
	{
		cout<<"***********Error opening the plan file in order to print to a log file!\n";
	}
	char tmp_log[128];

	int ReadInNo;

	string lineread;
	int irequestSequence;
	int activeFileNo=0;  // The number of active maps:
	active_name.assign("ACTIVE.rndf"); // default name if no active map
	leave_name.assign("LEAVE.rndf");   // default name if not leave map
	while(!fss.eof())
	{
		getline(fss,lineread);
		if(lineread.size()!=0)
		{
			sscanf(lineread.c_str(),"%d %s %d",&ReadInNo,tmp_log, &irequestSequence);
			if(ReadInNo==ACTIVE)
			{
				active_name.assign(tmp_log);
				activeFileNo+=2;
			}
			else if(ReadInNo==LEAVE)
			{
				leave_name.assign(tmp_log);
				activeFileNo+=1;
			}
		}
	}
	iReqSequence=irequestSequence;
	fss.close();
	return activeFileNo;
}

void signal_status()
{
	fprintf (stderr, "in function : %s\n", __func__);
    int phase[8]={};
    int pedPhase[8]={};
    int iPed=0;
    int i;
    FILE * fp=fopen("/nojournal/bin/signal_status.txt","r");
    fscanf(fp,"%d%d%d%d%d%d%d%d",&phase[0],&phase[1],&phase[2],&phase[3],&phase[4],&phase[5],&phase[6],&phase[7]);
    fclose(fp);
    FILE * fp2=fopen("/nojournal/bin/Ped_status.txt","r");
   
    fstream fss;
	string lineread;
	fss.open("/nojournal/bin/Ped_status.txt",fstream::in);
	getline(fss,lineread);
	getline(fss,lineread);
	sscanf(lineread.c_str(),"%d%d%d%d%d%d%d%d",&pedPhase[0],&pedPhase[1],&pedPhase[2],&pedPhase[3],&pedPhase[4],&pedPhase[5],&pedPhase[6],&pedPhase[7]);
    fss.close();
    
    if (pedPhase[0]==1 || pedPhase[0]==2 || pedPhase[1]==1 || pedPhase[1]==2 || pedPhase[2]==1 || pedPhase[2]==2 || pedPhase[3] ==1 || pedPhase[3]==2 || pedPhase[4]==1 || pedPhase[4]==2 || pedPhase[5]==1 || pedPhase[5]==2 || pedPhase[6]==1 || pedPhase[6]==2 || pedPhase[7]==1 || pedPhase[7]==2)
		iPed=1;
	printf("<div style=\"position:relative; top:20px; left:10px;\">");
	printf("<H2> <font color=\"yellow\" size=\"6\" style=\"font-family:Time New Roman\" > Current Signal Status <H2>\n");
	printf("\t\t<table border=\"1\" bordercolor=\"#FFFFFF\" style=\"font-size: 22px;text-align:center\">\n");
	printf("\t\t\t<tr><th> Phase </th><th>1</th><th>2</th><th>3</th><th>4</th><th>5</th><th>6</th><th>7</th><th>8</th></tr>\n");
	printf("\t\t\t<tr>");
	printf("</div>");  
	printf("<td bgcolor=\"#FFFFFF\"><font color=\"#000000\">Signal Status </td>");
	for(i=0;i<8;i++)
	{
		if (phase[i]==GREEN)  // Green=3
		{
			printf("<td bgcolor=\"#FFFFFF\"><font color=\"#00FF00\">G</td>");
		}
		else if (phase[i]==YELLOW) // Yellow=4:#FFFF00
		{
			printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FFCC00\">Y</td>");
		}
		else if (phase[i]==RED)                // Red=0
		{
			printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\">R</td>");
		}
		else 
		{
			//printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\">R</td>");
			printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> -- </td>");
		}

	}
	printf("</tr>\n");
	printf("<td bgcolor=\"#FFFFFF\"><font color=\"#000000\"> Ped Status </td>");
	if ( (iPed>0) )
	{	
		for (int ii=0;ii<8;ii++)
		{
			if (pedPhase[ii]==1 || pedPhase[ii]==2 )
			{
				if (pedPhase[ii]==1)  // in this case the ped phase is in WALKING state
					printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> WK </td>");  
				else if (pedPhase[ii]==2)  // in this case the ped phase is in PED CLEARANCE state
					printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> PC </td>");									
			}
			else
				printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> -- </td>");
		}
	}
	else
	{
		printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> -- </td>");
		printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> -- </td>");
		printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> -- </td>");
		printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> -- </td>");
		printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> -- </td>");
		printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> -- </td>");
		printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> -- </td>");
		printf("<td bgcolor=\"#FFFFFF\"><font color=\"#FF0000\"> -- </td>");
	}
	printf("\t\t</table>\n");
 
   
    
}


void request_table()
{
	fprintf (stderr, "in function : %s\n", __func__);

    //bool isconn;
    LinkedList<ReqEntry> ReqList,ReqList_Com;






    int VehClass;		//	The Class of the vehicle
    float ETA;			//	The Estimation Time of Arrival
    int Phase,split_phase;			//	The phase of the Intersection


    //FILE * fp=fopen("/nojournal/bin/requests.txt","r");
    //FILE * fp=fopen("/nojournal/bin/requests_combined.txt","r");
    //fscanf(fp,"%s%d%d", sz_temp,&num_req,&temp1);

	int num_req=0;

	num_req=ReqListFromFile(requestfilename,ReqList);
	Have_Request=num_req;  // Used for displaying if the Priority-Controlled or not.

    FindCombinedReqList(ReqList,ReqList_Com);


    if(isconn)
    {
        if (Have_Request>0)
        {
			printf("<div style=\"position:relative; top:100px; left:0px;\">");

			printf("<H2> <font color=\"yellow\" size=\"6\" style=\"font-family:Time New Roman\" > Current Active Request Table <H2>\n");
            //printf("\t\t<table border=\"1\" bordercolor=\"#FFFFFF\" style=\"font-size: 22px;text-align:center\">\n");
            printf("\t\t<table border=\"1\" bordercolor=\"#FFFFFF\" >\n");
            printf("\t\t\t<tr><th> Request Entry </th><th>Active</th><th>Vehicle ID</th><th>Vehicle Type</th> <th> In Lane</th> <th> Out Lane</th> <th> Phase(s) </th> <th>Arrival Time</th><th>Time of Service Desired </th> <th> Time of Estimated Departure </th> <th> Vehicle State</th>  </tr>\n");
 
            ReqList_Com.Reset();  

            int HighestP=FindListHighestPriority(ReqList_Com);
            int iRequestEntry=1;

            while(!ReqList_Com.EndOfList())
            {
                ReqEntry R(ReqList_Com.Data());

                VehClass=R.VehClass;
                
             
                
             //   printf("<tr> <td class=\"tg-031e\">%d</td>  <td class=\"tg-mgbs\">--</td> <td class=\"tg-0ord\">--</td>
                printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\">%d</td>",iRequestEntry);

                if(HighestP==TRANSIT)  // NO EV case, ONLY have request of TRANSIT and TRUCK
                {
					if(VehClass==TRANSIT && R.iIsCanceled!=7)
					{
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><img src=\"./pics/green.png\"  width=\"22\" height=\"22\" ></td>");
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> %s </td>",R.VehID);
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#330033\">TRANSIT</td>");
					}
					if(VehClass==TRUCK && R.iIsCanceled!=7)
					{
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><img src=\"./pics/green.png\"  width=\"22\" height=\"22\" ></td>");
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> %s </td>",R.VehID);
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#330033\">TRUCK</td>");
						
					}
					if(VehClass==TRANSIT && R.iIsCanceled==7)
					{
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><img src=\"./pics/red.png\"  width=\"22\" height=\"22\" ></td>");
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> %s </td>",R.VehID);
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#330033\">TRANSIT</td>");
					}
					if(VehClass==TRUCK && R.iIsCanceled==7)
					{
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><img src=\"./pics/red.png\"  width=\"22\" height=\"22\" ></td>");
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> %s </td>",R.VehID);
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#330033\">TRUCK</td>");
						
					}
                }
                else if(HighestP==TRUCK)  // NO EV case, ONLY have request of TRANSIT and TRUCK
                {      
					if ( R.iIsCanceled !=7)       
					{
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><img src=\"./pics/green.png\"  width=\"22\" height=\"22\" ></td>");
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> %s </td>",R.VehID);
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#330033\">TRUCK</td>");
					}else
					{
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><img src=\"./pics/red.png\"  width=\"22\" height=\"22\" ></td>");
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> %s </td>",R.VehID);
						printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#330033\">TRUCK</td>");
					}
				}                
                else                
                {   // EV case, ONLY both requests of TRANSIT and EV

                    if(VehClass==EV)   
                    {
                        printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><img src=\"./pics/green.png\"  width=\"22\" height=\"22\"  ></td>");
                        printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> %s </td>",R.VehID);
                        printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#CC0000\">EV</td>");
                    }
                    else if(VehClass==TRANSIT)
                    {
                        printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><img src=\"./pics/red.png\"  width=\"22\" height=\"22\"  ></td>");
                        printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> %s </td>",R.VehID);
                        printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#330033\">TRANSIT</td>");
                    }else if(VehClass==TRUCK)
                    {
					   printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><img src=\"./pics/red.png\"  width=\"22\" height=\"22\"  ></td>");
                        printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> %s </td>",R.VehID);
                        printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#330033\">TRUCK</td>");
                    }
                }  
             
   
                
                printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#330033\"> %s </td>",R.cInLane);
                  printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#330033\"> %s </td>",R.cOutLane);

                Phase=R.Phase; split_phase=R.Split_Phase;
                if(split_phase>0)
                {
                    printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\">%d & %d</td>",Phase,split_phase);
                }
                else
                {
                    printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\">%d</td>",Phase);
                }
                
                    ETA=R.ETA;
                    
                    
                if ( (-1000.0 <R.ETA) && (1000.0 >R.ETA) && (-1000.0<R.MinGreen) && (1000.0>R.MinGreen) ) 
				{
					if(R.MinGreen==0) 
					    printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\">%4.2f</td>",ETA);	
					else 
					    printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#FF0000\">%4.2f</td>",R.MinGreen);
				}
				else
					    printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#FF0000\"> -- </td>");

                    

                             
                if (( -1 <R.iStartHour) && (25>R.iStartHour) && (-60< R.iStartMin) && (60> R.iStartMin) && (-60<R.iStartSec) && (60>R.iStartSec))
                {             
					printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\">%d:%d:%d</td>",R.iStartHour,R.iStartMin,R.iStartSec);
				}else
					printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- : -- : -- </td>");
                
                if ( (-1< R.iEndHour) && (25> R.iEndHour) && (-60<R.iEndMin) && (60>R.iEndMin)&& (-60<R.iEndSec) &&(60>R.iEndSec))
					printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#FF0000\">%d:%d:%d</td>",R.iEndHour,R.iEndMin,R.iEndSec);
				else
					printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#FF0000\"> -- : -- : -- </td>");
              
				if (R.iVehicleState==1)
				{
					printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> Approaching</td>");
				}else if (R.iVehicleState==2)
				{
					printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> In the Queue</td>");
				}else if (R.iVehicleState==3)
				{
					printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> Leaving</td>");
				}
				
                printf("</tr>\n");



                ReqList_Com.Next();
                iRequestEntry++;
            }

         printf("\t\t</table>\n");
         printf("<br> <br>");
			

        }
        else //if (num_req<=0)
        {
			
			
			printf("<div style=\"position:relative; top:100px; left:0px;\">");
			printf("<H2> <font color=\"yellow\" size=\"6\" style=\"font-family:Time New Roman\" > Current Request Table <H2>\n");
            printf("\t\t<table border=\"1\" bordercolor=\"#FFFFFF\" >\n");
            printf("\t\t\t<tr><th> Request Entry </th><th>Active</th><th>Vehicle ID</th><th>Vehicle Type</th><th>Vehicle Class</th> <th> In Lane</th> <th> Out Lane</th> <th> Phase(s) </th> <th>Arrival Time</th><th>Time of Service Desired </th> <th> Time of Estimated Departure </th> <th> Vehicle State</th> </tr>\n");
			printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("<td bgcolor=\"#FFFFFF\" align=\"center\"><font color=\"#000000\"> -- </td>");
            printf("\t\t</table>\n");
            printf("<br> <br>");
		}
    }
    else // if(!isconn)
    {
        printf("<H2>No Active Request(NOT Connected)! <H2>\n");
    }

}

int FindListHighestPriority(LinkedList<ReqEntry> ReqList)
{
	fprintf (stderr, "in function : %s\n", __func__);
    ReqList.Reset();

    int priority=10;  // TRANSIT(2) is lower than EV(1)

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

void FindCombinedReq(LinkedList<ReqEntry> &ReqList, ReqEntry &TestEntry)
{
	ReqList.Reset();

	int SplitPhase=FindCombinedPhase(TestEntry.Phase);

	if(!ReqList.ListEmpty())
	{
		while(!ReqList.EndOfList())
		{
			if(strcmp (ReqList.Data().VehID,TestEntry.VehID)==0 && ReqList.Data().Phase == TestEntry.Phase)
			{
				ReqList.DeleteAt();
				if(TestEntry.VehClass==TRANSIT) break;
			}
			else if(strcmp (ReqList.Data().VehID,TestEntry.VehID)==0 && ReqList.Data().Phase == SplitPhase)
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
		cout<<"VehID\tClass\tETA\tPhase\tQueueCLR\n";

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
		cout<<"VehID\tClass\tETA\tPhase\tSplit\tQueueCLR\n";

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


//*** TODO: ***//
int FindCombinedPhase(int phase)
{
	int CombinedPhase[8]={1,2,3,4,5,6,7,8};
    //*** From global array CombinedPhase[] to get the combined phase :get_configfile() generates CombinedPhase[]***//
    //*** If the phase exsits, the value is not 0; if not exists, the default value is '0'. ***//
    //*** The argument phase should be among {1..8}
    //int Phase_Seq[8]={1,2,3,4,5,6,7,8};

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
    //	printf("<div style=\"position:fixed; top:0px; left:0px;\"> Wrong Phase Information </div>");
		
      
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
	char OBU_ID[128];
	int Veh_Class,Req_Phase;
	int abs_time;
	float ETA,MinGrn;
	int inlane, outlane, strhour, strmin, strsec, endhour, endmin, endsec;
	int vehstate;
	char cinlane[64];
	char coutlane[64];
	int tmpinlane,tmpoutlane;
	int imsgcnt;
	int iIsCanceled;

	string lineread;
	Req_List.ClearList();


	getline(fss,lineread);
	sscanf(lineread.c_str(),"%*s %d",&ReqNo); //---IMPORTANT---- TODO: differet from the function in other apps
	while(!fss.eof())
	{
		getline(fss,lineread);

		if(lineread.size()!=0)    
		{
			sscanf(lineread.c_str(),"%*s %s %d %f %d %f %d %d %d %d %d %d %d %d %d %d %d %d",OBU_ID,&Veh_Class,
				&ETA,&Req_Phase,&MinGrn,&abs_time, &inlane, &outlane, &strhour, &strmin, &strsec, &endhour, &endmin, &endsec, &vehstate, &imsgcnt, &iIsCanceled);
				
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
			

			ReqEntry req_temp(OBU_ID,Veh_Class,ETA,Req_Phase,MinGrn,abs_time,0,inlane, outlane, strhour, strmin, strsec, endhour, endmin, endsec, vehstate,cinlane,coutlane, imsgcnt,iIsCanceled);

			Req_List.InsertAfter(req_temp);

			//cout<<lineread<<endl;
		}
	}
	fss.close();

    return ReqNo;


}
