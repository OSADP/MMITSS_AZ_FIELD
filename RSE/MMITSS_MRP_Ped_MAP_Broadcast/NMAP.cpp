//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************



/* NMAP.cpp
*  Created by :Yiheng Feng
*  University of Arizona   
*  ATLAS Research Center 
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.

*Modified by Sara Khosravi: change the map format to ped map

*/

#include "NMAP.h"
#include<iostream>
#include <string>
#include <fstream>
#include "stdlib.h"
#include "math.h"

#define DEFAULT_ARRAY_SIZE 80

using namespace std;

//This function parse information from *.nmap and save to the MAP data structure
int MAP::ParseIntersection(char* filename)
{
	//Start parsing the MAP data from file

	//Note: when the approaches number of lane number exceeds 10, there will be problem with token_char[0]-48,
	//because now the number takes 2 positions
	ifstream map_file; 
	map_file.open(filename);

	if (!map_file)
	{
		cout<<"Error: Open file "<<filename<<endl;
		return -1;
	}

	string lineread;
	char token_char [DEFAULT_ARRAY_SIZE];
	char temp_char [DEFAULT_ARRAY_SIZE];
	string token;
	// Read first 6 lines of Intersection information
	int i,j,k,l;
	for(i=0;i<5;i++)
	{
		getline(map_file, lineread); // Read line by line
		sscanf(lineread.c_str(), "%s",token_char);
		//token.assign(token_char);
		if(strcmp(token_char,"MAP_Name")==0)
		{
			sscanf(lineread.c_str(), "%*s %s",temp_char);
			intersection.Map_Name.assign(temp_char);
		}
		else if(strcmp(token_char,"RSU_ID")==0)
		{
			sscanf(lineread.c_str(), "%*s %s",temp_char);
			intersection.Rsu_ID.assign(temp_char);
		}
		else if(strcmp(token_char,"IntersectionID")==0)
		{
			sscanf(lineread.c_str(), "%*s %s",temp_char);
			intersection.ID=atoi(temp_char);
		}

		else if(strcmp(token_char,"Reference_point")==0)
		{
			sscanf(lineread.c_str(), "%*s %lf %lf %lf",&intersection.Ref_Lat,&intersection.Ref_Long,&intersection.Ref_Ele);
		}
		else if(strcmp(token_char,"No_Approach")==0)
		{
			sscanf(lineread.c_str(), "%*s %d",&intersection.Appro_No);
		}
	}
	//end of reading intersection information

	//initialize approaches
	Approach temp_appro;
	for(i=0;i<intersection.Appro_No;i++)
	{
		intersection.Approaches.push_back(temp_appro);
	}

    cout<<"No. of approach is: "<<intersection.Appro_No<<endl;
	//Start to parse the approaches, lanes and lane nodes
	for(i=0;i<intersection.Appro_No;i++)
	{
		getline(map_file, lineread); // Read line by line
		sscanf(lineread.c_str(), "%s",token_char);
		while(strcmp(token_char,"end_approach"))  //if not end_approach
		{
			//The next 3 lines: Read Approach attributes
			for(j=0;j<3;j++)
			{
				if(strcmp(token_char,"Approach")==0)
				{
					sscanf(lineread.c_str(), "%*s %s",temp_char);
					intersection.Approaches[i].ID=atoi(temp_char);
					getline(map_file, lineread); // Read line by line
					sscanf(lineread.c_str(), "%s",token_char);
				}
				else if(strcmp(token_char,"Approach_type")==0)
				{
					sscanf(lineread.c_str(), "%*s %s",temp_char);
					intersection.Approaches[i].type=atoi(temp_char);
					getline(map_file, lineread); // Read line by line
					sscanf(lineread.c_str(), "%s",token_char);
				}
				else if(strcmp(token_char,"No_nodes")==0)
				{
					sscanf(lineread.c_str(), "%*s %s",temp_char);
					intersection.Approaches[i].Node_No=atoi(temp_char);
				}
				cout<< "node complete"<<endl;
			}
			//End reading approaches attributes


			//Initialize Nodes First
			LaneNodes temp_node;
			for(k=0;k<intersection.Approaches[i].Node_No;k++)
			{
				intersection.Approaches[i].Nodes.push_back(temp_node);
			}
			//Start to read the nodes
			for(k=0;k<intersection.Approaches[i].Node_No;k++)  //get the node number
			{
				getline(map_file, lineread); // Read line by line
				sscanf(lineread.c_str(), "%s",token_char);
				//get element number from token char
				intersection.Approaches[i].Nodes[k].index.Approach=token_char[0]-49;  //Token_char is 2.1
				intersection.Approaches[i].Nodes[k].index.Node=k+1;
				//get lat, long data
				sscanf(lineread.c_str(), "%*s %lf %lf",&intersection.Approaches[i].Nodes[k].Latitude,
						&intersection.Approaches[i].Nodes[k].Longitude);
				cout<<"Reading node "<<intersection.Approaches[i].Nodes[k].index.Approach<<" "<<intersection.Approaches[i].Nodes[k].index.Node << endl;
			}
			getline(map_file, lineread); // Read line by line
			sscanf(lineread.c_str(), "%s",token_char);
		}
	}
	//The end line
	getline(map_file, lineread); // Read line by line
	sscanf(lineread.c_str(), "%s",token_char);
	cout << "token_char : " << token_char << endl;
	if(strcmp(token_char,"end_map")==0)
	{
		cout<<"Parse map file successfully!"<<endl;
	}
}
