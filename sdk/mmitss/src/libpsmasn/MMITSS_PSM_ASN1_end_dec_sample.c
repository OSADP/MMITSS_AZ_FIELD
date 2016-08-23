/*
 * Generic converter template for a selected ASN.1 type.
 * Copyright (c) 2005, 2006, 2007 Lev Walkin <vlm@lionet.info>.
 * All rights reserved.
 * 
 * To compile with your own ASN.1 type, please redefine the PDU as shown:
 * 
 * cc -DPDU=MyCustomType -o myDecoder.o -c converter-sample.c
 */

#ifdef	HAVE_CONFIG_H
#include <config.h>
#endif
#include <stdio.h>
#include <sys/types.h>
#include <stdlib.h>	/* for atoi(3) */
//#include <unistd.h>	/* for getopt(3) */
#include <string.h>	/* for strerror(3) */
//#include <sysexits.h>	/* for EX_* exit codes */
#include <errno.h>	/* for errno */

#include <asn_application.h>
#include <asn_internal.h>	/* for _ASN_DEFAULT_STACK_MAX */


#include <PSMmsg.h>


//#include <string>
//#include <iostream>
//#include <fstream>

asn_enc_rval_t ec; /* Encoder return value */
asn_dec_rval_t rval;
int ii;
char PSMbuf[256];
char * str;
int ret=0;
int numberOfReq=2;
long IntID=1212;
int inlane=12;
int outlane=42;
int type=5;
long vehID=1233;
int msgCnt=23;
int vehState=1;
int ETA=360; // in deci second
int priorityReqTimeOfMsg=1000;
int priorityReqTimeOfSrvDesd=1360;
int i;


	
	
int main(int ac, char *av[])
{
	printf(" numberOfReq %d \n", numberOfReq);
	printf(" IntID %l \n", IntID);
	printf(" inlane %d \n",inlane);
	printf(" outlane %d \n",outlane);
	printf(" type %d \n",type);
	printf(" vehID %l \n", vehID);
	printf(" msgCont%d \n", msgCnt);
	printf(" vehState %d \n", vehState);
	printf(" ETA %l \n" , ETA);
	printf(" priorityReqTimeOfMsg %d \n" , priorityReqTimeOfMsg);
	printf(" priorityReqTimeOfSrvDesd %d \n" , priorityReqTimeOfSrvDesd);
	
	PSMmsg_t * psm=0;
	ActiveRequestTable_t * art=0;
	DSecond_t * dSecond=0;
	psm = (PSMmsg_t *) calloc(1, sizeof * psm);
	art =(ActiveRequestTable_t *) calloc(1, sizeof(ActiveRequestTable_t));
	
	psm->id = * OCTET_STRING_new_fromBuf(&asn_DEF_OCTET_STRING,(char*)&IntID,-1);
	
	psm->msgID=17;
	psm->numberOfRequests=numberOfReq;
	for (i=0 ; i<numberOfReq; i++)
	{
		asn_sequence_add(&psm->activeRequestTable,art);
		psm->activeRequestTable.list.array[i]->inLane=*OCTET_STRING_new_fromBuf(&asn_DEF_OCTET_STRING,(char*)&inlane,-1);
		psm->activeRequestTable.list.array[i]->outLane=*OCTET_STRING_new_fromBuf(&asn_DEF_OCTET_STRING,(char*)&outlane,-1);
		psm->activeRequestTable.list.array[i]->priorityActive=0;
		psm->activeRequestTable.list.array[i]->priorityReqEntryNum=1;
		psm->activeRequestTable.list.array[i]->priorityReqID=*OCTET_STRING_new_fromBuf(&asn_DEF_OCTET_STRING,(char*)&vehID,4);
		psm->activeRequestTable.list.array[i]->priorityReqTimeToLive=0;
		psm->activeRequestTable.list.array[i]->requestSeqNumber=msgCnt;
		psm->activeRequestTable.list.array[i]->requestState=vehState;
		psm->activeRequestTable.list.array[i]->type=*OCTET_STRING_new_fromBuf(&asn_DEF_OCTET_STRING,(char*)&type,1);
		psm->activeRequestTable.list.array[i]->priorityReqTimeOfEstdDepart=ETA;
		psm->activeRequestTable.list.array[i]->priorityReqTimeOfMsg=priorityReqTimeOfMsg;
		psm->activeRequestTable.list.array[i]->priorityReqTimeOfSrvDesd=priorityReqTimeOfSrvDesd;
	}

	ec = der_encode_to_buffer(&asn_DEF_PSMmsg, psm,PSMbuf,255);
	
	rval = ber_decode(0, &asn_DEF_PSMmsg,(void **)&psm, PSMbuf, 255);
	xer_fprint(stdout, &asn_DEF_PSMmsg, psm);


		
	return 0;
}
