//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************


#include <stdio.h>
#include <stdlib.h>
#include "libgps.h"
#include <j2735common.h>

#include "main.h"

uint8_t buf[BUF_SIZE];
bsm_t bsm;

extern long veh_id;

void printbsmcsv ();

void printbsmcsv () {
	static int first = 1;
	
	if (first) {
		log ("Type,Timestamp,MsgID,MsgCnt,TempID,SecMark,Latt,Long,Elev,Speed,Heading\n");
		first = 0;
	}

	log ("BSM,");
	log ("%lf,", gps.time);
	log ("%x,", bsm.msgid);
	log ("%x,", bsm.msgcount);
	log ("%lld,", bsm.temp_id);
	log ("%d,", bsm.secmark);
	log ("%f,", bsm.latitude);
	log ("%f,", bsm.longitude);
	log ("%f,", bsm.elevation);
	log ("%f,", bsm.speed);
	log ("%f\n", bsm.heading);
}

void fill_bsm () {
	static int bsm_counter = 0;

	read_gps();

	bsm.msgid = 2;
	bsm.msgcount = (++bsm_counter) % 128;
	bsm.temp_id = veh_id;
	bsm.secmark = gps.dsecond;
	bsm.latitude = gps.latitude;
	bsm.longitude = gps.longitude;
	bsm.elevation = gps.altitude;
	bsm.speed = gps.speed;
	bsm.heading = gps.heading;
}

void printbsmbuf () {
	int i;
	for (i = 0; i < 100; i++)
		printf ("%02x ", buf[i]);
	printf ("\n");
}

void pack_bsm () {
	BSM_TYPE type = (BSM_TYPE) (BSM_PART1);

	memset(&bsm, 0, sizeof(bsm_t));
	memset(&buf, 0, BUF_SIZE);

	fill_bsm ();

	if (j2735_encode_bsm(&bsm, type,(char *) buf, BUF_SIZE) < 0) {
		fprintf(stderr, "BSM: Encode Failure\n");
		return;
	}

	printbsmcsv();
	/* printbsmbuf(); */
}
