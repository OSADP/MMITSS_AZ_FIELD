#include "main.h"

#include <time.h>

J2735SRM_t srm; // Mehdi Added

void printsrmcsv () {
	static int first = 1;
	
	if (first) {
		log ("Type,Timestamp,MsgID,MsgCnt,IntsID,Latt,Long,Elev,Speed,Heading\n");
		first = 0;
	}

	log ("SRM,");
	log ("%lf,", gps.time);
	log ("%x,", srm.message_id);
	log ("%x,", srm.msgcount);
	log ("%x,", (unsigned int) srm.intersection_id);
	log ("%f,", srm.latitude);
	log ("%f,", srm.longitude);
	log ("%f,", srm.elevation);
	log ("%f,", srm.speed);
	log ("%f\n", srm.heading);
}

void fill_srm () {
	static int srm_counter = 0;
	time_t gpstime = (time_t) gps.time;
	struct tm *ctime = localtime (&gpstime);

	srm.starttime_hour = ctime->tm_hour;
	srm.starttime_min  = ctime->tm_min;
	srm.starttime_sec  = ctime->tm_sec;
	srm.endtime_hour   = ctime->tm_hour;
	srm.endtime_min    = ctime->tm_min;
	srm.endtime_sec    = ctime->tm_sec;

	srm.message_id           = 14;
	srm.msgcount             = (++srm_counter) % 128;
	srm.intersection_id      = 5426;
	srm.cancelreq_priority   = 0;
	srm.cancelreq_preemption = 0;
	srm.in_lanenum           = 3;
	srm.out_lanenum          = 2;

	srm.vehicle_type         = 1;
	srm.vehicle_grouptype    = 3;
	srm.vehicle_class        = 9985;
	srm.vehicleclass_type    = 1;
	srm.vehicleclass_level   = 1;
	srm.vehicle_width        = 120;
	srm.vehicle_length       = 120;
	srm.vehicle_status       = 8;
	strcpy(srm.vehicle_name,"Civic");
	strcpy(srm.vehicle_ownercode,"SVSCR"); 

	srm.bsm_msgcount = bsm.msgcount;
	srm.temp_id      = bsm.temp_id ;
	srm.dsecond      = bsm.secmark;
	srm.latitude     = bsm.latitude;
	srm.longitude    = bsm.longitude;
	srm.elevation    = bsm.elevation;
	srm.heading      = bsm.heading ;
	srm.speed        = bsm.speed;
}

void printsrmbuf () {
	int i;
	for (i = 0; i < 200; i++)
		printf ("%x ", buf[i]);
	printf ("\n");
}

void pack_srm () {
	int iType = SRM_BSMBLOB | SRM_VEHICLESTATUS;

	memset(&srm, 0, sizeof(J2735SRM_t));
	memset(&buf, 0, BUF_SIZE);

	fill_srm ();

	if (j2735_encode_srm(&srm, iType, buf, BUF_SIZE) < 0) {
		fprintf(stderr, "SRM: Encode Failure\n");
		return;
	}

	/* printsrmcsv(); */
	/* printsrmbuf(); */
}
