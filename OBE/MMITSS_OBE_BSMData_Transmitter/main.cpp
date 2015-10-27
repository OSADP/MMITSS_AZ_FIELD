//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

#include "main.h"
#include <stdarg.h>
#include <ctype.h>

long veh_id;
char Veh_File_Name[128]="/nojournal/bin/vehicleid.txt";

volatile int sigint_received = 0;

void sigint_handler (int val) {
	sigint_received = 1;
}

void log (char *fmt, ...) {
	static FILE *logfp = NULL;
	va_list myargs;
	va_start (myargs, fmt);
	char temp[100];
	int i, j;

	if (NULL == logfp) {
		time_t curtime = time (NULL);
		struct tm *tm = localtime (&curtime);
		char filename[100];
		sprintf (filename, "/nojournal/bin/log/bsmd_log_%02d_%02d_%02d_%02d_%02d", 
				tm->tm_mon, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
		fprintf (stderr, "writing log to file %s\n", filename);
		if ((logfp = fopen (filename, "w")) < 0) {
			fprintf (stderr, "COuldn't open log file\n");
			exit (-1);
		}
	}

	/* vprintf (fmt, myargs); */

	vsnprintf (temp, 100, fmt, myargs);
	for (i = 0; temp[i]; i++) {
		if (temp[i] == '\n') continue;
		if (temp[i] > 128 || !isprint(temp[i])) {
			fprintf (logfp, "Illegal character found at %d. Full buffer : ", i);
			for (j = 0; temp[i]; j++)
				fprintf (logfp, "%c:%d\t", temp[i], temp[i]);
			fprintf (logfp, "\n");
			return;
		}
	}

	vfprintf (logfp, fmt, myargs);
	va_end (myargs);
}

int main (int argc, char **argv) {

	//get vehicle id from vehicleid.txt
	FILE *f;
	f=fopen(Veh_File_Name,"r");
	fscanf(f,"%d",&veh_id);
	fclose(f);

	gps_init();
	wme_initialize ();
	signal (SIGINT, sigint_handler);

	while (sigint_received == 0) {
		pack_bsm ();
		send_bsm ();

		usleep (100000);
	}

	gps_close();

	return 0;
}
