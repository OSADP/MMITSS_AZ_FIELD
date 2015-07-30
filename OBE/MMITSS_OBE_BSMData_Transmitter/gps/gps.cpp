#include "main.h"

gps_data_t *gps_handle;
savari_gps_data_t gps;

int gps_init () {
    int is_async = 0;
    int fd;

    gps_handle = savari_gps_open(&fd, is_async);
    if (gps_handle == 0) {
        printf("sorry no gpsd running\n");
        return -1;
    }

    return 0;
}

void printgpscsv () {
	static int first = 1;

	if (first) {
		printf ("time,latitude,longitude,elevation,speed,heading\n");
		first = 0;
	}
	printf ("%lf,", gps.time);
	printf ("%lf,", gps.latitude);
	printf ("%lf,", gps.longitude);
	printf ("%lf,", gps.altitude);
	printf ("%lf,", gps.speed);
	printf ("%lf\n", gps.heading);
}

/* int main () { */

/* 	gps_init (); */

/* 	while (1) { */
/* 		savari_gps_read (&gps, gps_handle); */
/* 		printgpscsv (); */
/* 	} */
/* 	return 0; */
/* } */

void read_gps () {
	savari_gps_read (&gps, gps_handle);
	/* printgpscsv (); */
}


void gps_close () {
	savari_gps_close (gps_handle);
}
