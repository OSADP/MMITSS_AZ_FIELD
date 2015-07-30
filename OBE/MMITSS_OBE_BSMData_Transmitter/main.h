#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libgps.h>
#include <j2735common.h>

#define BUF_SIZE 2048

int wme_initialize ();
int gps_init ();
void gps_close ();
void read_gps ();
void pack_bsm ();
int send_bsm ();
void log (char *, ...);

extern bsm_t bsm;
extern savari_gps_data_t gps;
extern uint8_t buf[];
#endif
