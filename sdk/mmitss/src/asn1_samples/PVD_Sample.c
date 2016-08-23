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
#include <stdlib.h>
#include <sys/types.h>
#include <stdlib.h>	/* for atoi(3) */
//#include <unistd.h>	/* for getopt(3) */
#include <string.h>	/* for strerror(3) */
//#include <sysexits.h>	/* for EX_* exit codes */
#include <errno.h>	/* for errno */

#include <asn_application.h>
#include <asn_internal.h>	/* for _ASN_DEFAULT_STACK_MAX */
#include <ProbeVehicleData.h>

#define PVD_BUF_SIZE 128
int main (int argc, char *argv[]) {
	int i;
	ProbeVehicleData_t *PVD =NULL, *PVD_DEC = NULL;
	Snapshot_t *snapshot = NULL;
	asn_enc_rval_t enc_ret_val;
	asn_dec_rval_t dec_ret_val;
	unsigned char PVDbuf[PVD_BUF_SIZE];

	PVD = (ProbeVehicleData_t *) calloc (1, sizeof(ProbeVehicleData_t));
	snapshot = (Snapshot_t *) calloc (1, sizeof(Snapshot_t));

	PVD->msgID = DSRCmsgID_probeVehicleData;
	PVD->startVector.Long = 1100;
	PVD->startVector.lat = 330;
	PVD->vehicleType = VehicleType_car;
	snapshot->thePosition.Long = 1100;
	snapshot->thePosition.lat = 330;
	asn_sequence_add(&PVD->snapshots,snapshot);


	printf ("\nStructure Before Encoding: \n");
	xer_fprint (stdout, &asn_DEF_ProbeVehicleData, PVD);
	printf ("\n===========================\n");

	enc_ret_val = der_encode_to_buffer (&asn_DEF_ProbeVehicleData, PVD, PVDbuf, PVD_BUF_SIZE);
	printf ("\nBuffer Contents : \n");
	for (i = 0; i < PVD_BUF_SIZE; i++) {
		if (!(i % 32)) printf ("\n");
		if (!(i % 16)) printf ("  ");
		if (!(i % 8)) printf ("  ");
		if (!(i % 4)) printf ("  ");
		printf ("%02x ", PVDbuf[i]);
	}
	printf ("\n===========================\n");

	dec_ret_val = ber_decode (0, &asn_DEF_ProbeVehicleData, (void **) &PVD_DEC, PVDbuf, PVD_BUF_SIZE);
	printf ("\nStructure After Decoding: \n");
	xer_fprint (stdout, &asn_DEF_ProbeVehicleData, PVD_DEC);
	printf ("\n===========================\n");
	return getchar();
}