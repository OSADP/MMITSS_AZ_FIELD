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
#include <ProbeDataManagement.h>

#define PDM_BUF_SIZE 128
int main (int argc, char *argv[]) {
	ProbeDataManagement_t *PDM = NULL, *PDM_DEC = NULL;
	VehicleStatusRequest_t *dataElements = NULL;
	int i, headingSlice = 400;
	unsigned char PDMbuf[PDM_BUF_SIZE];
	long subType = 8;
	long sendOnLessThenValue = 1024;
	long sendOnMoreThenValue = 1024;
	BOOLEAN_t sendAll = 1;
	asn_enc_rval_t enc_ret_val;
	asn_dec_rval_t dec_ret_val;

	PDM = (ProbeDataManagement_t *) calloc (1, sizeof(ProbeDataManagement_t));
	dataElements = (VehicleStatusRequest_t*) calloc (1, sizeof(VehicleStatusRequest_t));

	PDM->msgID = DSRCmsgID_probeDataManagement;
	PDM->sample.sampleStart = 0;
	PDM->sample.sampleEnd = 255;
	PDM->directions = *OCTET_STRING_new_fromBuf(&asn_DEF_OCTET_STRING,(char*)&headingSlice,-1);
	PDM->term.present = term_PR_termtime;
	PDM->term.choice.termtime = 574;
	PDM->snapshot.present = snapshot_PR_snapshotTime;
	PDM->snapshot.choice.snapshotTime.t1 = 20;
	PDM->snapshot.choice.snapshotTime.s1 = 6;
	PDM->snapshot.choice.snapshotTime.t2 = 60;
	PDM->snapshot.choice.snapshotTime.s2 = 20;
	PDM->txInterval = 10;
	PDM->cntTthreshold = 16;
	
	dataElements->dataType = VehicleStatusDeviceTypeTag_hozAccelLong;
	dataElements->subType = &subType;
	dataElements->sendOnLessThenValue = &sendOnLessThenValue;
	dataElements->sendOnMoreThenValue = &sendOnMoreThenValue;
	dataElements->sendAll = &sendAll;
	asn_sequence_add(&PDM->dataElements,dataElements);

	printf ("\nStructure Before Encoding: \n");
	xer_fprint (stdout, &asn_DEF_ProbeDataManagement, PDM);
	printf ("\n===========================\n");

	enc_ret_val = der_encode_to_buffer (&asn_DEF_ProbeDataManagement, PDM, PDMbuf, PDM_BUF_SIZE);
	printf ("\nBuffer Contents : \n");
	for (i = 0; i < PDM_BUF_SIZE; i++) {
		if (!(i % 32)) printf ("\n");
		if (!(i % 16)) printf ("  ");
		if (!(i % 8)) printf ("  ");
		if (!(i % 4)) printf ("  ");
		printf ("%02x ", PDMbuf[i]);
	}
	printf ("\n===========================\n");

	dec_ret_val = ber_decode (0, &asn_DEF_ProbeDataManagement, (void **) &PDM_DEC, PDMbuf, PDM_BUF_SIZE);
	printf ("\nStructure After Decoding: \n");
	xer_fprint (stdout, &asn_DEF_ProbeDataManagement, PDM_DEC);
	printf ("\n===========================\n");
	return getchar();
}