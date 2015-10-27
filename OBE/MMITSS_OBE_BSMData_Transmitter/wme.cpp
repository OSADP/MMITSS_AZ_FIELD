//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

#include <strings.h>
#include <sys/select.h>
#include <signal.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "libwme.h"

#include "main.h"

#define PSC "MMITSS"

savari_wme_handler_t bsm_handler;
struct savariwme_tx_req bsm_tx;
struct savariwme_reg_req bsm_req;

void bsm_tx_cfg () {
    bsm_tx.psid = 0x20;
    bsm_tx.priority = 20;
    bsm_tx.channel = 172;
    bsm_tx.datarate = 6;
    bsm_tx.txpower = 15;
	for (int i = 0; i < 6; i++) bsm_tx.mac[i] = 0xFF;
    bsm_tx.expiry_time = 0;
    bsm_tx.element_id = WAVE_ELEMID_WSMP;
    bsm_tx.tx_length = 300;
}

void bsm_req_cfg () {
    bsm_req.psid = wme_convert_psid_be(bsm_tx.psid);
    strcpy (bsm_req.psc, PSC);
    bsm_req.psc_length = sizeof(PSC);
    bsm_req.priority = 20;
	for (int i = 0; i < 6; i++) bsm_req.destmacaddr[i] = 0xFF;
    bsm_req.channel = 172;
    bsm_req.request_type = LIBWME_USER_AUTOACCESS_UNCOND;
    bsm_req.extended_access = 0xFFFF;
}

static struct sockaddr_in ipaddr;                                        
static int sockfd = 0;                                                   

#include <sys/types.h>
#include <sys/socket.h>
                                                                         
void socket_main () {                                                    
	if ((sockfd = socket (AF_INET, SOCK_DGRAM, 0)) < 0) {                
		perror ("sock descriptor");                                      
		exit(-1);                                                        
	}                                                                    

	ipaddr.sin_family = AF_INET;                                         
	ipaddr.sin_port = htons (3333);                          
	ipaddr.sin_addr.s_addr = inet_addr ("127.0.0.1");                 
	memset (ipaddr.sin_zero, 0, sizeof (ipaddr.sin_zero));               

	if(connect(sockfd, (struct sockaddr*) &ipaddr, sizeof ipaddr) < 0) {
		perror ("connect socket");                                      
		exit (-1);                                                   
	}                                                                
}                                                                        

void setSocketData () {                                         
	int send_data_len, addr_len = sizeof (ipaddr);                          
	send_data_len = sendto(sockfd, buf, 2048, 0,              
			(struct sockaddr *)&ipaddr, addr_len);                          
	/* printf ("Sent data over socket\n"); */                                     
}                                                                           

int wme_initialize ()
{
	bsm_tx_cfg ();
	bsm_req_cfg ();

    if ((bsm_handler = wme_init("::1","ath1")) < 0) {
        printf ("wme handle to bsm failed\n");
        exit (-1);
    }

    wme_register_user(bsm_handler, &bsm_req);

	socket_main ();
    return 0;
}

int send_bsm () {
    /* if (wme_wsm_tx(bsm_handler, &bsm_tx, buf) != FAIL) { */
		/* /1* fprintf (stderr, "transmit success %s\n", buf); *1/ */
		/* return 0; */
	/* } */
	setSocketData ();
	return 0;
}
