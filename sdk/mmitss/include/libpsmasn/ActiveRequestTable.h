/*
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "MyModule"
 * 	found in "module.asn1"
 * 	`asn1c -S/skeletons`
 */

#ifndef	_ActiveRequestTable_H_
#define	_ActiveRequestTable_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RequestEntry.h"
#include "VehicleID.h"
#include "ActivePriority.h"
#include "NTCIPVehicleClass.h"
#include "LaneNumber.h"
#include "DSecond.h"
#include "VehicleState.h"
#include "SequenceNumber.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ActiveRequestTable */
typedef struct ActiveRequestTable {
	RequestEntry_t	 priorityReqEntryNum;
	VehicleID_t	 priorityReqID;
	ActivePriority_t	 priorityActive;
	NTCIPVehicleClass_t	 type;
	LaneNumber_t	 inLane;
	LaneNumber_t	 outLane;
	DSecond_t	 priorityReqTimeOfSrvDesd;
	DSecond_t	 priorityReqTimeOfEstdDepart;
	VehicleState_t	 requestState;
	SequenceNumber_t	 requestSeqNumber;
	DSecond_t	 priorityReqTimeOfMsg;
	DSecond_t	 priorityReqTimeToLive;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ActiveRequestTable_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ActiveRequestTable;

#ifdef __cplusplus
}
#endif

#endif	/* _ActiveRequestTable_H_ */
