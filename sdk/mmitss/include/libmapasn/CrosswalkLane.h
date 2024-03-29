/*
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "J2735MAPMESSAGE"
 * 	found in "module.asn1"
 * 	`asn1c -S/skeletons`
 */

#ifndef	_CrosswalkLane_H_
#define	_CrosswalkLane_H_


#include <asn_application.h>

/* Including external dependencies */
#include "LaneNumber.h"
#include "LaneWidth.h"
#include "CrosswalkLaneAttributes.h"
#include "NodeList.h"
#include "ConnectsTo.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct NodeList;

/* CrosswalkLane */
typedef struct CrosswalkLane {
	LaneNumber_t	 laneNumber;
	LaneWidth_t	*laneWidth	/* OPTIONAL */;
	CrosswalkLaneAttributes_t	 laneAttributes;
	NodeList_t	 nodeList;
	struct NodeList	*keepOutList	/* OPTIONAL */;
	ConnectsTo_t	*connectsTo	/* OPTIONAL */;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CrosswalkLane_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CrosswalkLane;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "NodeList.h"

#endif	/* _CrosswalkLane_H_ */
