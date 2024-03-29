/*
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "MyModule"
 * 	found in "module.asn1"
 * 	`asn1c -S/skeletons`
 */

#ifndef	_FullPositionVector_H_
#define	_FullPositionVector_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Longitude.h"
#include "Latitude.h"
#include "Elevation.h"
#include "Heading.h"
#include "TransmissionAndSpeed.h"
#include "PositionalAccuracy.h"
#include "TimeConfidence.h"
#include "PositionConfidenceSet.h"
#include "SpeedandHeadingandThrottleConfidence.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct DDateTime;

/* FullPositionVector */
typedef struct FullPositionVector {
	struct DDateTime	*utcTime	/* OPTIONAL */;
	Longitude_t	 Long;
	Latitude_t	 lat;
	Elevation_t	*elevation	/* OPTIONAL */;
	Heading_t	*heading	/* OPTIONAL */;
	TransmissionAndSpeed_t	*speed	/* OPTIONAL */;
	PositionalAccuracy_t	*posAccuracy	/* OPTIONAL */;
	TimeConfidence_t	*timeConfidence	/* OPTIONAL */;
	PositionConfidenceSet_t	*posConfidence	/* OPTIONAL */;
	SpeedandHeadingandThrottleConfidence_t	*speedConfidence	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} FullPositionVector_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_FullPositionVector;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "DDateTime.h"

#endif	/* _FullPositionVector_H_ */
