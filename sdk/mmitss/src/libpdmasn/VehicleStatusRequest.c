/*
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "MyModule"
 * 	found in "PDM_ASN1_tool.txt"
 * 	`asn1c -S/skeletons`
 */

#include <asn_internal.h>

#include "VehicleStatusRequest.h"

static int
memb_subType_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 1 && value <= 15)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_sendOnLessThenValue_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -32767 && value <= 32767)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_sendOnMoreThenValue_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -32767 && value <= 32767)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_TYPE_member_t asn_MBR_VehicleStatusRequest_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleStatusRequest, dataType),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleStatusDeviceTypeTag,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"dataType"
		},
	{ ATF_POINTER, 4, offsetof(struct VehicleStatusRequest, subType),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_subType_constraint_1,
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"subType"
		},
	{ ATF_POINTER, 3, offsetof(struct VehicleStatusRequest, sendOnLessThenValue),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_sendOnLessThenValue_constraint_1,
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"sendOnLessThenValue"
		},
	{ ATF_POINTER, 2, offsetof(struct VehicleStatusRequest, sendOnMoreThenValue),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_sendOnMoreThenValue_constraint_1,
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"sendOnMoreThenValue"
		},
	{ ATF_POINTER, 1, offsetof(struct VehicleStatusRequest, sendAll),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BOOLEAN,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"sendAll"
		},
};
static ber_tlv_tag_t asn_DEF_VehicleStatusRequest_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_TYPE_tag2member_t asn_MAP_VehicleStatusRequest_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* dataType at 118 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* subType at 119 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* sendOnLessThenValue at 120 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* sendOnMoreThenValue at 121 */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 } /* sendAll at 122 */
};
static asn_SEQUENCE_specifics_t asn_SPC_VehicleStatusRequest_specs_1 = {
	sizeof(struct VehicleStatusRequest),
	offsetof(struct VehicleStatusRequest, _asn_ctx),
	asn_MAP_VehicleStatusRequest_tag2el_1,
	5,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	4,	/* Start extensions */
	6	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_VehicleStatusRequest = {
	"VehicleStatusRequest",
	"VehicleStatusRequest",
	SEQUENCE_free,
	SEQUENCE_print,
	SEQUENCE_constraint,
	SEQUENCE_decode_ber,
	SEQUENCE_encode_der,
	SEQUENCE_decode_xer,
	SEQUENCE_encode_xer,
	0, 0,	/* No PER support, use "-gen-PER" to enable */
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_VehicleStatusRequest_tags_1,
	sizeof(asn_DEF_VehicleStatusRequest_tags_1)
		/sizeof(asn_DEF_VehicleStatusRequest_tags_1[0]), /* 1 */
	asn_DEF_VehicleStatusRequest_tags_1,	/* Same as above */
	sizeof(asn_DEF_VehicleStatusRequest_tags_1)
		/sizeof(asn_DEF_VehicleStatusRequest_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_VehicleStatusRequest_1,
	5,	/* Elements count */
	&asn_SPC_VehicleStatusRequest_specs_1	/* Additional specs */
};

