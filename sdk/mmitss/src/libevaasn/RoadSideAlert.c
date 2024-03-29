/*
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "MyModule"
 * 	found in "module.asn1"
 * 	`asn1c -S/skeletons`
 */

#include <asn_internal.h>

#include "RoadSideAlert.h"

static asn_TYPE_member_t asn_MBR_RoadSideAlert_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct RoadSideAlert, msgID),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DSRCmsgID,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"msgID"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RoadSideAlert, msgCnt),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MsgCount,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"msgCnt"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RoadSideAlert, typeEvent),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ITIScodes,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"typeEvent"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RoadSideAlert, crc),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MsgCRC,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"crc"
		},
};
static ber_tlv_tag_t asn_DEF_RoadSideAlert_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_TYPE_tag2member_t asn_MAP_RoadSideAlert_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* msgID at 37 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* msgCnt at 39 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* typeEvent at 40 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* crc at 47 */
};
static asn_SEQUENCE_specifics_t asn_SPC_RoadSideAlert_specs_1 = {
	sizeof(struct RoadSideAlert),
	offsetof(struct RoadSideAlert, _asn_ctx),
	asn_MAP_RoadSideAlert_tag2el_1,
	4,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* Start extensions */
	-1	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_RoadSideAlert = {
	"RoadSideAlert",
	"RoadSideAlert",
	SEQUENCE_free,
	SEQUENCE_print,
	SEQUENCE_constraint,
	SEQUENCE_decode_ber,
	SEQUENCE_encode_der,
	SEQUENCE_decode_xer,
	SEQUENCE_encode_xer,
	0, 0,	/* No PER support, use "-gen-PER" to enable */
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_RoadSideAlert_tags_1,
	sizeof(asn_DEF_RoadSideAlert_tags_1)
		/sizeof(asn_DEF_RoadSideAlert_tags_1[0]), /* 1 */
	asn_DEF_RoadSideAlert_tags_1,	/* Same as above */
	sizeof(asn_DEF_RoadSideAlert_tags_1)
		/sizeof(asn_DEF_RoadSideAlert_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_RoadSideAlert_1,
	4,	/* Elements count */
	&asn_SPC_RoadSideAlert_specs_1	/* Additional specs */
};

