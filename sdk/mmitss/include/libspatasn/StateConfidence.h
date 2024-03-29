/*
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "J2735SPATMESSAGE"
 * 	found in "module.asn1"
 * 	`asn1c -S/skeletons`
 */

#ifndef	_StateConfidence_H_
#define	_StateConfidence_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum StateConfidence {
	StateConfidence_unKnownEstimate	= 0,
	StateConfidence_minTime	= 1,
	StateConfidence_maxTime	= 2,
	StateConfidence_timeLikeklyToChange	= 3
} e_StateConfidence;

/* StateConfidence */
typedef long	 StateConfidence_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_StateConfidence;
asn_struct_free_f StateConfidence_free;
asn_struct_print_f StateConfidence_print;
asn_constr_check_f StateConfidence_constraint;
ber_type_decoder_f StateConfidence_decode_ber;
der_type_encoder_f StateConfidence_encode_der;
xer_type_decoder_f StateConfidence_decode_xer;
xer_type_encoder_f StateConfidence_encode_xer;

#ifdef __cplusplus
}
#endif

#endif	/* _StateConfidence_H_ */
