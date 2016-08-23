/*
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "J2735MAPMESSAGE"
 * 	found in "module.asn1"
 * 	`asn1c -S/skeletons`
 */

#ifndef	_SignalReqScheme_H_
#define	_SignalReqScheme_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SignalReqScheme */
typedef OCTET_STRING_t	 SignalReqScheme_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SignalReqScheme;
asn_struct_free_f SignalReqScheme_free;
asn_struct_print_f SignalReqScheme_print;
asn_constr_check_f SignalReqScheme_constraint;
ber_type_decoder_f SignalReqScheme_decode_ber;
der_type_encoder_f SignalReqScheme_encode_der;
xer_type_decoder_f SignalReqScheme_decode_xer;
xer_type_encoder_f SignalReqScheme_encode_xer;

#ifdef __cplusplus
}
#endif

#endif	/* _SignalReqScheme_H_ */