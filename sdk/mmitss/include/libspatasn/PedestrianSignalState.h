/*
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "J2735SPATMESSAGE"
 * 	found in "module.asn1"
 * 	`asn1c -S/skeletons`
 */

#ifndef	_PedestrianSignalState_H_
#define	_PedestrianSignalState_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PedestrianSignalState {
	PedestrianSignalState_unavailable	= 0,
	PedestrianSignalState_stop	= 1,
	PedestrianSignalState_caution	= 2,
	PedestrianSignalState_walk	= 3
} e_PedestrianSignalState;

/* PedestrianSignalState */
typedef long	 PedestrianSignalState_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PedestrianSignalState;
asn_struct_free_f PedestrianSignalState_free;
asn_struct_print_f PedestrianSignalState_print;
asn_constr_check_f PedestrianSignalState_constraint;
ber_type_decoder_f PedestrianSignalState_decode_ber;
der_type_encoder_f PedestrianSignalState_encode_der;
xer_type_decoder_f PedestrianSignalState_decode_xer;
xer_type_encoder_f PedestrianSignalState_encode_xer;

#ifdef __cplusplus
}
#endif

#endif	/* _PedestrianSignalState_H_ */
