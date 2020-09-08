/*
 * nand_ecc.h
 *
 *  Created on: 21 рту. 2017 у.
 *      Author: alexey.chibryakov
 */

#ifndef NAND_ECC_H_
#define NAND_ECC_H_

/* #############################################################################
###  Numonyx Confidential                                                      #
###  Copyright (c) Numonyx B.V. 2004-2009 ###  All Rights Reserved.            #
###  ------------------------------------------------------------------------- #
###  Project: NFTL                                                             #
###   Filename: 512byte_ECC.h                                                  #
###                                                                            #
###  MODULE:ERROR CORRECTION CODE.ECC algorithm that detects and               #
###                corrects 1 bit errors in a 512/256 byte block of data.      #
###  NOTICE OF LICENSE AGREEMENT                                               #
###                                                                            #
###  This code is provided by Numonyx B.V., and the use is governed            #
###  under the terms of a license agreement. See license agreement for         #
###  complete terms of license.                                                #
###                                                                            #
###  YOU MAY ONLY USE THE SOFTWARE SUBJECT TO THE TERMS OF THE                 #
###  NUMONYX SOFTWARE LICENSE AGREEMENT.                                       #
###                                                                            #
##############################################################################*/

typedef unsigned char ubyte;
typedef unsigned short int uword;
typedef unsigned long int udword;

typedef enum {
    ECC_NO_ERROR			= 0,	/* no error */
    ECC_CORRECTABLE_ERROR	= 1,	/* one bit data error */
    ECC_ECC_ERROR			= 2,	/* one bit ECC error */
    ECC_UNCORRECTABLE_ERROR	= 3		/* uncorrectable error */
} eccdiff_t;

#ifdef __cplusplus
extern "C"{
#endif

void make_ecc(ubyte *ecc_code,const ubyte *data);
eccdiff_t ecc_check(ubyte *data, ubyte *stored_ecc, ubyte *new_ecc);

#ifdef __cplusplus
}

#endif


#endif /* NAND_ECC_H_ */
