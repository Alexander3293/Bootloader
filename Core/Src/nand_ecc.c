/*
 * ecc.c
 *
 *  Created on: 21 рту. 2017 у.
 *      Author: alexey.chibryakov
 */

/* #############################################################################
###  Numonyx Confidential                                                      #
###  Copyright (c) Numonyx B.V. 2004-2009 ###  All Rights Reserved.            #
###  ------------------------------------------------------------------------  #
###  Project: NFTL                                                             #
###   Filename: 512byte_ECC.c                                                  #
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

#include "nand_ecc.h"

/*
 * Parity look up table
 */
static const ubyte  byte_parity_table[] = {
0xFF,0xD4,0xD2,0xF9,0xCC,0xE7,0xE1,0xCA,
0xCA,0xE1,0xE7,0xCC,0xF9,0xD2,0xD4,0xFF,
0xB4,0x9F,0x99,0xB2,0x87,0xAC,0xAA,0x81,
0x81,0xAA,0xAC,0x87,0xB2,0x99,0x9F,0xB4,
0xB2,0x99,0x9F,0xB4,0x81,0xAA,0xAC,0x87,
0x87,0xAC,0xAA,0x81,0xB4,0x9F,0x99,0xB2,
0xF9,0xD2,0xD4,0xFF,0xCA,0xE1,0xE7,0xCC,
0xCC,0xE7,0xE1,0xCA,0xFF,0xD4,0xD2,0xF9,
0xAC,0x87,0x81,0xAA,0x9F,0xB4,0xB2,0x99,
0x99,0xB2,0xB4,0x9F,0xAA,0x81,0x87,0xAC,
0xE7,0xCC,0xCA,0xE1,0xD4,0xFF,0xF9,0xD2,
0xD2,0xF9,0xFF,0xD4,0xE1,0xCA,0xCC,0xE7,
0xE1,0xCA,0xCC,0xE7,0xD2,0xF9,0xFF,0xD4,
0xD4,0xFF,0xF9,0xD2,0xE7,0xCC,0xCA,0xE1,
0xAA,0x81,0x87,0xAC,0x99,0xB2,0xB4,0x9F,
0x9F,0xB4,0xB2,0x99,0xAC,0x87,0x81,0xAA,
0xAA,0x81,0x87,0xAC,0x99,0xB2,0xB4,0x9F,
0x9F,0xB4,0xB2,0x99,0xAC,0x87,0x81,0xAA,
0xE1,0xCA,0xCC,0xE7,0xD2,0xF9,0xFF,0xD4,
0xD4,0xFF,0xF9,0xD2,0xE7,0xCC,0xCA,0xE1,
0xE7,0xCC,0xCA,0xE1,0xD4,0xFF,0xF9,0xD2,
0xD2,0xF9,0xFF,0xD4,0xE1,0xCA,0xCC,0xE7,
0xAC,0x87,0x81,0xAA,0x9F,0xB4,0xB2,0x99,
0x99,0xB2,0xB4,0x9F,0xAA,0x81,0x87,0xAC,
0xF9,0xD2,0xD4,0xFF,0xCA,0xE1,0xE7,0xCC,
0xCC,0xE7,0xE1,0xCA,0xFF,0xD4,0xD2,0xF9,
0xB2,0x99,0x9F,0xB4,0x81,0xAA,0xAC,0x87,
0x87,0xAC,0xAA,0x81,0xB4,0x9F,0x99,0xB2,
0xB4,0x9F,0x99,0xB2,0x87,0xAC,0xAA,0x81,
0x81,0xAA,0xAC,0x87,0xB2,0x99,0x9F,0xB4,
0xFF,0xD4,0xD2,0xF9,0xCC,0xE7,0xE1,0xCA,
0xCA,0xE1,0xE7,0xCC,0xF9,0xD2,0xD4,0xFF
};



/***********************Make_ecc *************************************
*  Generate 3 byte ECC code for 512/256 byte block                   *
*                                                                    *
*--------------------------------------------------------------------*
*   Parameters :                                                     *
*                ecc_code: the location where ECC should be stored   *
*               data: given data                                     *
*    No return values:                                               *
*                                                                    *
*                                                                    *
*********************************************************************/

void make_ecc(ubyte *ecc_code, const ubyte *data)
{
	ubyte byte_reg;
	uword word_reg;
	udword reg32, temp;
	udword LP0, LP1, LP2, LP3, LP4, LP5, LP6, LP7, LP8, LP9,
			LP10, LP11, LP12, LP13, LP14, LP15, LP16, LP17;
	udword *uddata;

	/* Initialize variables */
	byte_reg = 0;
	reg32 = 0;
	LP0 = LP1 = LP2 = LP3 = LP4 = LP5 = LP6 = LP7 = LP8 = LP9 =
	LP10 = LP11 = LP12 = LP13 = LP14 = LP15 = LP16 = LP17 = 0;
	ecc_code[0] = ecc_code[1] = ecc_code[2] = 0;
	uddata = (udword *)data;

	/* Build up column parity */
	for(ubyte j = 0; j < 128; j++)
		{
		temp = uddata[j];

		//BIG/LITTLE ENDIAN Support
		//temp = (udword)data[j*4] | ((udword)data[j*4+1] << 8) | ((udword)data[j*4+2] << 16) | ((udword)data[j*4+3] << 24);

		if(j & 0x01)
			LP5  ^= temp;
		else
			LP4  ^= temp;

		if(j & 0x02)
			LP7  ^= temp;
		else
			LP6  ^= temp;

		if(j & 0x04)
			LP9  ^= temp;
		else
			LP8  ^= temp;

		if(j & 0x08)
			LP11  ^= temp;
		else
			LP10  ^= temp;

		if(j & 0x10)
			LP13  ^= temp;
		else
			LP12  ^= temp;

		if(j & 0x20)
			LP15  ^= temp;
		else
			LP14  ^= temp;

		if(j & 0x40)
			LP17  ^= temp;
		else
			LP16  ^= temp;
		}

	reg32 = LP15 ^ LP14;

	byte_reg ^= (ubyte) ((reg32) & 0xFF);
	byte_reg ^= (ubyte) ((reg32 >> 8)  & 0xFF);
	byte_reg ^= (ubyte) ((reg32 >> 16) & 0xFF);
	byte_reg ^= (ubyte) ((reg32 >> 24) & 0xFF);

	byte_reg = byte_parity_table[byte_reg];
	word_reg = (uword) (LP16 >> 16) ^ (uword) LP16;
	LP16 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));
	word_reg = (uword) (LP17 >> 16) ^ (uword) LP17;
	LP17 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));
	ecc_code[2] = (ubyte)((byte_reg & 0xFE) << 1) | (byte_parity_table[(ubyte)LP16] & 0x01) |
				 ((byte_parity_table[(ubyte)LP17] & 0x01) << 1);

	/* Create line parity */
	LP0 = (ubyte) (reg32 ^ (reg32 >> 16));
	LP1  =  (ubyte) ((reg32 >> 8) ^ (reg32 >> 24));

	LP2 = (ubyte) (reg32 ^ (reg32 >> 8));
	LP3 =   (ubyte) ((reg32 >> 16) ^ (reg32 >> 24));

	word_reg = (uword) (LP4 >> 16) ^ (uword) LP4;
	LP4 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));

	word_reg = (uword) (LP5 >> 16) ^ (uword) LP5;
	LP5 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));


	word_reg = (uword) (LP6 >> 16) ^ (uword) LP6;
	LP6 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));


	word_reg = (uword) (LP7 >> 16) ^ (uword) LP7;
	LP7 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));


	word_reg = (uword) (LP8 >> 16) ^ (uword) LP8;
	LP8 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));


	word_reg = (uword) (LP9 >> 16) ^ (uword) LP9;
	LP9 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));


	word_reg = (uword) (LP10 >> 16) ^ (uword) LP10;
	LP10 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));


	word_reg = (uword) (LP11 >> 16) ^ (uword) LP11;
	LP11 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));

	word_reg = (uword) (LP12 >> 16) ^ (uword) LP12;
	LP12 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));


	word_reg = (uword) (LP13 >> 16) ^ (uword) LP13;
	LP13 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));


	word_reg = (uword) (LP14 >> 16) ^ (uword) LP14;
	LP14 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));


	word_reg = (uword) (LP15 >> 16) ^ (uword) LP15;
	LP15 = (ubyte) ((ubyte) word_reg ^ ((ubyte) (word_reg >> 8)));


	/* Calculate final ECC code */
	ecc_code[0] = (byte_parity_table[(ubyte)LP0] & 0x01) |
				((byte_parity_table[(ubyte)LP1] & 0x01) << 1) |
				((byte_parity_table[(ubyte)LP2] & 0x01) << 2) |
				((byte_parity_table[(ubyte)LP3] & 0x01) << 3) |
				((byte_parity_table[(ubyte)LP4] & 0x01) << 4) |
				((byte_parity_table[(ubyte)LP5] & 0x01) << 5) |
				((byte_parity_table[(ubyte)LP6] & 0x01) << 6) |
				((byte_parity_table[(ubyte)LP7] & 0x01) << 7);

	ecc_code[1] = ((byte_parity_table[(ubyte)LP8] & 0x01)) |
				 (byte_parity_table[(ubyte)LP9] & 0x01) << 1 |
				 (byte_parity_table[(ubyte)LP10] & 0x01) << 2 |
				 (byte_parity_table[(ubyte)LP11] & 0x01) << 3 |
				 (byte_parity_table[(ubyte)LP12] & 0x01) << 4 |
				 (byte_parity_table[(ubyte)LP13] & 0x01) << 5 |
				 (byte_parity_table[(ubyte)LP14] & 0x01) << 6 |
				 (byte_parity_table[(ubyte)LP15] & 0x01) << 7;
}

/***********************  BitCount   *********************************
*  Count the bit number                                              *
*                                                                    *
*--------------------------------------------------------------------*
*   Parameters :                                                     *
*                ecc_code :the location where ECC should be stored   *
*    No return values:                                               *
*                                                                    *
*                                                                    *
*********************************************************************/
static ubyte BitCount(ubyte* ecc_code)
{
	udword i;
	/* NFTL-23 */
	ubyte temp, count = 0;

	for(i = 0; i < 3; ++i)
		{
		temp = ecc_code[i];
		while(temp)
			{
			if(temp & 0x01)
				++count;
			temp >>= 1;
			}
		}
	return count;
}


/*********************** ecc_check   ************************************
*   Detect and correct a 1 bit error for 512/256 byte block             *
*                                                                       *
*-----------------------------------------------------------------------*
*   Parameters :                                                        *
*        stored_ecc        one ECC to be compared                       *
*        new_ecc         the other ECC to be compared                   *
*        data            content of data page                           *
*                                                                       *
*    Return values:    Upon successful completion, ecc_check returns    *
*                   SUCCESS.                                            *
*                       Otherwise, corresponding error code is returned.*
*                                                                       *
************************************************************************/
eccdiff_t ecc_check(ubyte *data, ubyte *stored_ecc, ubyte *new_ecc)
{
	ubyte bit_count, ecc_xor[3];

	udword  byte_address = 0;

	ubyte bit_address = 0;

	/* Basic Error Detection phase */
	ecc_xor[0] = new_ecc[0] ^ stored_ecc[0];
	ecc_xor[1] = new_ecc[1] ^ stored_ecc[1];
	ecc_xor[2] = new_ecc[2] ^ stored_ecc[2];

	if ((ecc_xor[0] | ecc_xor[1] | ecc_xor[2]) == 0)
		{
		/* No errors */
		return ECC_NO_ERROR;
		}
	else
		{
		/* Counts the bit number */
		bit_count = BitCount(ecc_xor);

		if (bit_count == 12)
			{
			/*Set the bit address*/
			bit_address = ((ecc_xor[2] >> 3) & 0x01) |
						((ecc_xor[2] >> 4) & 0x02) |
						((ecc_xor[2] >> 5) & 0x04);
			byte_address = ((ecc_xor[0] >> 1) & 0x01) |
						((ecc_xor[0] >> 2) & 0x02) |
						((ecc_xor[0] >> 3) & 0x04) |
						((ecc_xor[0] >> 4) & 0x08) |
						((ecc_xor[1] << 3) & 0x10) |
						((ecc_xor[1] << 2) & 0x20) |
						((ecc_xor[1] << 1) & 0x40) |
						(ecc_xor[1] & 0x80) | ((((uword)ecc_xor[2]) << 7) & 0x100);

			/*Correct bit error in the data*/
			data[byte_address] ^= (0x01 << bit_address);
			return ECC_CORRECTABLE_ERROR;
			}
		else if (bit_count == 1)
			{
			/* ECC Code Error Correction */
			stored_ecc[0] = new_ecc[0];
			stored_ecc[1] = new_ecc[1];
			stored_ecc[2] = new_ecc[2];
			return ECC_ECC_ERROR;
			}
		else
			{
			/* Uncorrectable Error */
			return ECC_UNCORRECTABLE_ERROR;
			}
		}
}
