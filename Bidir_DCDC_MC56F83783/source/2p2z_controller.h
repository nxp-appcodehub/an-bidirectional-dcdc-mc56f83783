/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * controller.h
 *
 *  Created on: Feb 3, 2023
 *      Author: nxa22573
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "mlib.h"
#include "gflib.h"

#define CFD      28 /* Coefficient fractional digits */
#define IFD      30 /* Internal result fractional digits */

typedef struct
{
	int32_t i32UpperLim;
	int32_t i32LowerLim;
}FILTER_LIM_T;

typedef struct
{
	int32_t i32B0;         // +4 ,B0
	int32_t i32B1;         // +6 ,B1
	int32_t i32A1;         // +8 ,A1
	int32_t i32B2;         // +10,B2
	int32_t i32A2;         // +12,A2
}COEFF_2P2Z_TRANS_T;

typedef struct
{
	int32_t i32W1;
	int32_t i32W2;
	COEFF_2P2Z_TRANS_T sCoeff;
	FILTER_LIM_T sLim;
}FILTER_2P2Z_TRANS_LIM_T;

#define FRAC_Q31(x) x*0x80000000
#define FRAC_Q30(x) x*0x40000000
#define FRAC_Q29(x) x*0x20000000
#define FRAC_Q28(x) x*0x10000000
#define FRAC_Q27(x) x*0x8000000
#define FRAC_Q26(x) x*0x4000000
#define FRAC_Q25(x) x*0x2000000
#define FRAC_Q24(x) x*0x1000000
#define FRAC_Q23(x) x*0x800000
#define FRAC_Q22(x) x*0x400000
#define FRAC_Q21(x) x*0x200000
#define FRAC_Q20(x) x*0x100000
#define FRAC_Q19(x) x*0x80000
#define FRAC_Q18(x) x*0x40000
#define FRAC_Q17(x) x*0x20000
#define FRAC_Q16(x) x*0x10000
#define FRAC_Q15(x) x*0x8000
#define FRAC_Q14(x) x*0x4000
#define FRAC_Q13(x) x*0x2000
#define FRAC_Q12(x) x*0x1000
#define FRAC_Q11(x) x*0x800
#define FRAC_Q10(x) x*0x400
#define FRAC_Q9(x) x*0x200
#define FRAC_Q8(x) x*0x100
#define FRAC_Q7(x) x*0x80
#define FRAC_Q6(x) x*0x40
#define FRAC_Q5(x) x*0x20
#define FRAC_Q4(x) x*0x10
#define FRAC_Q3(x) x*0x8
#define FRAC_Q2(x) x*0x4
#define FRAC_Q1(x) x*0x2

#define FRAC_DYN(y,x) FRAC_DYN1(y,x)
#define FRAC_DYN1(y,x) FRAC_Q##y(x)

/* Transposed Direct form II - Fractional in Assembly language, with MAC and parallel move - Inline version */
/*
 *  
 *  @param   f16In - Q1.15 input 
 *		     ptr - A pointer to the controller with limiting performed on the output the controller only.
 *	@return  Q1.15 format value	    
 * */
inline frac16_t IIR_2P2Z_II_TRANS_LIM_output_mac_asm_sat_inline_F16(register frac16_t f16In, register FILTER_2P2Z_TRANS_LIM_T *ptr)
{
	
	/* 
	 *  typedef struct
		{
			int32_t i32B0;         // +4 ,B0 8EA
			int32_t i32B1;         // +6 ,B1 8EC
			int32_t i32A1;         // +8 ,A1 8EE
			int32_t i32B2;         // +10,B2 8F0
			int32_t i32A2;         // +12,A2 8F2
		}COEFF_2P2Z_TRANS_T;
		
		typedef struct
		{
			int32_t i32UpperLim;   // +14
			int32_t i32LowerLim;   // +16
		}FILTER_LIM_T;
				
		typedef struct
		{
			int32_t i32W1;         // +0 ,W1 8E6
			int32_t i32W2;         // +2 ,W2 8E8
			COEFF_2P2Z_T sCoeff;
			FILTER_LIM_T sLim;
		}FILTER_2P2Z_TRANS_T;
		
		step 1: out = w2 + in*b0
		step 2: w2 = a1*out + b1*in + w1
		step 3: w1 = a2*out + b2*in
		
	 * */
	/*     y is the input in Q(IFD) format,
	 *     b is the updated output in Q(IFD) format
	 * 
	 * */

		register int32_t *ptr_alt;
		register int32_t *ptr_lim;
		register int16_t i16OutReg;
	
	asm{
		.optimize_iasm on
		move.w f16In,a
		asrr.l #(31-IFD),a       // a = IN, Q(IFD)
		tfr    a,y               // y = IN		
		adda   #14,ptr,ptr_lim   // ptr_lim->UpperLim
		adda   #2,ptr,ptr_alt    // ptr_alt->W2, ptr->W1
		move.l x:(ptr_alt)+,a    // a = W2, ptr_alt->B0
		move.l x:(ptr_alt)+,c    // c = B0, ptr_alt->B1
		mpy32  c,y,c             // c = B0*IN
		asll.l #(31-CFD),c
		add    c,a               // a = B0*IN + W2, now a is updated output
		sat    a                 // Saturate the updated output
		
		move.l x:(ptr_lim)+,c    // c = UpperLim, ptr_lim->LowerLim
		cmp    c,a
		bge    LIM_OUTPUT
		move.l x:(ptr_lim)-,c    // c = LowerLim, ptr_lim->UpperLim
		cmp    c,a
		blt    LIM_OUTPUT
		tfr    a,c
		
LIM_OUTPUT:
		tfr    c,b               // b is updated output with limiting
		
		move.l x:(ptr_alt)+,c    // c = B1, ptr_alt->A1
		mpy32  c,y,a x:(ptr_alt)+,c       // a = B1*IN, c = A1, ptr_alt->B2
		mac32  b,c,a             // a = B1*IN + A1*OUT
		sat    a
		asll.l #(31-CFD),a
		move.l x:(ptr)+,c        // c = W1, ptr->W2
		add    c,a               // a = W1 + B1*IN + A1*OUT, which is the updated W2
		sat    a                 // Saturate the updated W2
			
		move.l a10, x:(ptr)-     // Store W2, ptr->W1
		
		move.l x:(ptr_alt)+,c    // c = B2, ptr_alt->A2
		mpy32  c,y,a x:(ptr_alt)+,c       // a = B2*IN, c = A2
		mac32  c,b,a             // a = B2*IN + A2*OUT
		sat    a                 // Saturate the updated W1
		asll.l #(31-CFD),a
		move.l a10, x:(ptr)      // Store W1
		
		asll.l #(31-IFD),b       // Q(IFD)->Q1.31
		move.w b1,i16OutReg      // Q1.31 -> Q1.15
		.optimize_iasm off
	}
	
	return i16OutReg;
}

/************************** initialization function for transposed type-II transformation ****************************************************/
inline IIR_2P2Z_II_TRANS_LIM_output_mac_asm_inline_init_F16(FILTER_2P2Z_TRANS_LIM_T *ptr)
{
	ptr->i32W1 = 0;
	ptr->i32W2 = 0;
}


#endif /* CONTROLLER_H_ */
