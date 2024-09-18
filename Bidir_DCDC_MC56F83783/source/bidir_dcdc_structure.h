/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * bidir_dcdc_structure.h
 *
 *  Created on: Aug 2, 2023
 *      Author: nxf65232
 */

#ifndef BIDIR_DCDC_STRUCTURE_H_
#define BIDIR_DCDC_STRUCTURE_H_

#include "Cpu.h"
#include "pclib.h"
#include "gflib.h"
#include "gdflib.h"
#include "mlib.h"
#include "MC56F83783.h"
#include "2p2z_controller.h"

/******************************************************************************
* Types
******************************************************************************/
typedef struct 
{
	GFLIB_RAMP_T_F16      			sRamp;      
	frac16_t               			f16InitVal; 				/* Ramp initial value */
	frac16_t               			f16Target;  				/* Required value (ramp output) */
}CLLC_SOFTSTART_RAMP_T;

typedef struct
{
	frac16_t			   			f16Vhv;						/* high voltage port DC voltage sample result in Q1.15 format */
	frac16_t			   			f16VhvFilt;					/* the filtered high voltage port DC voltage sample result */
	GDFLIB_FILTER_IIR1_T_F32  		sVhvFilter;					/* Vhv IIR1 filter */
	frac16_t			   			f16Vlv;						/* low voltage port DC voltage sample result in Q1.15 format */
	frac16_t			   			f16VlvFilt;					/* the filtered low voltage port DC voltage sample result */
	GDFLIB_FILTER_IIR1_T_F32  		sVlvFilter;					/* Vlv IIR1 filter */
	
	frac16_t			   			f16VdcSoftEnd;				/* The open loop soft-start is done when the DC voltage reaches the value */
	
	CLLC_SOFTSTART_RAMP_T			sPSDutyRamp; 				/* CLLC phase shift duty cycle ramp parameters */
	CLLC_SOFTSTART_RAMP_T			sPeriodRamp; 				/* CLLC period ramp parameters */
	
	frac16_t			   			f16VdcRef;					/* DC voltage reference */
	frac16_t			   			f16VdcErr;					/* DC voltage error */
	GFLIB_CTRL_PI_P_AW_T_A32  		sPIpAWParams;  				/* DC voltage PI regulator parameters */
	bool_t                 			StopIntegFlag; 				/* PI regulator integration stop flag */
	
	FILTER_2P2Z_TRANS_LIM_T			sParam2p2z;					/* DC voltage 2P2Z regulator parameters */
	
	frac16_t       					f16VdcCtrlOut;  	    	/* output of the DC voltage controller */	
} CLLC_STRUC_VOL_CTRL_T;

typedef struct
{
	frac16_t			   			f16Ihv;						/* high voltage port DC current sample result in Q1.15 format */
	frac16_t			   			f16IhvFilt;					/* the filtered high voltage port DC current sample result */
	GDFLIB_FILTER_IIR1_T_F32  		sIhvFilter;					/* Ihv IIR1 filter */
	frac16_t			   			f16Ilv;						/* low voltage port DC current sample result in Q1.15 format */
	frac16_t			   			f16IlvFilt;					/* the filtered low voltage port DC current sample result */
	GDFLIB_FILTER_IIR1_T_F32  		sIlvFilter;					/* Ilv IIR1 filter */
	
	frac16_t			   			f16IdcRef;					/* DC current reference */
	frac16_t			   			f16IdcErr;					/* DC current error */
	GFLIB_CTRL_PI_P_AW_T_A32  		sPIpAWParams;  				/* DC current PI regulator parameters */
	bool_t                 			StopIntegFlag; 				/* PI regulator integration stop flag */
	
	frac16_t       					f16IdcCtrlOut;  	    	/* Output of the DC current controller */
	
	frac16_t						f16IhvOffset;				/* high voltage port dc current sampling offset */
	GDFLIB_FILTER_MA_T_A32    		sIhvOffsetFilter;			/* high voltage port dc current offset filter */
	frac16_t						f16IlvOffset;				/* low voltage port dc current sampling offset */
	GDFLIB_FILTER_MA_T_A32    		sIlvOffsetFilter;			/* low voltage port dc current offset filter */	
} CLLC_STRUC_CUR_CTRL_T;

typedef struct
{
	frac16_t f16Phv;           			 						/* calculated value of the high-voltage port power */
	frac16_t f16Plv;           			 						/* calculated value of the low-voltage port power */
	acc32_t  a32HvMosTempFreq;      	 						/* high-voltage side MOSFET temperature sample value [kHz] */
	uint16_t u16HvMosTemp;      		 						/* high-voltage side MOSFET temperature actual value [¡æ] */
	acc32_t  a32LvMosTempFreq;      	 						/* low-voltage side MOSFET temperature sample value [kHz] */
	uint16_t u16LvMosTemp;      		 						/* low-voltage side MOSFET temperature actual value [¡æ] */
}DCDC_STRUC_METERING_T;

typedef union
{
	uint16_t R;
	struct
	{
		uint16_t   VhvOver          : 1; 						/* high-voltage port dc voltage over voltage flag */
		uint16_t   VhvUnder         : 1; 						/* high-voltage port dc voltage under voltage flag */
		uint16_t   VlvOver      	: 1; 						/* low-voltage port dc voltage over voltage flag */
		uint16_t   VlvUnder     	: 1; 						/* low-voltage port dc voltage under voltage flag */		
		uint16_t   IhvOver          : 1; 						/* high-voltage port dc current over current flag */
		uint16_t   IlvOver          : 1; 						/* low-voltage port dc current over current flag */
		uint16_t   HvMosTempOver    : 1; 						/* high-voltage side MOSFET over temperature flag */
		uint16_t   LvMosTempOver    : 1; 						/* low-voltage side MOSFET over temperature flag */		
		uint16_t   HW_IhvOver       : 1; 						/* HW protection - high-voltage port dc current over current flag */
		uint16_t   HW_IlvOver       : 1; 						/* HW protection - low-voltage port dc current over current flag */
		uint16_t   VbatOver         : 1; 						/* battery over voltage flag */
		uint16_t   VbatUnder        : 1; 						/* battery under voltage flag */
		uint16_t   IbatOver      	: 1; 						/* battery over current flag */
		
		uint16_t   Reserved         : 3; 						
	} B;
}DCDC_STRUC_FAULT_STATUS_T;

typedef struct
{
	frac16_t     f16VhvOver;       								/* high-voltage port dc voltage maximum allowable value */ 
	frac16_t     f16VhvUnder;      								/* high-voltage port dc voltage minimum allowable value */ 
	frac16_t     f16VlvOver;       								/* low-voltage port dc voltage maximum allowable value */ 
	frac16_t     f16VlvUnder;      								/* low-voltage port dc voltage minimum allowable value */
	frac16_t     f16IhvOver;       								/* high-voltage port dc current maximum allowable value */
	frac16_t     f16IlvOver;       								/* low-voltage port dc current maximum allowable value */
	frac16_t     f16VbatOver;      								/* battery voltage maximum allowable value */
	frac16_t     f16VbatUnder;     								/* battery voltage minimum allowable value */
	frac16_t     f16IbatOver;      								/* battery current maximum allowable value */
}DCDC_STRUC_FAULT_THRESHOLDS_T;

typedef struct
{
	uint16_t     BurstOff:1; 									/* when switching frequency is above the MAX value, the flag is set and PWM output is disabled */
	uint16_t     VarInitReady:1;            					/* variables init status flag */
	uint16_t     SampOffsetReady:1;    							/* flag that indicate whether the ADC sample offset value is obtained */
	uint16_t     RelayOn:1;    									/* flag that indicate whether the relay is on or off */
	uint16_t     RunState:1;  			    					/* 1: RUN state, 0: not RUN state */
	uint16_t     Reserved:11;
} DCDC_STRUC_FLAG_T;

typedef struct
{
	CLLC_STRUC_VOL_CTRL_T             sVdcCtrl;
	CLLC_STRUC_CUR_CTRL_T             sCurCtrl;
	DCDC_STRUC_FAULT_THRESHOLDS_T     sFaultThresholds;
	DCDC_STRUC_FAULT_STATUS_T         sFaultId;
	DCDC_STRUC_FAULT_STATUS_T         sFaultIdPending;	
	DCDC_STRUC_METERING_T             sPowerMetering;	
	DCDC_STRUC_FLAG_T                 sFlag;
	
	frac16_t    					  f16PSDuty;				/* Phase-shift duty cycle */
	frac16_t    					  f16Period;				/* switching period */
	 		
	uint16_t 						  gu16WorkModeCmd;			/* Work mode command, 1: BCM, 2: BDM */
	uint16_t    					  gu16WorkModeUsed;         /* Work mode used now */
}DCDC_STRUC_CTRL_T;

#endif /* BIDIR_DCDC_STRUCTURE_H_ */
