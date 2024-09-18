/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * bidir_dcdc_ctrl.h
 *
 *  Created on: Aug 2, 2023
 *      Author: nxf65232
 */

#ifndef BIDIR_DCDC_CTRL_H_
#define BIDIR_DCDC_CTRL_H_

/******************************************************************************
* Macros 
******************************************************************************/

/* work mode */
#define BCM    		 1		 /* Battery charge mode */  
#define BDM    		 2		 /* Battery discharge mode */
#define BOARD_TEST   0       /* 0= power conversion, 1= board test
                                BCM mode is selected when board test is enabled, don't connect the external power supply to the DC port */
/* work command */
#define RUN    		 1		 /* Run the converter */  
#define STOP    	 0		 /* Stop the converter */

#define SR_ENABLE    1  	 /* synchronous rectification control: 1 = enabled; 0 = disabled */

/************************************** control mode selection **************************************/
//note: once open loop mode is enabled in either BDM or BCM, open loop mode is used in both BDM and BCM.
/* BCM control mode selection */
#define BCM_MODE_OPTION				2		/* select the control mode here */ 

#define BCM_OPEN_LOOP               1  		/* open loop */
#define BCM_SINGLE_VOLT_PI_LOOP     2 		/* output voltage single loop control using PI regulator */
#define BCM_SINGLE_VOLT_2P2Z_LOOP   3 		/* output voltage single loop control using 2p2z regulator */
#define BCM_VOLT_CUR_COMP_LOOP   	4  		/* output voltage loop and output current loop for CC/CV charging control */

/* BDM control mode selection */
#define BDM_MODE_OPTION				2		/* select the control mode here */ 

#define BDM_OPEN_LOOP               1  		/* open loop */
#define BDM_SINGLE_VOLT_PI_LOOP     2 		/* output voltage single loop control using PI regulator */
#define BDM_SINGLE_VOLT_2P2Z_LOOP   3 		/* output voltage single loop control using 2p2z regulator */

/************************************** voltage & current scales ************************************************/
#define VHV_SCALE        		451.0   /* MAX measurable high-voltage port DC voltage[V] */
#define IHV_SCALE        		12.4    /* MAX measurable high-voltage port DC current[A] */
#define VLV_SCALE        		84.0    /* MAX measurable low-voltage port DC voltage[V] */
#define ILV_SCALE        		74.2  	/* MAX measurable low-voltage port DC current[A] */ 
#define VBAT_SCALE       		69.3    /* MAX measurable battery port DC voltage[V] */
#define IBAT_SCALE       		74.2  	/* MAX measurable battery port DC current[A] */ 

/*************************************** fault thresholds ************************************************/
#define VHV_UP_LIMIT            420     /* MAX allowable high-voltage port DC voltage[V] */
#define VHV_LOW_LIMIT           310     /* MIN allowable high-voltage port DC voltage[V] */
#define IHV_UP_LIMIT_BCM        7     	/* MAX allowable high-voltage port DC current[A] */
#define IHV_UP_LIMIT_BDM        2.7     /* MAX allowable high-voltage port DC current[A] */
#define VLV_UP_LIMIT            65      /* MAX allowable low-voltage port DC voltage[V] */
#define VLV_LOW_LIMIT           32      /* MIN allowable low-voltage port DC voltage[V] */
#define ILV_UP_LIMIT_BCM        25      /* MAX allowable low-voltage port DC current[A] */
#define ILV_UP_LIMIT_BDM        40      /* MAX allowable low-voltage port DC current[A] */
#define VBAT_UP_LIMIT           65      /* MAX allowable battery port DC voltage[V] */
#define VBAT_LOW_LIMIT          39      /* MIN allowable battery port DC voltage[V] */
#define IBAT_UP_LIMIT           28      /* MAX allowable battery port DC current[A] */
#define HVMOS_TEMP_UP_LIMIT     100     /* MAX allowable temperature of the high-voltage side MOSFET[¡æ] */
#define LVMOS_TEMP_UP_LIMIT     100     /* MAX allowable temperature of the low-voltage side MOSFET[¡æ] */

/*************************************** soft-start parameters ************************************************/
#define BCM_FREQ_SOFT_START         250.0   		/* soft-start initial switching frequency in BCM [kHz] */
#define BCM_SOFT_START_INIT_DUTY    0.15 			/* the initial phase shift duty for soft-start in BCM */
#define BCM_SOFT_START_DUTY_STEP    0.002           /* the phase shift duty step for soft-start in BCM */
#define BCM_SOFT_START_PERIOD_STEP  0.0001          /* the period step for soft-start in BCM */

#define BDM_FREQ_SOFT_START         350.0   		/* soft-start initial switching frequency in BDM [kHz] */
#define BDM_SOFT_START_INIT_DUTY    0.15 			/* the initial phase shift duty for soft-start in BDM */
#define BDM_SOFT_START_DUTY_STEP    0.0001          /* the phase shift duty step for soft-start in BDM */
#define BDM_SOFT_START_PERIOD_STEP  0.00004         /* the period step for soft-start in BDM */

/**************************************** frequency and duty limit ***********************************************/
#define FREQ_OPEN_LOOP          	150.0   		/* open loop switching frequency ranging from 100 to 180[kHz] */
#define CTRLOUT_TO_PERIOD_GAIN  	1000    		/* controller output to PWM period (PWM VAL register value) gain */

#define BCM_CTRL_BURST_ON_HYS   	(100/310.0)    	/* BCM: In burst mode, when switching frequency reduces to the value, PWM output is enabled[kHz] */
#define BCM_CTRL_BURST_OFF_HYS  	(100/320.0)    	/* BCM: In burst mode, when switching frequency increases to the value, PWM output is disabled[kHz] */
#define BDM_CTRL_BURST_ON_HYS   	(100/340.0)    	/* BDM: In burst mode, when switching frequency reduces to the value, PWM output is enabled[kHz] */
#define BDM_CTRL_BURST_OFF_HYS  	(100/350.0)    	/* BDM: In burst mode, when switching frequency increases to the value, PWM output is disabled[kHz] */

#define CTRL_UP_LIMIT           	(100/101.0)   	/* MAX output of the controller output */ 
#define CTRL_PWM_PSM_BOUND      	(100/180.0)   	/* The boundary of the controller output between PFM and PSM */

/* ************************ digital filter parameters ************************ */
/* high-voltage port dc voltage filter parameters, loop sample time = 0.00001[sec](100kHz) */
#define DCDC_VHV_IIR_B0         FRAC32(0.0592 / 2.0)		//fc = 2kHz
#define DCDC_VHV_IIR_B1         FRAC32(0.0592 / 2.0)
#define DCDC_VHV_IIR_A1         FRAC32(-0.8816 / -2.0)
/* high-voltage port dc current filter parameters, loop sample time = 0.00001[sec](100kHz) */
#define DCDC_IHV_IIR_B0         FRAC32(0.0592 / 2.0)		//fc = 2kHz
#define DCDC_IHV_IIR_B1         FRAC32(0.0592 / 2.0)
#define DCDC_IHV_IIR_A1         FRAC32(-0.8816 / -2.0)
/* low-voltage port dc voltage filter parameters, loop sample time = 0.00001[sec](100kHz) */ 
#define DCDC_VLV_IIR_B0         FRAC32(0.0592 / 2.0)		//fc = 2kHz
#define DCDC_VLV_IIR_B1         FRAC32(0.0592 / 2.0)
#define DCDC_VLV_IIR_A1         FRAC32(-0.8816 / -2.0)
/* low-voltage port dc current filter parameters, loop sample time = 0.00001[sec](100kHz) */
#define DCDC_ILV_IIR_B0         FRAC32(0.0592 / 2.0)		//fc = 2kHz
#define DCDC_ILV_IIR_B1         FRAC32(0.0592 / 2.0)
#define DCDC_ILV_IIR_A1         FRAC32(-0.8816 / -2.0)
/* battery port dc voltage filter parameters, loop sample time = 0.00001[sec](100kHz) */  
#define DCDC_VBAT_IIR_B0        FRAC32(0.0592 / 2.0)		//fc = 2kHz
#define DCDC_VBAT_IIR_B1        FRAC32(0.0592 / 2.0)
#define DCDC_VBAT_IIR_A1        FRAC32(-0.8816 / -2.0)
/* battery port dc current filter parameters, loop sample time = 0.00001[sec](100kHz) */
#define DCDC_IBAT_IIR_B0        FRAC32(0.0592 / 2.0)		//fc = 2kHz
#define DCDC_IBAT_IIR_B1        FRAC32(0.0592 / 2.0)
#define DCDC_IBAT_IIR_A1        FRAC32(-0.8816 / -2.0)
/* MA filter number for current offset, number of samples for averaging = 2^DCDC_SAMP_OFFSET_MA_WINDOW */
#define DCDC_SAMP_OFFSET_MA_WINDOW   3

/*************************************** DC voltage control ************************************************/
/* BCM mode */
#define VLV_BCM_REF       	  	56.0 						 /* low-voltage port dc voltage reference[V] */
#define VLV_BCM_SOFT_END  	  	(VLV_BCM_REF - 0.2) 		 /* open loop soft-start required low-voltage port dc voltage, when reached, switch to close-loop control[V] */
//PI regulator parameters
#define VLV_BCM_KP            	5.0						 	 /* kp */
#define VLV_BCM_KI            	0.05						 /* ki*T */
#define VLV_BCM_UP_LIMIT      	CTRL_UP_LIMIT 				 /* restricted the minimum allowable switching frequency */
#define VLV_BCM_LOW_LIMIT     	BCM_CTRL_BURST_OFF_HYS  	 /* restricted the maximum allowable phase shift duty cycle: 0.555 - the defined value */
//2P2Z regulator parameters, (z1=4kHz£¬z2=50kHz£¬p1=30kHz£¬k=2)
#define VLV_BCM_2P2Z_B0       	2.2173913043478260869565217391304
#define VLV_BCM_2P2Z_B1       	-3.4608695652173913043478260869565
#define VLV_BCM_2P2Z_B2		  	1.278260869565217391304347826087
#define VLV_BCM_2P2Z_A1		  	1.7391304347826086956521739130435
#define VLV_BCM_2P2Z_A2		  	-0.73913043478260869565217391304348

#define VLV_BCM_COMP_UP_LIMIT 	0.0 				 	 	 				/* restricted the maximum allowable voltage loop result in current and voltage compare mode */
#define VLV_BCM_COMP_LOW_LIMIT	(BCM_CTRL_BURST_OFF_HYS - CTRL_UP_LIMIT) 	/* restricted the minimum allowable voltage loop result in current and voltage compare mode */

/* BDM mode */
#define VHV_BDM_REF       	  	380 						 /* high-voltage port dc voltage reference[V] */
#define VHV_BDM_SOFT_END  	  	(VHV_BDM_REF - 1) 		 	 /* open loop soft-start required high-voltage port dc voltage, when reached, switch to close-loop control[V] */
//PI regulator parameters
#define VHV_BDM_KP            	2.3						 	 /* kp */
#define VHV_BDM_KI            	0.01						 /* ki*T */
#define VHV_BDM_UP_LIMIT      	CTRL_UP_LIMIT 			 	 /* restricted the minimum allowable switching frequency */
#define VHV_BDM_LOW_LIMIT     	BDM_CTRL_BURST_OFF_HYS 		 /* restricted the maximum allowable phase shift duty cycle: 0.555 - the defined value */
//2P2Z regulator parameters, (z1=900Hz£¬z2=30kHz£¬p1=50kHz£¬k=2)
#define VHV_BDM_2P2Z_B0       	1.84828
#define VHV_BDM_2P2Z_B1       	-3.19784
#define VHV_BDM_2P2Z_B2		  	1.35388
#define VHV_BDM_2P2Z_A1		  	1.6
#define VHV_BDM_2P2Z_A2		  	-0.6

/**************************************** Battery charging current control ***********************************************/
#define IBAT_BCM_REF    	  	10.0 						 /* battery charging current reference[A]*/
//PI regulator parameters
#define IBAT_BCM_KP           	5.0							 /* kp */
#define IBAT_BCM_KI           	0.05						 /* ki*T */
#define IBAT_BCM_UP_LIMIT     	CTRL_UP_LIMIT				 /* restricted the maximum allowable current loop result in current and voltage compare mode */
#define IBAT_BCM_LOW_LIMIT    	BCM_CTRL_BURST_OFF_HYS 		 /* restricted the minimum allowable current loop result in current and voltage compare mode */

/**************************************** Application time base ***********************************************/
#define SR_OFFDELAY   				  0ul 					 /* SR switch off delay time = SR_OFFDELAY * PWM CLOCK */
#define SR_ONDELAY    				  10ul 					 /* SR switch on delay time = SR_ONDELAY * PWM CLOCK */
/* time delay duration based on time base 100us */
#define SAMPOFFSET_CALIB_DURATION     2000  				 /* 200ms */
#define FAULT_RELEASE_DURATION		  30000					 /* 3s */  
#define LED_TOGGLE_DURATION     	  5000					 /* 0.5s */  

#endif /* BIDIR_DCDC_CTRL_H_ */
