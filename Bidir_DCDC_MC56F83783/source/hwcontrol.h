/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * hwcontrol.h
 *
 *  Created on: Aug 2, 2023
 *      Author: nxf65232
 */

#ifndef HWCONTROL_H_
#define HWCONTROL_H_

#include "fsl_device_registers.h"
#include "bidir_dcdc_statemachine.h"
#include "pin_mux.h"
#include "fsl_gpio.h"

#pragma inline_max_total_size(30000)
#pragma inline_max_size(3000)

/* Timer counter run/stop */
#define PWMA_SM0123_RUN()      			 PWMA->MCTRL |= PWM_MCTRL_RUN(0xF)		/* Enable the counter of the PWMA SM0-3 */
#define PWMA_SM0123_STOP()     			 PWMA->MCTRL &= ~PWM_MCTRL_RUN(0xF)		/* Disable the counter of the PWMA SM0-3 */

#define PWMB_SM1_RUN()         			 PWMB->MCTRL |= PWM_MCTRL_RUN(0x2)		/* Enable the counter of the PWMB SM1 */
#define PWMB_SM1_STOP()        			 PWMB->MCTRL &= ~PWM_MCTRL_RUN(0x2)		/* Disable the counter of the PWMB SM1 */

#define PIT0_RUN()      	   			 PIT0->CTRL |= PIT_CTRL_CNT_EN_MASK		/* Enable the counter of the PIT0 */
#define PIT0_STOP()      	   			 PIT0->CTRL &= ~PIT_CTRL_CNT_EN_MASK	/* Disable the counter of the PIT0 */

/* Hardware protection */
#define DCDC_HW_OVERCUR()             	 PWMA->FAULT[0].FSTS = ((PWMA->FAULT[0].FSTS & PWM_FSTS_FFLAG(1))|(PWMA->FAULT[0].FSTS & PWM_FSTS_FFPIN(1)))
#define DCDC_CLEAR_HWOVERCUR_FAULT()  	 PWMA->FAULT[0].FSTS |= PWM_FSTS_FFLAG(1)

#define DCDC_HV_PWM_DIS()     	 		 PWMA->OUTEN &= ~(PWM_OUTEN_PWMA_EN(3)|PWM_OUTEN_PWMB_EN(3))
#define DCDC_HV_PWM_EN()      			 PWMA->OUTEN |= (PWM_OUTEN_PWMA_EN(3)|PWM_OUTEN_PWMB_EN(3))
#define DCDC_LV_PWM_DIS()     	 		 PWMA->OUTEN &= ~(PWM_OUTEN_PWMA_EN(0xC)|PWM_OUTEN_PWMB_EN(0xC))
#define DCDC_LV_PWM_EN()      			 PWMA->OUTEN |= (PWM_OUTEN_PWMA_EN(0xC)|PWM_OUTEN_PWMB_EN(0xC))

#define DCDC_HV_PWM_MASK()  			 PWMA->MASK |= PWM_MASK_UPDATE_MASK(1)|(PWM_MASK_MASKA(3)|PWM_MASK_MASKB(3))
#define DCDC_HV_PWM_NOMASK()  			 PWMA->MASK = (PWMA->MASK & 0xFCCF)|PWM_MASK_UPDATE_MASK(1)
#define DCDC_LV_PWM_MASK()  			 PWMA->MASK |= PWM_MASK_UPDATE_MASK(1)|(PWM_MASK_MASKA(0xC)|PWM_MASK_MASKB(0xC))
#define DCDC_LV_PWM_NOMASK()  			 PWMA->MASK = (PWMA->MASK & 0xF33F)|PWM_MASK_UPDATE_MASK(1)

/* SR operation */
#define DCDC_BCM_SR_ENABLE()  			 {EVTG->EVTG_INST[1].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[1].EVTG_AOI1_BFT01 |= EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[2].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[2].EVTG_AOI1_BFT01 |= EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK;}

#define DCDC_BCM_SR_DISABLE()  			 {EVTG->EVTG_INST[1].EVTG_AOI0_BFT01 &= ~EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[1].EVTG_AOI1_BFT01 &= ~EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[2].EVTG_AOI0_BFT01 &= ~EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[2].EVTG_AOI1_BFT01 &= ~EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK;}

#define DCDC_BDM_SR_ENABLE()  			 {EVTG->EVTG_INST[1].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[1].EVTG_AOI1_BFT01 |= EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[2].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[2].EVTG_AOI1_BFT01 |= EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK;}

#define DCDC_BDM_SR_DISABLE()  			 {EVTG->EVTG_INST[1].EVTG_AOI0_BFT01 &= ~EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[1].EVTG_AOI1_BFT01 &= ~EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[2].EVTG_AOI0_BFT01 &= ~EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
										  EVTG->EVTG_INST[2].EVTG_AOI1_BFT01 &= ~EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK;}

/* Load switch (relay) */
#define CLOSE_LOAD_SW()      			 GPIO_PinSet(BOARD_Load_SW_GPIO,BOARD_Load_SW_PIN_MASK)    		/* GF7 */
#define OPEN_LOAD_SW()       			 GPIO_PinClear(BOARD_Load_SW_GPIO,BOARD_Load_SW_PIN_MASK)  		/* GF7 */

/* User LED */
#define USER_LED_TOGGLE()    			 GPIO_PinToggle(BOARD_User_LED_GPIO,BOARD_User_LED_PIN_MASK)	/* GC0 */
#define USER_LED1_TOGGLE()   			 GPIO_PinToggle(BOARD_User_LED1_GPIO,BOARD_User_LED1_PIN_MASK) 	/* GC8 */
#define HVP_LED_TOGGLE()     			 GPIO_PinToggle(BOARD_LED_HVP_GPIO,BOARD_LED_HVP_PIN_MASK)		/* GC1 */

/* Power board test point */
#define TP41_HIGH()      	 			 GPIO_PinSet(BOARD_XBAR_TP1_GPIO,BOARD_XBAR_TP1_PIN_MASK) 	    /* GC13 */
#define TP41_LOW()      	 			 GPIO_PinClear(BOARD_XBAR_TP1_GPIO,BOARD_XBAR_TP1_PIN_MASK) 	/* GC13 */
#define TP42_HIGH()      	 			 GPIO_PinSet(BOARD_XBAR_TP2_GPIO,BOARD_XBAR_TP2_PIN_MASK) 	    /* GC5 */
#define TP42_LOW()      	 			 GPIO_PinClear(BOARD_XBAR_TP2_GPIO,BOARD_XBAR_TP2_PIN_MASK) 	/* GC5 */
#define TP43_HIGH()      	 			 GPIO_PinSet(BOARD_XBAR_TP4_GPIO,BOARD_XBAR_TP4_PIN_MASK) 	    /* GC7 */
#define TP43_LOW()      	 			 GPIO_PinClear(BOARD_XBAR_TP4_GPIO,BOARD_XBAR_TP4_PIN_MASK) 	/* GC7 */
#define TP44_HIGH()      	 			 GPIO_PinSet(BOARD_XBAR_TP5_GPIO,BOARD_XBAR_TP5_PIN_MASK) 	    /* GC9 */
#define TP44_LOW()      	 			 GPIO_PinClear(BOARD_XBAR_TP5_GPIO,BOARD_XBAR_TP5_PIN_MASK) 	/* GC9 */
#define TP45_HIGH()      	 			 GPIO_PinSet(BOARD_XBAR_TP3_GPIO,BOARD_XBAR_TP3_PIN_MASK) 	    /* GF0 */
#define TP45_LOW()      	 			 GPIO_PinClear(BOARD_XBAR_TP3_GPIO,BOARD_XBAR_TP3_PIN_MASK) 	/* GF0 */

/* CAN TRANSRECEIVER STB CONTROL */
#define CAN_STB_TOGGLE()   	 			 GPIO_PinToggle(BOARD_STB_EN_GPIO,BOARD_STB_EN_PIN_MASK) 		/* GC15 */
#define CAN_STB_ENABLE()   	 			 GPIO_PinSet(BOARD_STB_EN_GPIO,BOARD_STB_EN_PIN_MASK)	   		/* GC15 */
#define CAN_STB_DISABLE()  	 			 GPIO_PinClear(BOARD_STB_EN_GPIO,BOARD_STB_EN_PIN_MASK)  		/* GC15 */

inline void DCDC_PSM_PFM_UPDATE(frac16_t f16Ts, frac16_t f16PSDuty)
{
	uint32_t u32Val1, u32Val2, u32Val3, u32HalfTs, u32QuartTs;
	uint16_t u16PhaseAngle;
	
	u32Val1 = MLIB_Mul_F32ss(f16Ts,CTRLOUT_TO_PERIOD_GAIN) - 1;	
	u32HalfTs = u32Val1>>1;
	u32QuartTs = u32HalfTs>>1;
	u16PhaseAngle = MLIB_Mul_F16(f16PSDuty,MLIB_Conv_F16l(u32Val1));
	
	u32Val2 = u32HalfTs - u32QuartTs;
	u32Val3 = u32HalfTs + u32QuartTs;
	
	/* BCM mode */
	if(gsDCDC_Drive.gu16WorkModeUsed == BCM)
	{
		PWMA->MCTRL |= PWM_MCTRL_CLDOK(0xF);
		//SM0
		*gsDCDC_SM0PwmVal.pui32PwmFrac1Val1 = u32Val1;
		*gsDCDC_SM0PwmVal.pui32PwmFrac2Val2 = u32Val2;
		*gsDCDC_SM0PwmVal.pui32PwmFrac3Val3 = u32Val3;
		//SM1
		*gsDCDC_SM1PwmVal.pui32PwmFrac1Val1 = u32Val1;
		*gsDCDC_SM1PwmVal.pui32PwmFrac2Val2 = u32Val3;
		*gsDCDC_SM1PwmVal.pui32PwmFrac3Val3 = u32Val2;
		PWMA->SM[1].PHASEDLY = u16PhaseAngle;
#if SR_ENABLE
		//SM2
		*gsDCDC_SM2PwmVal.pui32PwmFrac1Val1 = u32Val1;
		*gsDCDC_SM2PwmVal.pui32PwmFrac2Val2 = u32Val2 + (SR_ONDELAY<<16);	
		*gsDCDC_SM2PwmVal.pui32PwmFrac3Val3 = u32Val3 + (SR_OFFDELAY<<16);
		PWMA->SM[2].PHASEDLY = u16PhaseAngle;
		//SM3
		*gsDCDC_SM3PwmVal.pui32PwmFrac1Val1 = u32Val1;
		*gsDCDC_SM3PwmVal.pui32PwmFrac2Val2 = u32Val3 + (SR_ONDELAY<<16);	
		*gsDCDC_SM3PwmVal.pui32PwmFrac3Val3 = u32Val2 + (SR_OFFDELAY<<16);	
		PWMA->SM[3].PHASEDLY = u16PhaseAngle;
#endif
		PWMA->MCTRL |= PWM_MCTRL_LDOK(0xF);
	}
	/* BDM mode */
	else if(gsDCDC_Drive.gu16WorkModeUsed == BDM)
	{
		PWMA->MCTRL |= PWM_MCTRL_CLDOK(0xF);
		//SM0
		*gsDCDC_SM0PwmVal.pui32PwmFrac1Val1 = u32Val1;
		*gsDCDC_SM0PwmVal.pui32PwmFrac2Val2 = u32Val2 + (SR_ONDELAY<<16);
		*gsDCDC_SM0PwmVal.pui32PwmFrac3Val3 = u32Val3 + (SR_OFFDELAY<<16);	
		PWMA->SM[0].VAL5 = (uint16_t)(u32Val1>>16) - u16PhaseAngle;
#if SR_ENABLE		
		//SM1
		*gsDCDC_SM1PwmVal.pui32PwmFrac1Val1 = u32Val1;
		*gsDCDC_SM1PwmVal.pui32PwmFrac2Val2 = u32Val3 + (SR_ONDELAY<<16);
		*gsDCDC_SM1PwmVal.pui32PwmFrac3Val3 = u32Val2 + (SR_OFFDELAY<<16);	
#endif
		//SM2
		*gsDCDC_SM2PwmVal.pui32PwmFrac1Val1 = u32Val1;
		*gsDCDC_SM2PwmVal.pui32PwmFrac2Val2 = u32Val2;	
		*gsDCDC_SM2PwmVal.pui32PwmFrac3Val3 = u32Val3;
		//SM3
		*gsDCDC_SM3PwmVal.pui32PwmFrac1Val1 = u32Val1;
		*gsDCDC_SM3PwmVal.pui32PwmFrac2Val2 = u32Val3;	
		*gsDCDC_SM3PwmVal.pui32PwmFrac3Val3 = u32Val2;						

		PWMA->MCTRL |= PWM_MCTRL_LDOK(0xF);
	}
}

#endif /* HWCONTROL_H_ */
