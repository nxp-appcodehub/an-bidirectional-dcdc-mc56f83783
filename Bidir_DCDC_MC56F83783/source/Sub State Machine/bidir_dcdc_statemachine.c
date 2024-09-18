/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * BIDIRDCDC.C
 *
 *  Created on: Aug 2, 2023
 *      Author: nxf65232
 */

#include "bidir_dcdc_statemachine.h"
#include "hwcontrol.h"

/******************************************************************************
* Global variables
******************************************************************************/
DCDC_RUN_SUBSTATE_T   eDCDC_Runsub;
DCDC_STRUC_CTRL_T     gsDCDC_Drive;

bool_t      		  bDCDC_Run; 	  		/* DCDC run/stop command, 1 = Run, 0 = Stop */ 
DCDC_DRV_PWMVAL    	  gsDCDC_SM0PwmVal, gsDCDC_SM1PwmVal, gsDCDC_SM2PwmVal, gsDCDC_SM3PwmVal;
uint32_t    		  guw32StartCnt; 		/* time interval measurement counter */

/******************************************************************************
* Local variables
******************************************************************************/

/* initialize the scale for the sample value */
float 	   			  VhvScale, VlvScale, IhvScale, IlvScale, VdcScale, IdcScale, PhvScale, PlvScale;

/*------------------------------------
 * User state machine functions
 * ----------------------------------*/
static void DCDC_StateFault(void);
static void DCDC_StateInit(void);
static void DCDC_StateStop(void);
static void DCDC_StateRun(void);

/*------------------------------------
 * User state-transition functions
 * ----------------------------------*/
static void DCDC_TransFaultInit(void);
static void DCDC_TransInitFault(void);
static void DCDC_TransInitStop(void);
static void DCDC_TransStopFault(void);
static void DCDC_TransStopRun(void);
static void DCDC_TransRunFault(void);
static void DCDC_TransRunStop(void);

/* State machine functions field (in pmem) */
__pmem static const SM_APP_STATE_FCN_T msSTATE = {DCDC_StateFault, DCDC_StateInit, DCDC_StateStop, DCDC_StateRun};

/* State-transition functions field (in pmem) */
__pmem static const SM_APP_TRANS_FCN_T msTRANS = {DCDC_TransFaultInit, DCDC_TransInitFault, DCDC_TransInitStop, DCDC_TransStopFault, DCDC_TransStopRun, DCDC_TransRunFault, DCDC_TransRunStop};

/* State machine structure declaration and initialization */
SM_APP_CTRL_T gsDCDC_Ctrl = 
{
	/* gsM1_Ctrl.psState, User state functions  */
	&msSTATE,
 	
 	/* gsM1_Ctrl.psTrans, User state-transition functions */
 	&msTRANS,
 
  	/* gsM1_Ctrl.uiCtrl, Default no control command */
  	SM_CTRL_NONE,
  	
  	/* gsM1_Ctrl.eState, Default state after reset */
  	INIT 	
};

void DCDC_FaultDetection(void);

bool_t DCDC_TimeDelay(uint32_t uw32timerstartval, uint32_t uw32delaytime);

#pragma section CODES_IN_RAM begin
void DCDC_FaultDetection(void)
{
	/* Clearing actual faults before detecting them again  */	
	gsDCDC_Drive.sFaultIdPending.R = 0;
	DCDC_CLEAR_HWOVERCUR_FAULT();
	
	/* high voltage port over/under voltage protection */
	if(gsDCDC_Drive.sVdcCtrl.f16VhvFilt >= gsDCDC_Drive.sFaultThresholds.f16VhvOver)
	{
		gsDCDC_Drive.sFaultIdPending.B.VhvOver = 1;
	}
	else if((gsDCDC_Drive.sVdcCtrl.f16VhvFilt <= gsDCDC_Drive.sFaultThresholds.f16VhvUnder) && eDCDC_Runsub == NORMAL)
	{
		gsDCDC_Drive.sFaultIdPending.B.VhvUnder = 1;
	}
		
	/* high voltage port over current protection */
	if(MLIB_Abs_F16(gsDCDC_Drive.sCurCtrl.f16IhvFilt) >= gsDCDC_Drive.sFaultThresholds.f16IhvOver)
	{
		gsDCDC_Drive.sFaultIdPending.B.IhvOver = 1;
	}
	/* low voltage port over/under voltage protection */
	if(gsDCDC_Drive.sVdcCtrl.f16VlvFilt >= gsDCDC_Drive.sFaultThresholds.f16VlvOver)
	{
		gsDCDC_Drive.sFaultIdPending.B.VlvOver = 1;
	}
	else if((gsDCDC_Drive.sVdcCtrl.f16VlvFilt <= gsDCDC_Drive.sFaultThresholds.f16VlvUnder) && eDCDC_Runsub == NORMAL)
	{
		gsDCDC_Drive.sFaultIdPending.B.VlvUnder = 1;
	}		
	/* low voltage port over current protection */
	if(MLIB_Abs_F16(gsDCDC_Drive.sCurCtrl.f16IlvFilt) >= gsDCDC_Drive.sFaultThresholds.f16IlvOver)
	{
		gsDCDC_Drive.sFaultIdPending.B.IlvOver = 1;
	}
	/* over-temperature protection */
	// MOSFET at high-voltage side
	if(gsDCDC_Drive.sPowerMetering.u16HvMosTemp > HVMOS_TEMP_UP_LIMIT)
	{
		gsDCDC_Drive.sFaultIdPending.B.HvMosTempOver = 1;
	}
	// MOSFET at low-voltage side
	if(gsDCDC_Drive.sPowerMetering.u16LvMosTemp > LVMOS_TEMP_UP_LIMIT)
	{
		gsDCDC_Drive.sFaultIdPending.B.LvMosTempOver = 1;
	}	
	/* hardware over-current protection */
	if(DCDC_HW_OVERCUR()) 
	{
	    if(gsDCDC_Drive.gu16WorkModeUsed == BCM)
	    {
	    	gsDCDC_Drive.sFaultIdPending.B.HW_IlvOver = 1;
	    }
	    else if(gsDCDC_Drive.gu16WorkModeUsed == BDM)
	    {
	    	gsDCDC_Drive.sFaultIdPending.B.HW_IhvOver = 1;
	    }
	}	
	if(gsDCDC_Drive.sFaultIdPending.R > 0)
	{
		gsDCDC_Drive.sFaultId.R |= gsDCDC_Drive.sFaultIdPending.R;
		gsDCDC_Ctrl.uiCtrl |= SM_CTRL_FAULT;
	}		
}

/***************************************************************************//*!
*
* @brief   time delay function
*
* @param   uw16timerstartval - the counter value recorded at the start point of the delay
*          uw16delaytime - required delay time, counter difference 
*
* @return  bool_t - delay ready flag
*
******************************************************************************/
static bool_t DCDC_TimeDelay(uint32_t uw32timerstartval, uint32_t uw32delaytime)
{
	uint32_t uw32delta_t;
	
	if(uw32timerstartval > gu32TimerCnt)
	{
	 	uw32delta_t = gu32TimerCnt + 0xFFFFFFFF - uw32timerstartval;
	}
	else
	{
	 	uw32delta_t = gu32TimerCnt - uw32timerstartval;
	}
	if(uw32delta_t >= uw32delaytime)   return 1;
	else                               return 0;
}
#pragma section CODES_IN_RAM end
/***************************************************************************//*!
*
* @brief   FAULT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void DCDC_StateFault(void)
{
	DCDC_HV_PWM_DIS();
	DCDC_LV_PWM_DIS();
	DCDC_FaultDetection();
	
	if(!gsDCDC_Drive.sFaultIdPending.R) // no fault detected for a certain time interval, recover from INIT state
	{
		if(DCDC_TimeDelay(guw32StartCnt, FAULT_RELEASE_DURATION))
		{
			//gsDCDC_Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR;
		}
	}
	else guw32StartCnt = gu32TimerCnt;	
}

/***************************************************************************//*!
*
* @brief   INIT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void DCDC_StateInit(void)
{
	if((gsDCDC_Drive.gu16WorkModeCmd == BCM)||(gsDCDC_Drive.gu16WorkModeCmd == BDM))
	{
	    gsDCDC_Drive.gu16WorkModeUsed = gsDCDC_Drive.gu16WorkModeCmd;
	}	
	if(!gsDCDC_Drive.sFlag.VarInitReady)
	{
		bDCDC_Run = STOP;		
		eDCDC_Runsub = SOFTSTART;
		/****** PWM reset, ensure start from a small duty cycle to avoid current spike ******/	
		PWMA->MCTRL |= PWM_MCTRL_CLDOK(0xF);  
		PWMA->SM[0].VAL2 = 0;
		PWMA->SM[0].VAL3 = 4;
		PWMA->SM[1].VAL2 = 0;
		PWMA->SM[1].VAL3 = 4;
		PWMA->SM[2].VAL2 = 0;			
		PWMA->SM[2].VAL3 = 4;
		PWMA->SM[3].VAL2 = 0;		
		PWMA->SM[3].VAL3 = 4;
		PWMA->MCTRL |= PWM_MCTRL_LDOK(0xF);			
		/***************** fault state initialisation *****************/
		gsDCDC_Drive.sFaultId.R = 0;
		gsDCDC_Drive.sFaultIdPending.R = 0;
		DCDC_CLEAR_HWOVERCUR_FAULT();
		
		gsDCDC_Drive.sFaultThresholds.f16VhvOver = FRAC16(VHV_UP_LIMIT/VHV_SCALE);
		gsDCDC_Drive.sFaultThresholds.f16VhvUnder = FRAC16(VHV_LOW_LIMIT/VHV_SCALE);
		gsDCDC_Drive.sFaultThresholds.f16VlvOver = FRAC16(VLV_UP_LIMIT/VLV_SCALE);
		gsDCDC_Drive.sFaultThresholds.f16VlvUnder = FRAC16(VLV_LOW_LIMIT/VLV_SCALE);
		gsDCDC_Drive.sFaultThresholds.f16VbatOver = FRAC16(VBAT_UP_LIMIT/VBAT_SCALE);
		gsDCDC_Drive.sFaultThresholds.f16VbatUnder = FRAC16(VBAT_LOW_LIMIT/VBAT_SCALE);
		gsDCDC_Drive.sFaultThresholds.f16IbatOver = FRAC16(IBAT_UP_LIMIT/IBAT_SCALE);
	    /****************** flag initialisation ******************/
	    gsDCDC_Drive.sFlag.BurstOff = false;
		gsDCDC_Drive.sFlag.RelayOn = false;
		gsDCDC_Drive.sFlag.SampOffsetReady = false;
		gsDCDC_Drive.sFlag.RunState = false;	    
	    /****************** vol&cur scale initialisation ******************/
	    VhvScale=VHV_SCALE; 
	    VlvScale=VLV_SCALE;
	    IhvScale=IHV_SCALE;
	    IlvScale=ILV_SCALE;
	    PhvScale=VHV_SCALE*IHV_SCALE;
	    PlvScale=VLV_SCALE*ILV_SCALE;
		/******************************** get PWM value register address ****************************/
		gsDCDC_SM0PwmVal.pui32PwmFrac1Val1 = (uint32_t *)(&PWMA->SM[0].FRACVAL1);
		gsDCDC_SM0PwmVal.pui32PwmFrac2Val2 = (uint32_t *)(&PWMA->SM[0].FRACVAL2);
		gsDCDC_SM0PwmVal.pui32PwmFrac3Val3 = (uint32_t *)(&PWMA->SM[0].FRACVAL3);
		
		gsDCDC_SM1PwmVal.pui32PwmFrac1Val1 = (uint32_t *)(&PWMA->SM[1].FRACVAL1);
		gsDCDC_SM1PwmVal.pui32PwmFrac2Val2 = (uint32_t *)(&PWMA->SM[1].FRACVAL2);
		gsDCDC_SM1PwmVal.pui32PwmFrac3Val3 = (uint32_t *)(&PWMA->SM[1].FRACVAL3);
		
		gsDCDC_SM2PwmVal.pui32PwmFrac1Val1 = (uint32_t *)(&PWMA->SM[2].FRACVAL1);
		gsDCDC_SM2PwmVal.pui32PwmFrac2Val2 = (uint32_t *)(&PWMA->SM[2].FRACVAL2);
		gsDCDC_SM2PwmVal.pui32PwmFrac3Val3 = (uint32_t *)(&PWMA->SM[2].FRACVAL3);
		
		gsDCDC_SM3PwmVal.pui32PwmFrac1Val1 = (uint32_t *)(&PWMA->SM[3].FRACVAL1);
		gsDCDC_SM3PwmVal.pui32PwmFrac2Val2 = (uint32_t *)(&PWMA->SM[3].FRACVAL2);
		gsDCDC_SM3PwmVal.pui32PwmFrac3Val3 = (uint32_t *)(&PWMA->SM[3].FRACVAL3);
		
		/******************************** vol&cur filter parameters ****************************/	    
	    /* initialize high voltage dc voltage IIR1 filter */
	    gsDCDC_Drive.sVdcCtrl.sVhvFilter.sFltCoeff.f32A1 = DCDC_VHV_IIR_A1;
	    gsDCDC_Drive.sVdcCtrl.sVhvFilter.sFltCoeff.f32B0 = DCDC_VHV_IIR_B0;
	    gsDCDC_Drive.sVdcCtrl.sVhvFilter.sFltCoeff.f32B1 = DCDC_VHV_IIR_B1;
	    GDFLIB_FilterIIR1Init_F16(&gsDCDC_Drive.sVdcCtrl.sVhvFilter);
	    /* initialize low voltage dc voltage IIR1 filter */
	    gsDCDC_Drive.sVdcCtrl.sVlvFilter.sFltCoeff.f32A1 = DCDC_VLV_IIR_A1;
	    gsDCDC_Drive.sVdcCtrl.sVlvFilter.sFltCoeff.f32B0 = DCDC_VLV_IIR_B0;
	    gsDCDC_Drive.sVdcCtrl.sVlvFilter.sFltCoeff.f32B1 = DCDC_VLV_IIR_B1;
	    GDFLIB_FilterIIR1Init_F16(&gsDCDC_Drive.sVdcCtrl.sVlvFilter);
	    /* initialize high voltage dc current IIR1 filter */
	    gsDCDC_Drive.sCurCtrl.sIhvFilter.sFltCoeff.f32A1 = DCDC_IHV_IIR_A1;
	    gsDCDC_Drive.sCurCtrl.sIhvFilter.sFltCoeff.f32B0 = DCDC_IHV_IIR_B0;
	    gsDCDC_Drive.sCurCtrl.sIhvFilter.sFltCoeff.f32B1 = DCDC_IHV_IIR_B1;
	    GDFLIB_FilterIIR1Init_F16(&gsDCDC_Drive.sCurCtrl.sIhvFilter);
	    /* initialize low voltage dc current IIR1 filter */
	    gsDCDC_Drive.sCurCtrl.sIlvFilter.sFltCoeff.f32A1 = DCDC_ILV_IIR_A1;
	    gsDCDC_Drive.sCurCtrl.sIlvFilter.sFltCoeff.f32B0 = DCDC_ILV_IIR_B0;
	    gsDCDC_Drive.sCurCtrl.sIlvFilter.sFltCoeff.f32B1 = DCDC_ILV_IIR_B1;
	    GDFLIB_FilterIIR1Init_F16(&gsDCDC_Drive.sCurCtrl.sIlvFilter);
	    /* initialize current offset MA filter */
	    gsDCDC_Drive.sCurCtrl.sIhvOffsetFilter.u16Sh = DCDC_SAMP_OFFSET_MA_WINDOW;
	    gsDCDC_Drive.sCurCtrl.sIhvOffsetFilter.a32Acc = 0;	    
	    gsDCDC_Drive.sCurCtrl.sIlvOffsetFilter.u16Sh = DCDC_SAMP_OFFSET_MA_WINDOW;
	    gsDCDC_Drive.sCurCtrl.sIlvOffsetFilter.a32Acc = 0;	
	    	    
	    if(gsDCDC_Drive.gu16WorkModeUsed == BCM)
	    {
	    	/* Disable the SR */
	    	DCDC_BCM_SR_DISABLE();
			/***************** fault state initialisation *****************/
			gsDCDC_Drive.sFaultThresholds.f16IhvOver = FRAC16(IHV_UP_LIMIT_BCM/IHV_SCALE);
			gsDCDC_Drive.sFaultThresholds.f16IlvOver = FRAC16(ILV_UP_LIMIT_BCM/ILV_SCALE);
	    	/******************************** Soft-start parameters ****************************/
	    	gsDCDC_Drive.sVdcCtrl.f16VdcSoftEnd = FRAC16(VLV_BCM_SOFT_END/VLV_SCALE);
	    	
			gsDCDC_Drive.f16PSDuty = FRAC16(BCM_SOFT_START_INIT_DUTY);
			gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16InitVal = FRAC16(BCM_SOFT_START_INIT_DUTY);
			gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.sRamp.f16RampUp = FRAC16(BCM_SOFT_START_DUTY_STEP);
			gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.sRamp.f16RampDown = FRAC16(BCM_SOFT_START_DUTY_STEP);
			gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16Target = FRAC16(0.004);	
			GFLIB_RampInit_F16(gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16InitVal, &gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.sRamp);
			
			gsDCDC_Drive.f16Period = FRAC16(100/BCM_FREQ_SOFT_START);
			gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16InitVal = FRAC16(100/BCM_FREQ_SOFT_START);
			gsDCDC_Drive.sVdcCtrl.sPeriodRamp.sRamp.f16RampUp = FRAC16(BCM_SOFT_START_PERIOD_STEP);
			gsDCDC_Drive.sVdcCtrl.sPeriodRamp.sRamp.f16RampDown = FRAC16(BCM_SOFT_START_PERIOD_STEP);
			gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16Target = FRAC16(CTRL_UP_LIMIT);
			GFLIB_RampInit_F16(gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16InitVal, &gsDCDC_Drive.sVdcCtrl.sPeriodRamp.sRamp);
	    	
	    	/******************************* Vlv control parameters *******************************/
	    	gsDCDC_Drive.sVdcCtrl.f16VdcRef = FRAC16(VLV_BCM_REF/VLV_SCALE);
	    	/* PI controller parameters */
	    	gsDCDC_Drive.sVdcCtrl.sPIpAWParams.a32PGain = ACC32(VLV_BCM_KP);
	    	gsDCDC_Drive.sVdcCtrl.sPIpAWParams.a32IGain = ACC32(VLV_BCM_KI);
		    gsDCDC_Drive.sVdcCtrl.sPIpAWParams.f16LowerLim = FRAC16(VLV_BCM_LOW_LIMIT);
		    gsDCDC_Drive.sVdcCtrl.sPIpAWParams.f16UpperLim = FRAC16(VLV_BCM_UP_LIMIT);
	    	gsDCDC_Drive.sVdcCtrl.StopIntegFlag = false;
	    	GFLIB_CtrlPIpAWInit_F16(0, &gsDCDC_Drive.sVdcCtrl.sPIpAWParams);
	    	/* 2P2Z controller parameters */
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32A1 = FRAC_DYN(CFD,VLV_BCM_2P2Z_A1);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32A2 = FRAC_DYN(CFD,VLV_BCM_2P2Z_A2);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32B0 = FRAC_DYN(CFD,VLV_BCM_2P2Z_B0);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32B1 = FRAC_DYN(CFD,VLV_BCM_2P2Z_B1);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32B2 = FRAC_DYN(CFD,VLV_BCM_2P2Z_B2);
#if BCM_MODE_OPTION == BCM_SINGLE_VOLT_2P2Z_LOOP
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sLim.i32LowerLim = FRAC_DYN(IFD,VLV_BCM_LOW_LIMIT);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sLim.i32UpperLim = FRAC_DYN(IFD,VLV_BCM_UP_LIMIT);
#elif BCM_MODE_OPTION == BCM_VOLT_CUR_COMP_LOOP	
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sLim.i32LowerLim = FRAC_DYN(IFD,VLV_BCM_COMP_LOW_LIMIT);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sLim.i32UpperLim = FRAC_DYN(IFD,VLV_BCM_COMP_UP_LIMIT);
#endif	
	    	IIR_2P2Z_II_TRANS_LIM_output_mac_asm_inline_init_F16(&gsDCDC_Drive.sVdcCtrl.sParam2p2z);
	    	
	    	/******************************* Ilv control parameters *******************************/
	    	gsDCDC_Drive.sCurCtrl.f16IdcRef = FRAC16(IBAT_BCM_REF/ILV_SCALE);
	    	/* PI controller parameters */
	    	gsDCDC_Drive.sCurCtrl.sPIpAWParams.a32PGain = ACC32(IBAT_BCM_KP);
	    	gsDCDC_Drive.sCurCtrl.sPIpAWParams.a32IGain = ACC32(IBAT_BCM_KI);	    	
	    	gsDCDC_Drive.sCurCtrl.sPIpAWParams.f16LowerLim = FRAC16(IBAT_BCM_LOW_LIMIT);
	    	gsDCDC_Drive.sCurCtrl.sPIpAWParams.f16UpperLim = FRAC16(IBAT_BCM_UP_LIMIT);
	    	gsDCDC_Drive.sCurCtrl.StopIntegFlag = false;
	    	GFLIB_CtrlPIpAWInit_F16(0, &gsDCDC_Drive.sCurCtrl.sPIpAWParams);
	    		    	
	    	VdcScale = VLV_SCALE;
	    	IdcScale = ILV_SCALE;
	    }
	    else if(gsDCDC_Drive.gu16WorkModeUsed == BDM)
	    {
	    	/* Disable the SR */
	    	DCDC_BDM_SR_DISABLE();
			/***************** fault state initialisation *****************/
			gsDCDC_Drive.sFaultThresholds.f16IhvOver = FRAC16(IHV_UP_LIMIT_BDM/IHV_SCALE);
			gsDCDC_Drive.sFaultThresholds.f16IlvOver = FRAC16(ILV_UP_LIMIT_BDM/ILV_SCALE);
	    	/******************************** Soft-start parameters ****************************/
	    	gsDCDC_Drive.sVdcCtrl.f16VdcSoftEnd = FRAC16(VHV_BDM_SOFT_END/VHV_SCALE);
	    	
			gsDCDC_Drive.f16PSDuty = FRAC16(BDM_SOFT_START_INIT_DUTY);
			gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16InitVal = FRAC16(BDM_SOFT_START_INIT_DUTY);
			gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.sRamp.f16RampUp = FRAC16(BDM_SOFT_START_DUTY_STEP);
			gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.sRamp.f16RampDown = FRAC16(BDM_SOFT_START_DUTY_STEP);
			gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16Target = FRAC16(0.004);	
			GFLIB_RampInit_F16(gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16InitVal, &gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.sRamp);
			
			gsDCDC_Drive.f16Period = FRAC16(100/BDM_FREQ_SOFT_START);
			gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16InitVal = FRAC16(100/BDM_FREQ_SOFT_START);
			gsDCDC_Drive.sVdcCtrl.sPeriodRamp.sRamp.f16RampUp = FRAC16(BDM_SOFT_START_PERIOD_STEP);
			gsDCDC_Drive.sVdcCtrl.sPeriodRamp.sRamp.f16RampDown = FRAC16(BDM_SOFT_START_PERIOD_STEP);
			gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16Target = FRAC16(CTRL_UP_LIMIT);
			GFLIB_RampInit_F16(gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16InitVal, &gsDCDC_Drive.sVdcCtrl.sPeriodRamp.sRamp);
	    	/******************************* Vhv control parameters *******************************/
	    	gsDCDC_Drive.sVdcCtrl.f16VdcRef = FRAC16(VHV_BDM_REF/VHV_SCALE);
	    	/* PI controller parameters */
	    	gsDCDC_Drive.sVdcCtrl.sPIpAWParams.a32PGain = ACC32(VHV_BDM_KP);
	    	gsDCDC_Drive.sVdcCtrl.sPIpAWParams.a32IGain = ACC32(VHV_BDM_KI);
	    	gsDCDC_Drive.sVdcCtrl.sPIpAWParams.f16LowerLim = FRAC16(VHV_BDM_LOW_LIMIT);
	    	gsDCDC_Drive.sVdcCtrl.sPIpAWParams.f16UpperLim = FRAC16(VHV_BDM_UP_LIMIT);
	    	gsDCDC_Drive.sVdcCtrl.StopIntegFlag = false;
	    	GFLIB_CtrlPIpAWInit_F16(0, &gsDCDC_Drive.sVdcCtrl.sPIpAWParams); 
	    	/* 2P2Z controller parameters */	    	
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32A1 = FRAC_DYN(CFD,VHV_BDM_2P2Z_A1);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32A2 = FRAC_DYN(CFD,VHV_BDM_2P2Z_A2);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32B0 = FRAC_DYN(CFD,VHV_BDM_2P2Z_B0);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32B1 = FRAC_DYN(CFD,VHV_BDM_2P2Z_B1);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32B2 = FRAC_DYN(CFD,VHV_BDM_2P2Z_B2);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sLim.i32LowerLim = FRAC_DYN(IFD,VHV_BDM_LOW_LIMIT);
	    	gsDCDC_Drive.sVdcCtrl.sParam2p2z.sLim.i32UpperLim = FRAC_DYN(IFD,VHV_BDM_UP_LIMIT);
	    	IIR_2P2Z_II_TRANS_LIM_output_mac_asm_inline_init_F16(&gsDCDC_Drive.sVdcCtrl.sParam2p2z);
	    		    	
	    	VdcScale = VHV_SCALE;
	    	IdcScale = IHV_SCALE;
	    }
	    gsDCDC_Drive.sFlag.VarInitReady = true;
	    guw32StartCnt = gu32TimerCnt;
	}
	/********************************* sample offset calibration *********************************/
	if(DCDC_TimeDelay(guw32StartCnt, SAMPOFFSET_CALIB_DURATION))
	{		
		ADC->OFFST[10] = MLIB_Add_F16(ADC->OFFST[10],gsDCDC_Drive.sCurCtrl.f16IhvOffset);
		ADC->OFFST[8] = MLIB_Add_F16(ADC->OFFST[8],gsDCDC_Drive.sCurCtrl.f16IlvOffset);			
		gsDCDC_Drive.sFlag.SampOffsetReady = true;
		gsDCDC_Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;
	}
       
	DCDC_FaultDetection();
}
/***************************************************************************//*!
*
* @brief   STOP state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void DCDC_StateStop(void)
{
#if BOARD_TEST
	gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16Target = FRAC16(100/FREQ_OPEN_LOOP);
	bDCDC_Run = STOP;
#endif
	/* software protection */
	DCDC_FaultDetection();
		
    if(bDCDC_Run == RUN) 	gsDCDC_Ctrl.uiCtrl |= SM_CTRL_START;
}
/***************************************************************************//*!
*
* @brief   RUN state
*
* @param   void
*
* @return  none
*
******************************************************************************/
#pragma section CODES_IN_RAM begin
static void DCDC_StateRun(void)
{
	DCDC_FaultDetection();
	
	if(bDCDC_Run == STOP)  gsDCDC_Ctrl.uiCtrl |= SM_CTRL_STOP;  
}
#pragma section CODES_IN_RAM end
/***************************************************************************//*!
*
* @brief   FAULT to INIT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void DCDC_TransFaultInit(void)
{
	gsDCDC_Drive.sFlag.VarInitReady = 0;
}
/***************************************************************************//*!
*
* @brief   INIT to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void DCDC_TransInitFault(void)
{
	guw32StartCnt = gu32TimerCnt;
}
/***************************************************************************//*!
*
* @brief   INIT to STOP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void DCDC_TransInitStop(void)
{
	
}
/***************************************************************************//*!
*
* @brief   STOP to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void DCDC_TransStopFault(void)
{
	guw32StartCnt = gu32TimerCnt;
}
/***************************************************************************//*!
*
* @brief   STOP to RUN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void DCDC_TransStopRun(void)
{	
#if (BCM_MODE_OPTION == BCM_OPEN_LOOP || BDM_MODE_OPTION == BDM_OPEN_LOOP)
	gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16Target = FRAC16(100/FREQ_OPEN_LOOP);
#else
	gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16Target = FRAC16(CTRL_UP_LIMIT);
#endif	
	
	/* begin to run the control loop */
	gsDCDC_Drive.sFlag.RunState = 1;
	
	/* enable the output of the PWM */	
	DCDC_HV_PWM_EN();
	DCDC_LV_PWM_EN();
	
	gsDCDC_Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
}
/***************************************************************************//*!
*
* @brief   RUN to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void DCDC_TransRunFault(void)
{
	DCDC_HV_PWM_DIS();
	DCDC_LV_PWM_DIS();
	gsDCDC_Drive.sFlag.RunState = 0;		/* stop the control loop */
	OPEN_LOAD_SW();
	gsDCDC_Drive.sFlag.RelayOn = 0;
	
	guw32StartCnt = gu32TimerCnt;
	
}
/***************************************************************************//*!
*
* @brief   RUN to STOP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void DCDC_TransRunStop(void)
{
	gsDCDC_Drive.sFlag.RunState = 0;		/* stop the control loop */
	
	OPEN_LOAD_SW();
	gsDCDC_Drive.sFlag.RelayOn = 0;
	
	/* disable the output of the PWM */
	DCDC_HV_PWM_DIS();
	DCDC_LV_PWM_DIS();
	/****** PWM reset, ensure start from a small duty cycle to avoid current spike ******/	
	PWMA->MCTRL |= PWM_MCTRL_CLDOK(0xF);  
	PWMA->SM[0].VAL2 = 0;
	PWMA->SM[0].VAL3 = 4;
	PWMA->SM[1].VAL2 = 0;
	PWMA->SM[1].VAL3 = 4;
	PWMA->SM[2].VAL2 = 0;		
	PWMA->SM[2].VAL3 = 4;
	PWMA->SM[3].VAL2 = 0;		
	PWMA->SM[3].VAL3 = 4;
	PWMA->MCTRL |= PWM_MCTRL_LDOK(0xF);
	
	/* initialize for soft-start */
	if(gsDCDC_Drive.gu16WorkModeUsed == BCM)
	{
		gsDCDC_Drive.f16PSDuty = FRAC16(BCM_SOFT_START_INIT_DUTY); 
		gsDCDC_Drive.f16Period = FRAC16(100/BCM_FREQ_SOFT_START);
#if SR_ENABLE
		/* Disable the SR */
		DCDC_BCM_SR_DISABLE();
#endif
	}
	else if(gsDCDC_Drive.gu16WorkModeUsed == BDM)
	{
		gsDCDC_Drive.f16PSDuty = FRAC16(BDM_SOFT_START_INIT_DUTY); 
		gsDCDC_Drive.f16Period = FRAC16(100/BDM_FREQ_SOFT_START);
#if SR_ENABLE	
		/* Disable the SR */
		DCDC_BDM_SR_DISABLE();
#endif
	}	
	GFLIB_RampInit_F16(gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16InitVal, &gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.sRamp);
	GFLIB_RampInit_F16(gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16InitVal, &gsDCDC_Drive.sVdcCtrl.sPeriodRamp.sRamp);	
	eDCDC_Runsub = SOFTSTART;
	
	gsDCDC_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;
}


