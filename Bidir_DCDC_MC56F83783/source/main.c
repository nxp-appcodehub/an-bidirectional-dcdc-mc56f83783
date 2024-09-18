
/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2023-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"
#include "state_machine.h"
#include "hwcontrol.h"
#include "bidir_dcdc_ctrl.h"
#include "bidir_dcdc_statemachine.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
uint32_t gu32TimerCnt = 0;
uint16_t u16TempHvCnt = 0, u16TempLvCnt = 0;
uint16_t u16LedToggleCnt;
uint16_t u16CpuLoadCnt;

acc32_t a32Temp_NTC[13] = {ACC32(5.99), ACC32(8.42), ACC32(11.12), ACC32(13.87), ACC32(16.43), ACC32(18.66), ACC32(20.49), ACC32(21.94), ACC32(23.06), ACC32(23.92), ACC32(24.56), ACC32(25.05), ACC32(26.72)};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void Read_Temperature();
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */

int main(void)
{
    /* Init board hardware. */

	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    /* Add user initialization code */
    
    __EI(0);
    
    PIT0_RUN();				/* run the count of the PIT0 */
    
#if BOARD_TEST
	gsDCDC_Drive.gu16WorkModeUsed = BCM;
#endif
	
    while (1)
    {
    	/*==================== work mode change, restart from INIT state =======================*/
    	if((gsDCDC_Drive.gu16WorkModeCmd!=gsDCDC_Drive.gu16WorkModeUsed)\
    	    &&((gsDCDC_Drive.gu16WorkModeCmd == BCM)||(gsDCDC_Drive.gu16WorkModeCmd == BDM)))
    	{    		
    		gsDCDC_Drive.sFlag.RunState = false;
    		
    		DCDC_HV_PWM_DIS();
    		DCDC_LV_PWM_DIS();
    	    gsDCDC_Drive.sFlag.VarInitReady = false;
    	    gsDCDC_Ctrl.eState = INIT;
    	    
    	    if(gsDCDC_Drive.gu16WorkModeCmd == BCM)
    	    {
    	    	/* Re-init the DSC configuration for working at BCM */ 
    	    	BDM_Config_Pin_deinit();
    	    	BCM_Config_Pin();
    	    	BCM_Config_Peripheral();
    	    	
    	        __EI(0);  
    	        
    	        PWMA_SM0123_RUN();		/* run the count of the PWMA SM0-4, which are used for the CLLC driver */
    	        PWMB_SM1_RUN(); 		/* run the count of the PWMB SM1, which are used to generate the interrupt for control loop */
    	    }
    	    else if(gsDCDC_Drive.gu16WorkModeCmd == BDM)
    	    {
    	    	/* Re-init the DSC configuration for working at BDM */ 
    	    	BCM_Config_Pin_deinit();
    	    	BDM_Config_Pin();
    	    	BDM_Config_Peripheral();
    	    	
    	        __EI(0);
    
    	        PWMA_SM0123_RUN();		/* run the count of the PWMA SM0-4, which are used for CLLC driver */
    	        PWMB_SM1_RUN(); 		/* run the count of the PWMB SM1, which are used to generate the interrupt for control loop */
    	    } 
    	}
    	if((gsDCDC_Drive.gu16WorkModeCmd == BCM)||(gsDCDC_Drive.gu16WorkModeCmd == BDM)
    	    ||(gsDCDC_Drive.gu16WorkModeUsed == BCM)||(gsDCDC_Drive.gu16WorkModeUsed == BDM))
    	{
    	    SM_StateMachine(&gsDCDC_Ctrl);  /* run state machine only in either BCM or BDM */   	    
        	/* Read the temperature of the MOS on both high-voltage and low-voltage side */
        	Read_Temperature();
        	/* Calculate the power of the low-voltage and high-voltage port */
        	gsDCDC_Drive.sPowerMetering.f16Phv = MLIB_Mul_F16(gsDCDC_Drive.sVdcCtrl.f16VhvFilt,gsDCDC_Drive.sCurCtrl.f16IhvFilt);
        	gsDCDC_Drive.sPowerMetering.f16Plv = MLIB_Mul_F16(gsDCDC_Drive.sVdcCtrl.f16VlvFilt,gsDCDC_Drive.sCurCtrl.f16IlvFilt);
    	}
    	FMSTR_Poll();
    }
}

#pragma section CODES_IN_RAM begin
/* keFlexPWMB_CMP1_VECTORn interrupt handler */
#pragma interrupt alignsp saveall

void DCDC_Ctrl_ISR(void) 								/* 100kHz ISR for control loop execution */
{
	/* init the CPU loading counter */
	TMRB->CHANNEL[2].CNTR = 0;
		
	/*====================== Read ADC sampling result ====================*/		
	while(!(ADC->RDY & 0x8)) {;} 						/* Read ADC results after all sampling conversion is done */
	gsDCDC_Drive.sVdcCtrl.f16Vhv = ADC->RSLT[2];
	gsDCDC_Drive.sVdcCtrl.f16Vlv = ADC->RSLT[0];		/* low-voltage port refers to the low voltage port of the CLLC converter */
	gsDCDC_Drive.sCurCtrl.f16Ihv = -ADC->RSLT[10];
	gsDCDC_Drive.sCurCtrl.f16Ilv = ADC->RSLT[8]; 
	
	gsDCDC_Drive.sVdcCtrl.f16VhvFilt = GDFLIB_FilterIIR1_F16(gsDCDC_Drive.sVdcCtrl.f16Vhv, &gsDCDC_Drive.sVdcCtrl.sVhvFilter);
	gsDCDC_Drive.sVdcCtrl.f16VlvFilt = GDFLIB_FilterIIR1_F16(gsDCDC_Drive.sVdcCtrl.f16Vlv, &gsDCDC_Drive.sVdcCtrl.sVlvFilter);
	gsDCDC_Drive.sCurCtrl.f16IhvFilt = GDFLIB_FilterIIR1_F16(gsDCDC_Drive.sCurCtrl.f16Ihv, &gsDCDC_Drive.sCurCtrl.sIhvFilter);
	gsDCDC_Drive.sCurCtrl.f16IlvFilt = GDFLIB_FilterIIR1_F16(gsDCDC_Drive.sCurCtrl.f16Ilv, &gsDCDC_Drive.sCurCtrl.sIlvFilter);
	
	/* Get current offset */
	if(!gsDCDC_Drive.sFlag.SampOffsetReady)
    {
		gsDCDC_Drive.sCurCtrl.f16IhvOffset = GDFLIB_FilterMA_F16(-gsDCDC_Drive.sCurCtrl.f16IhvFilt, &gsDCDC_Drive.sCurCtrl.sIhvOffsetFilter);
		gsDCDC_Drive.sCurCtrl.f16IlvOffset = GDFLIB_FilterMA_F16(gsDCDC_Drive.sCurCtrl.f16IlvFilt, &gsDCDC_Drive.sCurCtrl.sIlvOffsetFilter);
    }
	
	if(gsDCDC_Drive.sFlag.RunState)		
	{
		if(eDCDC_Runsub == NORMAL)
		{
			if(gsDCDC_Drive.gu16WorkModeUsed == BCM)				/* BCM */
			{
#if BCM_MODE_OPTION == BCM_SINGLE_VOLT_PI_LOOP	
				gsDCDC_Drive.sVdcCtrl.f16VdcErr = MLIB_SubSat_F16(gsDCDC_Drive.sVdcCtrl.f16VdcRef,gsDCDC_Drive.sVdcCtrl.f16VlvFilt);
				gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut = GFLIB_CtrlPIpAW_F16(gsDCDC_Drive.sVdcCtrl.f16VdcErr, &gsDCDC_Drive.sVdcCtrl.StopIntegFlag ,&gsDCDC_Drive.sVdcCtrl.sPIpAWParams);
#elif BCM_MODE_OPTION == BCM_SINGLE_VOLT_2P2Z_LOOP
				gsDCDC_Drive.sVdcCtrl.f16VdcErr = MLIB_SubSat_F16(gsDCDC_Drive.sVdcCtrl.f16VdcRef,gsDCDC_Drive.sVdcCtrl.f16VlvFilt);	
				gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut = IIR_2P2Z_II_TRANS_LIM_output_mac_asm_sat_inline_F16(gsDCDC_Drive.sVdcCtrl.f16VdcErr, &gsDCDC_Drive.sVdcCtrl.sParam2p2z);
#elif BCM_MODE_OPTION == BCM_VOLT_CUR_COMP_LOOP				
				gsDCDC_Drive.sVdcCtrl.f16VdcErr = MLIB_SubSat_F16(gsDCDC_Drive.sVdcCtrl.f16VdcRef,gsDCDC_Drive.sVdcCtrl.f16VlvFilt);
				gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut = IIR_2P2Z_II_TRANS_LIM_output_mac_asm_sat_inline_F16(gsDCDC_Drive.sVdcCtrl.f16VdcErr, &gsDCDC_Drive.sVdcCtrl.sParam2p2z);
				
				gsDCDC_Drive.sCurCtrl.f16IdcErr = MLIB_SubSat_F16(gsDCDC_Drive.sCurCtrl.f16IdcRef,gsDCDC_Drive.sCurCtrl.f16IlvFilt);
				gsDCDC_Drive.sCurCtrl.f16IdcCtrlOut = GFLIB_CtrlPIpAW_F16(gsDCDC_Drive.sCurCtrl.f16IdcErr, &gsDCDC_Drive.sCurCtrl.StopIntegFlag ,&gsDCDC_Drive.sCurCtrl.sPIpAWParams);
				
				gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut = MLIB_AddSat_F16(gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut,gsDCDC_Drive.sCurCtrl.f16IdcCtrlOut);		/* VdcCtrlOut used as final controller output */				
#endif			
				
				/* Modulation strategy switching */
				if(gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut >= FRAC16(CTRL_PWM_PSM_BOUND))		/* PFM */
			    {
			    	gsDCDC_Drive.f16Period = gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut;
			    }
			    else																		/* PSM */
			    {
			    	gsDCDC_Drive.f16Period = FRAC16(CTRL_PWM_PSM_BOUND);
			    	gsDCDC_Drive.f16PSDuty = MLIB_SubSat_F16(FRAC16(CTRL_PWM_PSM_BOUND),gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut);
			    	
			    	/* Amplitude limit for the phase shift duty cycle */
			    	gsDCDC_Drive.f16PSDuty = GFLIB_LowerLimit_F16(gsDCDC_Drive.f16PSDuty,FRAC16(0.003));
			    }
				/* Burst on/off control, when fs is over the burst-off frequency, disable PWM output, when fs is below the burst-on frequency, enable PWM output */
				if(gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut >= FRAC16(BCM_CTRL_BURST_ON_HYS))
				{
					gsDCDC_Drive.sFlag.BurstOff = false;
					
					DCDC_HV_PWM_EN();
					DCDC_LV_PWM_EN();
					DCDC_PSM_PFM_UPDATE(gsDCDC_Drive.f16Period, gsDCDC_Drive.f16PSDuty);	
				}
				else if(gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut <= FRAC16(BCM_CTRL_BURST_OFF_HYS)) 
				{
					gsDCDC_Drive.sFlag.BurstOff = true;
					
					DCDC_HV_PWM_DIS();
					DCDC_LV_PWM_DIS();
				}					
			}
			else if(gsDCDC_Drive.gu16WorkModeUsed == BDM)				/* BDM */
			{
#if BDM_MODE_OPTION == BDM_SINGLE_VOLT_PI_LOOP	
				gsDCDC_Drive.sVdcCtrl.f16VdcErr = MLIB_SubSat_F16(gsDCDC_Drive.sVdcCtrl.f16VdcRef,gsDCDC_Drive.sVdcCtrl.f16VhvFilt);
				gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut = GFLIB_CtrlPIpAW_F16(gsDCDC_Drive.sVdcCtrl.f16VdcErr, &gsDCDC_Drive.sVdcCtrl.StopIntegFlag ,&gsDCDC_Drive.sVdcCtrl.sPIpAWParams);
#elif BDM_MODE_OPTION == BDM_SINGLE_VOLT_2P2Z_LOOP
				gsDCDC_Drive.sVdcCtrl.f16VdcErr = MLIB_SubSat_F16(gsDCDC_Drive.sVdcCtrl.f16VdcRef,gsDCDC_Drive.sVdcCtrl.f16VhvFilt);	
				gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut = IIR_2P2Z_II_TRANS_LIM_output_mac_asm_sat_inline_F16(gsDCDC_Drive.sVdcCtrl.f16VdcErr, &gsDCDC_Drive.sVdcCtrl.sParam2p2z);
#endif							
				/* Modulation strategy switching */
				if(gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut >= FRAC16(CTRL_PWM_PSM_BOUND))		/* PFM */
			    {
			    	gsDCDC_Drive.f16Period = gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut;
			    }
			    else																		/* PSM */
			    {
			    	gsDCDC_Drive.f16Period = FRAC16(CTRL_PWM_PSM_BOUND);
			    	gsDCDC_Drive.f16PSDuty = MLIB_SubSat_F16(FRAC16(CTRL_PWM_PSM_BOUND),gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut);
			    	
			    	/* Amplitude limit for the phase shift duty cycle */
			    	gsDCDC_Drive.f16PSDuty = GFLIB_LowerLimit_F16(gsDCDC_Drive.f16PSDuty,FRAC16(0.0));
			    }
				/* Burst on/off control, when fs is over the burst-off frequency, disable PWM output, when fs is below the burst-on frequency, enable PWM output */
				if(gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut >= FRAC16(BDM_CTRL_BURST_ON_HYS))
				{
					gsDCDC_Drive.sFlag.BurstOff = false;
					
					DCDC_HV_PWM_EN();
					DCDC_LV_PWM_EN();
					DCDC_PSM_PFM_UPDATE(gsDCDC_Drive.f16Period, gsDCDC_Drive.f16PSDuty);	
				}
				else if(gsDCDC_Drive.sVdcCtrl.f16VdcCtrlOut <= FRAC16(BDM_CTRL_BURST_OFF_HYS)) 
				{
					gsDCDC_Drive.sFlag.BurstOff = true;
					
					DCDC_HV_PWM_DIS();
					DCDC_LV_PWM_DIS();
				}			
			}		
		}
		else if(eDCDC_Runsub == SOFTSTART)
		{
#if (BCM_MODE_OPTION == BCM_OPEN_LOOP || BDM_MODE_OPTION == BDM_OPEN_LOOP)
		
	    	/* Increase the equivalent duty cycle to 0.5 firstly, then decrease the switching frequency to the open loop frequency */
		    if(gsDCDC_Drive.f16PSDuty != gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16Target)
		    {
		    	gsDCDC_Drive.f16PSDuty = GFLIB_Ramp_F16(gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16Target, &gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.sRamp);
		    }
		    else if(gsDCDC_Drive.f16Period != gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16Target) 
		    { 
		    	gsDCDC_Drive.f16Period = GFLIB_Ramp_F16(gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16Target, &gsDCDC_Drive.sVdcCtrl.sPeriodRamp.sRamp);
		    }
		    /* PWM update */
		    DCDC_PSM_PFM_UPDATE(gsDCDC_Drive.f16Period, gsDCDC_Drive.f16PSDuty);				
#else
		    if(gsDCDC_Drive.gu16WorkModeUsed == BCM)					/* BCM mode */
		    {
		    	/* if the output voltage reaches the soft-start end-point voltage, switches to the RUN NORMAL sub-state */
			    if(gsDCDC_Drive.sVdcCtrl.f16VlvFilt < gsDCDC_Drive.sVdcCtrl.f16VdcSoftEnd)
			    {
				    if(gsDCDC_Drive.f16PSDuty != gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16Target)
				    {
				    	gsDCDC_Drive.f16PSDuty = GFLIB_Ramp_F16(gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16Target, &gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.sRamp);
				    }
				    else 
				    {
				    	gsDCDC_Drive.f16Period = GFLIB_Ramp_F16(gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16Target, &gsDCDC_Drive.sVdcCtrl.sPeriodRamp.sRamp);
				    }
				    /* PWM update */
				    DCDC_PSM_PFM_UPDATE(gsDCDC_Drive.f16Period, gsDCDC_Drive.f16PSDuty);	
			    }
			    else
			    {
				    /* Transition to RUN NORMAL sub-state */
			    	eDCDC_Runsub = NORMAL;
/* SR logic */			    	
#if SR_ENABLE
	if(gsDCDC_Drive.gu16WorkModeUsed == BCM)
	{
		DCDC_BCM_SR_ENABLE();
	}
	else if(gsDCDC_Drive.gu16WorkModeUsed == BDM)
	{
//		DCDC_BDM_SR_ENABLE();
		DCDC_BDM_SR_DISABLE();					/* todo: The debug of the SR in BDM hasn't been done yet */
	}	
#else
	if(gsDCDC_Drive.gu16WorkModeUsed == BCM)
	{
		DCDC_BCM_SR_DISABLE();
	}
	else if(gsDCDC_Drive.gu16WorkModeUsed == BDM)
	{
		DCDC_BDM_SR_DISABLE();
	}
#endif
			    	/* Initialize the accumulator of the controller in respective control mode */
#if BCM_MODE_OPTION == BCM_SINGLE_VOLT_PI_LOOP	
					GFLIB_CtrlPIpAWInit_F16(gsDCDC_Drive.f16Period, &gsDCDC_Drive.sVdcCtrl.sPIpAWParams);
#elif BCM_MODE_OPTION == BCM_SINGLE_VOLT_2P2Z_LOOP					
					gsDCDC_Drive.sVdcCtrl.sParam2p2z.i32W1 = MLIB_Mul_F32(MLIB_Conv_F32s(gsDCDC_Drive.f16Period)>>(31-IFD),gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32A2)<<(31-CFD);
					gsDCDC_Drive.sVdcCtrl.sParam2p2z.i32W2 = MLIB_Conv_F32s(gsDCDC_Drive.f16Period)>>(31-IFD);					
#elif BCM_MODE_OPTION == BCM_VOLT_CUR_COMP_LOOP
					GFLIB_CtrlPIpAWInit_F16(gsDCDC_Drive.f16Period, &gsDCDC_Drive.sCurCtrl.sPIpAWParams);	
					
					gsDCDC_Drive.sVdcCtrl.sParam2p2z.i32W1 = MLIB_Mul_F32(MLIB_Conv_F32s(gsDCDC_Drive.f16Period)>>(31-IFD),gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32A2)<<(31-CFD);
					gsDCDC_Drive.sVdcCtrl.sParam2p2z.i32W2 = MLIB_Conv_F32s(gsDCDC_Drive.f16Period)>>(31-IFD);
#endif					
			    }
		    }
		    else if(gsDCDC_Drive.gu16WorkModeUsed == BDM)					/* BDM mode */
		    { 
		    	/* if the output voltage reaches the soft-start end-point voltage, switches to RUN NORMAL sub-state */
			    if(gsDCDC_Drive.sVdcCtrl.f16VhvFilt < gsDCDC_Drive.sVdcCtrl.f16VdcSoftEnd)
			    {
				    if(gsDCDC_Drive.f16PSDuty != gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16Target)
				    {
				    	gsDCDC_Drive.f16PSDuty = GFLIB_Ramp_F16(gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.f16Target, &gsDCDC_Drive.sVdcCtrl.sPSDutyRamp.sRamp);
				    }
				    else 
				    {
				    	gsDCDC_Drive.f16Period = GFLIB_Ramp_F16(gsDCDC_Drive.sVdcCtrl.sPeriodRamp.f16Target, &gsDCDC_Drive.sVdcCtrl.sPeriodRamp.sRamp);
				    }
				    /* PWM update */
				    DCDC_PSM_PFM_UPDATE(gsDCDC_Drive.f16Period, gsDCDC_Drive.f16PSDuty);	
			    }
			    else
			    {
				    /* Transition to RUN NORMAL sub-state */
			    	eDCDC_Runsub = NORMAL;			    	
			    	/* Initialize the accumulator of the controller in respective control mode */
#if BDM_MODE_OPTION == BDM_SINGLE_VOLT_PI_LOOP	
					GFLIB_CtrlPIpAWInit_F16(gsDCDC_Drive.f16Period, &gsDCDC_Drive.sVdcCtrl.sPIpAWParams);
#elif BDM_MODE_OPTION == BDM_SINGLE_VOLT_2P2Z_LOOP					
					gsDCDC_Drive.sVdcCtrl.sParam2p2z.i32W1 = MLIB_Mul_F32(MLIB_Conv_F32s(gsDCDC_Drive.f16Period)>>(31-IFD),gsDCDC_Drive.sVdcCtrl.sParam2p2z.sCoeff.i32A2)<<(31-CFD);
					gsDCDC_Drive.sVdcCtrl.sParam2p2z.i32W2 = MLIB_Conv_F32s(gsDCDC_Drive.f16Period)>>(31-IFD);					
#endif					
			    }
		    }
#endif 
		}
	}	
	/* store the CPU loading counter */
	u16CpuLoadCnt = TMRB->CHANNEL[2].CNTR;
	
  /* clear status interrupt flags */
	PWMB->SM[1].STS |= PWM_STS_CMPF(1);
}
#pragma interrupt off

/* kPIT0_ROLLOVR_VECTORn interrupt handler */
#pragma interrupt alignsp saveall
void PIT0_IRQHANDLER(void) 					/* 10kHz ISR */
{
	gu32TimerCnt++;							/* time base count */
	if(++u16LedToggleCnt >= LED_TOGGLE_DURATION)
	{
		u16LedToggleCnt = 0;
		HVP_LED_TOGGLE();					/* Toggle the red LED on daughter card every 0.5s */
	}	
	FMSTR_Recorder(0);						/* run the FreeMASTER recorder function */
	PIT0->CTRL &= ~PIT_CTRL_PRF_MASK;		/* clear interrupt flag */
}
#pragma interrupt off

#pragma section CODES_IN_RAM end

void Read_Temperature()						/* transfer the sampled temperature signal into the actual temperature value */
{
    frac16_t f16SlopeHv, f16SlopeLv;
    uint16_t TempHvCnt=0, TempLvCnt=0;
    
    u16TempHvCnt = TMRB->CHANNEL[0].CAPT;
    u16TempLvCnt = TMRB->CHANNEL[1].CAPT;
    /* High-voltage side temperature sensing */
    if(u16TempHvCnt != 0)
    {
    	gsDCDC_Drive.sPowerMetering.a32HvMosTempFreq = MLIB_Div1Q_A32as(50000,u16TempHvCnt);	//Frequency unit£ºkHz
    	if((gsDCDC_Drive.sPowerMetering.a32HvMosTempFreq>=a32Temp_NTC[0])&&(gsDCDC_Drive.sPowerMetering.a32HvMosTempFreq<=a32Temp_NTC[12]))
    	{
            for(TempHvCnt = 0; TempHvCnt < 13; TempHvCnt++)
            {
                if(gsDCDC_Drive.sPowerMetering.a32HvMosTempFreq <= a32Temp_NTC[TempHvCnt])
                {
                	f16SlopeHv = MLIB_Div1QSat_F16ll(MLIB_Sub_F32(gsDCDC_Drive.sPowerMetering.a32HvMosTempFreq,a32Temp_NTC[TempHvCnt-1]),
                    		MLIB_Sub_F32(a32Temp_NTC[TempHvCnt],a32Temp_NTC[TempHvCnt-1]));

                    gsDCDC_Drive.sPowerMetering.u16HvMosTemp = (TempHvCnt - 1)*10 + MLIB_Mul_F16(10,f16SlopeHv);
                    break;
                }
            }  
    	}
    	else
    		gsDCDC_Drive.sPowerMetering.u16HvMosTemp = -1;	
    }
    else
    	gsDCDC_Drive.sPowerMetering.u16HvMosTemp = -1;
    /* Low-voltage side temperature sensing */
    if(u16TempLvCnt != 0)
    {
    	gsDCDC_Drive.sPowerMetering.a32LvMosTempFreq = MLIB_Div1Q_A32as(50000,u16TempLvCnt);	//Frequency unit£ºkHz
    	if((gsDCDC_Drive.sPowerMetering.a32LvMosTempFreq>=a32Temp_NTC[0])&&(gsDCDC_Drive.sPowerMetering.a32LvMosTempFreq<=a32Temp_NTC[12]))
    	{
            for(TempLvCnt = 0; TempLvCnt < 13; TempLvCnt++)
            {
                if(gsDCDC_Drive.sPowerMetering.a32LvMosTempFreq <= a32Temp_NTC[TempLvCnt])
                {
                	f16SlopeLv = MLIB_Div1QSat_F16ll(MLIB_Sub_F32(gsDCDC_Drive.sPowerMetering.a32LvMosTempFreq,a32Temp_NTC[TempLvCnt-1]),
                    		MLIB_Sub_F32(a32Temp_NTC[TempLvCnt],a32Temp_NTC[TempLvCnt-1]));

                    gsDCDC_Drive.sPowerMetering.u16LvMosTemp = (TempLvCnt - 1)*10 + MLIB_Mul_F16(10,f16SlopeLv);
                    break;
                }
            }  
    	}
    	else
    		gsDCDC_Drive.sPowerMetering.u16LvMosTemp = -1;	
    }
    else
    	gsDCDC_Drive.sPowerMetering.u16LvMosTemp = -1;    
}
