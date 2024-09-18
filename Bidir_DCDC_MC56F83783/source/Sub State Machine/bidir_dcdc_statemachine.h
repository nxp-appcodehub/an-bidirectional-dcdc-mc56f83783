/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * bidir_dcdc_statemachine.h
 *
 *  Created on: Aug 2, 2023
 *      Author: nxf65232
 */

#ifndef BIDIR_DCDC_STATEMACHINE_H_
#define BIDIR_DCDC_STATEMACHINE_H_

#include "state_machine.h"
#include "bidir_dcdc_structure.h"
#include "bidir_dcdc_ctrl.h"

/******************************************************************************
* Types
******************************************************************************/
typedef enum {
	SOFTSTART       = 0,
	NORMAL          = 1,
	LIGHTLOAD       = 2
} DCDC_RUN_SUBSTATE_T;     /* Application Run sub-state identification enum */

/******************************************************************************
* Global variables
******************************************************************************/
extern SM_APP_CTRL_T        gsDCDC_Ctrl;
extern DCDC_STRUC_CTRL_T    gsDCDC_Drive;
extern DCDC_RUN_SUBSTATE_T  eDCDC_Runsub;
extern uint32_t  gu32TimerCnt, guw32StartCnt;
__pmem extern const PFCN_VOID_VOID mDCDC_STATE_RUN_TABLE[3];
 
extern bool_t    bDCDC_Run;

typedef struct
{
	uint32_t     *pui32PwmFrac1Val1;
	uint32_t     *pui32PwmFrac2Val2;
	uint32_t     *pui32PwmFrac3Val3;
} DCDC_DRV_PWMVAL;
extern DCDC_DRV_PWMVAL     gsDCDC_SM0PwmVal, gsDCDC_SM1PwmVal, gsDCDC_SM2PwmVal, gsDCDC_SM3PwmVal;

/******************************************************************************
* Global functions
******************************************************************************/

#endif /* BIDIR_DCDC_STATEMACHINE_H_ */
