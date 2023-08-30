/*
 * si70cc.c
 * Copyright (c) 2022 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_config.h"

#if (halHAS_SI70XX > 0)
#include "si70xx.h"
#include "endpoints.h"
#include "options.h"
#include "printfx.h"
#include "rules.h"
#include "syslog.h"
#include "systiming.h"					// timing debugging
#include "x_errors_events.h"

#define	debugFLAG					0xF000

#define	debugDEVICE					(debugFLAG & 0x0001)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ############################################# Macros ############################################

#define	si70xxI2C_LOGIC				1					// 1=delay 2=stretch, 3=stages

// #################################### SI7006/13/20/21 Addresses ##################################

#define	si70xxADDR0					0x40				// SI702x, SI700x address
#define	si70xxADDR1					0x41				// SI701X address

#define	si70xxMODE_H12T14			0					// 12 + 11
#define	si70xxMODE_H08T12			1					// 4 + 4
#define	si70xxMODE_H10T13			2					// 5 + 7
#define	si70xxMODE_H11T11			3					// 7 + 3

#define	SI70XX_T_SNS				60000
#define SI70XX_MS_tPU				80					// Tpowerup worst case


// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################

u8_t const si70xxMRH_HMM 	= 0xE5;
u8_t const si70xxMRH_NHMM	= 0xF5;
u8_t const si70xxMT_HMM		= 0xE3;
u8_t const si70xxMT_NHMM	= 0xF3;
u8_t const si70xxRT_PMRH	= 0xE0;
u8_t const si70xxRESET		= 0xFE;
u8_t const si70xxWUR1		= 0xE6;
u8_t const si70xxRUR1		= 0xE7;
u8_t const si70xxWHCR		= 0x51;
u8_t const si70xxRHCR		= 0x11;
u8_t const si70xxREID1[2]	= { 0xFA, 0x0F };
u8_t const si70xxREID2[2]	= { 0xFC, 0xC9 };
u8_t const si70xxRFWR[2]	= { 0x84, 0xB8 };

const u8_t si70xxDelayRH[4] = { 12+11, 4+4, 5+7, 7+3};
const u8_t si70xxDelayT[4] = { 11, 4, 7, 3};

// ###################################### Local variables ##########################################

si70xx_t sSI70XX = { 0 };
u8_t si70xxNumDev;

// #################################### Local ONLY functions #######################################

static int si70xxWrite(const u8_t * pTxBuf, size_t TxLen) {
	return halI2CM_Queue(sSI70XX.psI2C, i2cW_B, (u8_t *) pTxBuf, TxLen,
			NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

static int si70xxWriteReg(u8_t Reg, u8_t Val) {
	u8_t caBuf[2];
	caBuf[0] = Reg;
	caBuf[1] = Val;
	return si70xxWrite(caBuf, sizeof(caBuf));
}

static int si70xxWriteRead(const u8_t * pTxBuf, size_t TxLen, u8_t * pRxBuf, size_t RxLen) {
	return halI2CM_Queue(sSI70XX.psI2C, i2cWR_B, (u8_t *) pTxBuf, TxLen,
			pRxBuf, RxLen, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

int si70xxModeSet(int Mode) {
	IF_myASSERT(debugPARAM, INRANGE(si70xxMODE_H12T14, Mode, si70xxMODE_H11T11));
	sSI70XX.sUR1.cfg1 = (Mode & 2) ? 1 : 0;
	sSI70XX.sUR1.cfg0 = (Mode & 1) ? 1 : 0;
	return si70xxWriteReg(si70xxWUR1, sSI70XX.UR1);
}

int si70xxModeGet(void) {
	return si70xxWriteRead(&si70xxRUR1, sizeof(si70xxRUR1), &sSI70XX.UR1, SO_MEM(si70xx_t, UR1));
}

#if (si70xxI2C_LOGIC == 1)

/**
 * @brief	trigger A->D conversion with delay
 * @param 	pointer to endpoint to be read
 */
int	si70xxSense(epw_t * psEWP) {
	table_work[psEWP->uri == URI_SI70XX_RH ? URI_SI70XX_TMP : URI_SI70XX_RH].fBusy = 1;
	const u8_t * pCMD = (psEWP == &table_work[URI_SI70XX_RH]) ? &si70xxMRH_HMM : &si70xxMT_HMM;
	u8_t Cfg = sSI70XX.sUR1.cfg1 ? 2 : 0;
	Cfg += sSI70XX.sUR1.cfg0 ? 1 : 0;
	u32_t Dly = (psEWP == &table_work[URI_SI70XX_RH]) ? si70xxDelayRH[Cfg] : si70xxDelayT[Cfg];
	Dly *= MICROS_IN_MILLISEC;
	IF_SYSTIMER_START(debugTIMING, stSI70XX);
	int iRV = halI2CM_Queue(sSI70XX.psI2C, i2cWDR_B, (u8_t *) pCMD, sizeof(u8_t),
			sSI70XX.u8Buf, SO_MEM(si70xx_t, u8Buf), (i2cq_p1_t) Dly, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stSI70XX);
	IF_PT(debugDEVICE, "uri=%d  [ %-'hhY ]", psEWP->uri, SO_MEM(si70xx_t, u8Buf), sSI70XX.u8Buf);
	sSI70XX.RawVal = (sSI70XX.u8Buf[0] << 8) + sSI70XX.u8Buf[1];
	x64_t X64;
	if (psEWP == &table_work[URI_SI70XX_RH])
		X64.x32[0].f32 = (float) (((sSI70XX.RawVal * 125) >> 16) - 6);
	else
		X64.x32[0].f32 = (((float) sSI70XX.RawVal * 175.72) / 65536.0) - 46.85;
	vCV_SetValueRaw(&psEWP->var, X64);
	table_work[psEWP->uri == URI_SI70XX_RH ? URI_SI70XX_TMP : URI_SI70XX_RH].fBusy = 0;
	IF_P(debugDEVICE, "  Raw=%d  Norm=%f\r\n", sSI70XX.RawVal, X64.x32[0].f32);
	return iRV;
}

#elif (si70xxI2C_LOGIC == 2)

/**
 * @brief	trigger A->D conversion without delay (clock stretching ?)
 * @param 	pointer to endpoint to be read
 */
int	si70xxSense(epw_t * psEWP) {
	table_work[psEWP->uri == URI_SI70XX_RH ? URI_SI70XX_TMP : URI_SI70XX_RH].fBusy = 1;
	const u8_t * pCMD = (psEWP == &table_work[URI_SI70XX_RH]) ? &si70xxMRH_HMM : &si70xxMT_HMM;
	IF_SYSTIMER_START(debugTIMING, stSI70XX);
	int iRV = halI2CM_Queue(sSI70XX.psI2C, i2cWR_B, (u8_t *) pCMD, sizeof(u8_t),
			sSI70XX.u8Buf, SO_MEM(si70xx_t, u8Buf), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stSI70XX);
	IF_PT(debugDEVICE, "uri=%d  [ %-'hhY ]", psEWP->uri, SO_MEM(si70xx_t, u8Buf), sSI70XX.u8Buf);
	sSI70XX.RawVal = (sSI70XX.u8Buf[0] << 8) + sSI70XX.u8Buf[1];
	x64_t X64;
	if (psEWP == &table_work[URI_SI70XX_RH])
		X64.x32[0].f32 = (float) (((sSI70XX.RawVal * 125) >> 16) - 6);
	else
		X64.x32[0].f32 = (((float) sSI70XX.RawVal * 175.72) / 65536.0) - 46.85;
	vCV_SetValueRaw(&psEWP->var, X64);
	table_work[psEWP->uri == URI_SI70XX_RH ? URI_SI70XX_TMP : URI_SI70XX_RH].fBusy = 0;
	IF_P(debugDEVICE, "  Raw=%d  Norm=%f\r\n", sSI70XX.RawVal, X64.x32[0].f32);
	return iRV;
}

#elif (si70xxI2C_LOGIC == 3)

/**
 * @brief	step 3: sample read, convert  store
 * @param 	Expired timer handle
 */
 void si70xxReadCB(void * pvPara) {
	IF_SYSTIMER_STOP(debugTIMING, stSI70XX);
	epw_t * psEWP = pvPara;
	IF_PT(debugDEVICE, "uri=%d  [ %-'hhY ]", psEWP->uri, SO_MEM(si70xx_t, u8Buf), sSI70XX.u8Buf);
	sSI70XX.RawVal = (sSI70XX.u8Buf[0] << 8) + sSI70XX.u8Buf[1];
	IF_P(debugDEVICE, "  Raw=%d", sSI70XX.RawVal);
	x64_t X64;
	if (psEWP == &table_work[URI_SI70XX_RH]) {
//		IF_myASSERT(debugRESULT, (sSI70XX.RawVal & 0x0003) == 0x0002);
		sSI70XX.RawVal &= 0xFFFC;
		X64.x32[0].f32 = (float) (((sSI70XX.RawVal * 125) >> 16) - 6);
	} else {
//		IF_myASSERT(debugRESULT, (sSI70XX.RawVal & 0x0003) == 0x0000);
		X64.x32[0].f32 = (((float) sSI70XX.RawVal * 175.72) / 65536.0) - 46.85;
	}
	vCV_SetValueRaw(&psEWP->var, X64);
	table_work[psEWP->uri == URI_SI70XX_RH ? URI_SI70XX_TMP : URI_SI70XX_RH].fBusy = 0;
	IF_P(debugDEVICE, "  Norm=%f\r\n", X64.x32[0].f32);
}

/**
 * @brief	step 2: conversion timer expired, trigger sample read
 * @param 	(expired) timer handle
 */
void si70xxTimerHdlr(TimerHandle_t xTimer) {
	halI2CM_Queue(sSI70XX.psI2C, i2cRC_B, NULL, 0, sSI70XX.u8Buf, SO_MEM(si70xx_t, u8Buf), (i2cq_p1_t) si70xxReadCB, (i2cq_p2_t) (void *) pvTimerGetTimerID(xTimer));
}

void si70xxSenseCB(void * pV) {
	epw_t * psEWP = (epw_t *) pV;
	u8_t Cfg = sSI70XX.sUR1.cfg1 ? 2 : 0;
	Cfg += sSI70XX.sUR1.cfg0 ? 1 : 0;
	u32_t Dly = (psEWP == &table_work[URI_SI70XX_RH]) ? si70xxDelayRH[Cfg] : si70xxDelayT[Cfg];
	xTimerStart(sSI70XX.th, Dly);
}

/**
 * @brief	step 1: trigger A->D conversion with delay
 * @param 	pointer to endpoint to be read
 */
int	si70xxSense(epw_t * psEWP) {
	// mark endpoint as BUSY
	table_work[psEWP->uri == URI_SI70XX_RH ? URI_SI70XX_TMP : URI_SI70XX_RH].fBusy = 1;
	vTimerSetTimerID(sSI70XX.th, (void *) psEWP);
	const u8_t * pCMD = (psEWP == &table_work[URI_SI70XX_RH]) ? &si70xxMRH_NHMM : &si70xxMT_NHMM;
	IF_SYSTIMER_START(debugTIMING, stSI70XX);
	return halI2CM_Queue(sSI70XX.psI2C, i2cWC, (u8_t *) pCMD, 1, NULL, 0, (i2cq_p1_t) si70xxSenseCB, (i2cq_p2_t) psEWP);
}

#endif

// ################################ Rules configuration support ####################################

int	si70xxConfigMode (struct rule_t * psR, int Xcur, int Xmax, int EI) {
	// mode /si70xx idx res htr lev
	u8_t AI = psR->ActIdx;
	int res = psR->para.x32[AI][0].i32;
	int htr = psR->para.x32[AI][1].i32;
	int lev = psR->para.x32[AI][2].i32;
	IF_P(ioB1GET(dbgMode), "MODE 'SI70XX' Xcur=%d Xmax=%d res=%d htr=%d lev=%d\r\n", Xcur, Xmax, res, htr, lev);

	if (OUTSIDE(0, res, 3) ||
		OUTSIDE(0, htr, 1) ||
		OUTSIDE(0, lev, 15)) {
		RETURN_MX("Invalid Resolution or Heater value", erINV_PARA);
	}
	int iRV;
	do {
		sSI70XX.sUR1.cfg0 = (res & 0x01) ? 1 : 0;
		sSI70XX.sUR1.cfg1 = (res & 0x02) ? 1 : 0;
		sSI70XX.sUR1.htre = (htr > 0) ? 1 : 0;
		iRV = si70xxWriteReg(si70xxWUR1, sSI70XX.UR1);
		if (iRV == erSUCCESS)
			iRV = si70xxWriteReg(si70xxWHCR, sSI70XX.sHCR.level = lev);
		if (iRV < erSUCCESS)
			break;
	} while (++Xcur < Xmax);
	return iRV;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * @brief	device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	si70xxIdentify(i2c_di_t * psI2C) {
	u8_t si70xxBuf[10];
	psI2C->TRXmS = 50;
	psI2C->CLKuS = 13000;			// Max 13000 (13mS)
	psI2C->Test = 1;
	sSI70XX.psI2C = psI2C;
	int iRV = si70xxWriteRead(si70xxREID1, sizeof(si70xxREID1), &si70xxBuf[0], 8);
	IF_EXIT(iRV != erSUCCESS);
	#if (debugDEVICE)
	for (int i = 0; i < 8; i += 2)
		si70xxBuf[i >> 1] = si70xxBuf[i + 1];
	#endif
	iRV = si70xxWriteRead(si70xxREID2, sizeof(si70xxREID2), &si70xxBuf[4], 6);
	#if (debugDEVICE)
	si70xxBuf[6] = si70xxBuf[7];
	si70xxBuf[7] = si70xxBuf[8];
	IF_P(debugDEVICE, "si70xx ID [ %-'hhY ]", 8, si70xxBuf);
	#endif
	if ((iRV == erSUCCESS) && (si70xxBuf[4] == 0x06)) {
		psI2C->Type = i2cDEV_SI70XX;
		psI2C->Speed = i2cSPEED_400;
		psI2C->DevIdx = si70xxNumDev++;
		#if (debugDEVICE)
		si70xxWriteRead(si70xxRFWR, sizeof(si70xxRFWR), si70xxBuf, 1);
		P("  FW Rev=%d\r\n", si70xxBuf[0] == 0xFF ? 1 : si70xxBuf[0] == 0x20 ? 2 : -1);
		#endif
	}
exit:
//	psI2C->Test = 0;				// Leave ON to remove timeout errors
	return iRV;
}

int	si70xxConfig(i2c_di_t * psI2C) {
	#if (si70xxI2C_LOGIC == 3)
	sSI70XX.th = xTimerCreateStatic("si70xx", pdMS_TO_TICKS(5), pdFALSE, NULL, si70xxTimerHdlr, &sSI70XX.ts);
	#endif
	IF_SYSTIMER_INIT(debugTIMING, stSI70XX, stMICROS, "SI70XX", 100, 10000);
	return si70xxReConfig(psI2C);
}

int si70xxReConfig(i2c_di_t * psI2C) {
	epw_t * psEWP = &table_work[URI_SI70XX_RH];
	psEWP->var.def = SETDEF_CVAR(0, 0, vtVALUE, cvF32, 1, 0);
	psEWP->Tsns = psEWP->Rsns = SI70XX_T_SNS;
	psEWP->uri = URI_SI70XX_RH;

	psEWP = &table_work[URI_SI70XX_TMP];
	psEWP->var.def = SETDEF_CVAR(0, 0, vtVALUE, cvF32, 1, 0);
	psEWP->Tsns = psEWP->Rsns = SI70XX_T_SNS;
	psEWP->uri = URI_SI70XX_TMP;

	si70xxModeGet();
	si70xxModeSet(si70xxMODE_H08T12);
	return erSUCCESS;
}

int	si70xxDiags(i2c_di_t * psI2C) { return erSUCCESS; }

// ######################################### Reporting #############################################

const char * caMode[] = { "H12T14", "H08T12", "H10T13", "H11T11" };
const char * caLevel[] = { "3.09", "9.18", "15.24", "", "27.39", "", "", "", "51.69", "", "", "", "", "", "", "94.20" };

int si70xxReportAll(report_t * psR) {
	int iRV = 0;
	for (int dev = 0; dev < si70xxNumDev; ++dev) {
		iRV += halI2C_DeviceReport(psR, sSI70XX.psI2C);
		u8_t Cfg = sSI70XX.sUR1.cfg1 ? 2 : 0;
		Cfg += sSI70XX.sUR1.cfg0 ? 1 : 0;
		iRV += wprintfx(psR, "\tMode=%d (%s)  VddS=%d  Heater=%sabled  Level=%d (%s)\r\n", Cfg, caMode[Cfg],
			sSI70XX.sUR1.vdds, sSI70XX.sUR1.htre  ? "EN" : "DIS", sSI70XX.sHCR.level, caLevel[sSI70XX.sHCR.level]);
	}
	#if (si70xxI2C_LOGIC == 3)
	iRV += xRtosReportTimer(psR, sSI70XX.th);
	#endif
	return iRV;
}
#endif
