// si70cc.c - Copyright (c) 2022-24 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_SI70XX > 0)
#include "endpoints.h"
#include "hal_i2c_common.h"
#include "report.h"
#include "rules.h"
#include "si70xx.h"
#include "syslog.h"
#include "systiming.h"					// timing debugging
#include "errors_events.h"

#define	debugFLAG					0xF000

#define	debugDEVICE					(debugFLAG & 0x0001)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ############################################# Macros ############################################


// #################################### SI7006/13/20/21 Addresses ##################################

#define	si70xxADDR0					0x40				// SI702x, SI700x address
#define	si70xxADDR1					0x41				// SI701X address

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
	IF_SYSTIMER_START(debugTIMING, stSI70XX);
	int iRV = halI2C_Queue(sSI70XX.psI2C, i2cW_B, (u8_t *) pTxBuf, TxLen, NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stSI70XX);
	return iRV;
}

int si70xxWriteReg(u8_t Reg, u8_t Val) {
	u8_t caBuf[2] = { Reg, Val };
	IF_SYSTIMER_START(debugTIMING, stSI70XX);
	int iRV = si70xxWrite(caBuf, sizeof(caBuf));
	IF_SYSTIMER_STOP(debugTIMING, stSI70XX);
	return iRV;
}

static int si70xxWriteRead(const u8_t * pTxBuf, size_t TxLen, u8_t * pRxBuf, size_t RxLen) {
	IF_SYSTIMER_START(debugTIMING, stSI70XX);
	int iRV = halI2C_Queue(sSI70XX.psI2C, i2cWR_B, (u8_t *) pTxBuf, TxLen,
			pRxBuf, RxLen, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stSI70XX);
	return iRV;
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

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * @brief	device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	si70xxIdentify(i2c_di_t * psI2C) {
	sSI70XX.psI2C = psI2C;
	psI2C->Type = i2cDEV_SI70XX;
	psI2C->Speed = i2cSPEED_400;
	psI2C->TObus = 50;	// 1000 13000
	psI2C->Test = 1;
	u8_t si70xxBuf[10];
	int iRV = si70xxWriteRead(si70xxREID1, sizeof(si70xxREID1), &si70xxBuf[0], 8);
	if (iRV != erSUCCESS) goto exit;
	#if (debugDEVICE)
	for (int i = 0; i < 8; i += 2) si70xxBuf[i >> 1] = si70xxBuf[i + 1];
	#endif
	iRV = si70xxWriteRead(si70xxREID2, sizeof(si70xxREID2), &si70xxBuf[4], 6);
	#if (debugDEVICE)
	si70xxBuf[6] = si70xxBuf[7];
	si70xxBuf[7] = si70xxBuf[8];
	IF_P(debugDEVICE, "si70xx ID [ %-'hhY ]", 8, si70xxBuf);
	#endif
	if (iRV < erSUCCESS) goto exit;
	if (si70xxBuf[4] != 0x06) goto err_version;
	psI2C->DevIdx = si70xxNumDev++;
	psI2C->IDok = 1;
	psI2C->Test = 0;
	#if (debugDEVICE)
	si70xxWriteRead(si70xxRFWR, sizeof(si70xxRFWR), si70xxBuf, 1);
	P("  FW Rev=%d\r\n", si70xxBuf[0] == 0xFF ? 1 : si70xxBuf[0] == 0x20 ? 2 : -1);
	#endif
	goto exit;
err_version:
	iRV = erINV_VERSION;
exit:
	return iRV;
}

int	si70xxConfig(i2c_di_t * psI2C) {
	if (!psI2C->IDok) return erINV_STATE;

	psI2C->CFGok = 0;
	int iRV = si70xxModeGet();
	if (iRV < erSUCCESS) goto exit;

	iRV = si70xxModeSet(si70xxMODE_H08T12);
	if (iRV < erSUCCESS) goto exit;
	psI2C->CFGok = 1;

	// once off init....
	if (!psI2C->CFGerr) {
		IF_SYSTIMER_INIT(debugTIMING, stSI70XX, stMICROS, "SI70XX", 100, 10000);
		#if (si70xxI2C_LOGIC == 3)
		sSI70XX.th = xTimerCreateStatic("si70xx", pdMS_TO_TICKS(5), pdFALSE, NULL, si70xxTimerHdlr, &sSI70XX.ts);
		#endif
	}
exit:
	return iRV;
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
		iRV += xReport(psR, "\tMode=%d (%s)  VddS=%d  Heater=%sabled  Level=%d (%s)\r\n", Cfg, caMode[Cfg],
			sSI70XX.sUR1.vdds, sSI70XX.sUR1.htre  ? "EN" : "DIS", sSI70XX.sHCR.level, caLevel[sSI70XX.sHCR.level]);
	}
	#if (si70xxI2C_LOGIC == 3)
	iRV += xRtosReportTimer(psR, sSI70XX.th);
	#endif
	return iRV;
}
#endif
