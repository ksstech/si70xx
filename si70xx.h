/*
 * si70xx.h - Copyright (c) 2022-23 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ########################################### Macros ##############################################

#define	si70xxMODE_H12T14			0					// 12 + 11
#define	si70xxMODE_H08T12			1					// 4 + 4
#define	si70xxMODE_H10T13			2					// 5 + 7
#define	si70xxMODE_H11T11			3					// 7 + 3

// ######################################## Enumerations ###########################################

enum {								// Heater current options
	si70xxHT03m09 = 0,
	si70xxHT09m18 = 1,
	si70xxHT15m24 = 2,
	si70xxHT27m39 = 4,
	si70xxHT51m69 = 8,
	si70xxHT94m20 = 15,
};

// ######################################### Structures ############################################

// See http://www.catb.org/esr/structure-packing/
// Also http://c0x.coding-guidelines.com/6.7.2.1.html

typedef struct __attribute__((packed)) {				// SI70006/13/14/20/xx TMP & RH sensors
	u8_t cfg0:1;					// Config 0
	u8_t res0:1;
	u8_t htre:1;					// Heater Enable
	u8_t res1:3;
	u8_t vdds:1;					// 0 = Vcc >= 1.9V, 1 = 1.8V <= Vcc < 1.9V
	u8_t cfg1:1;					// Config 1
} si70xx_ur1_t;
DUMB_STATIC_ASSERT(sizeof(si70xx_ur1_t) == 1);

typedef struct __attribute__((packed)) {				//
	u8_t level:4;					// 0 = 3.09mA -> 15 = 94.20mA
	u8_t lastOP:1;					// 0 = Temp read, 1 = RH read
	u8_t spare:3;
} si70xx_hcr_t;
DUMB_STATIC_ASSERT(sizeof(si70xx_hcr_t) == 1);

struct i2c_di_t;
typedef struct __attribute__((packed)) {				// SI70006/13/14/20/xx TMP & RH sensors
	struct i2c_di_t * psI2C;				// 4 bytes
	SemaphoreHandle_t mux;
	#if (si70xxI2C_LOGIC == 3)
	TimerHandle_t th;
	StaticTimer_t ts;
	#endif
	union {
		si70xx_ur1_t sUR1;			// 1 byte
		u8_t UR1;
	};
	union {
		si70xx_hcr_t sHCR;			// 1 byte
		u8_t HCR;
	};
	union {
		u16_t	RawVal;
		u8_t		u8Buf[2];
	};
} si70xx_t;
#if (si70xxI2C_LOGIC == 3)
	DUMB_STATIC_ASSERT(sizeof(si70xx_t) == 60);
#else
	DUMB_STATIC_ASSERT(sizeof(si70xx_t) == 12);
#endif

// ###################################### Public variables #########################################

extern u8_t const si70xxMRH_HMM;
extern u8_t const si70xxMRH_NHMM;
extern u8_t const si70xxMT_HMM;
extern u8_t const si70xxMT_NHMM;
extern u8_t const si70xxRT_PMRH;
extern u8_t const si70xxRESET;
extern u8_t const si70xxWUR1;
extern u8_t const si70xxRUR1;
extern u8_t const si70xxWHCR;
extern u8_t const si70xxRHCR;
extern u8_t const si70xxREID1[2];
extern u8_t const si70xxREID2[2];
extern u8_t const si70xxRFWR[2];

extern const u8_t si70xxDelayRH[4];
extern const u8_t si70xxDelayT[4];

extern si70xx_t sSI70XX;

// ###################################### Public functions #########################################

int si70xxWriteReg(u8_t Reg, u8_t Val);

int	si70xxConvertTemperature(si70xx_t * psSI70XX);
int	si70xxReadSP(si70xx_t * psSI70XX, int Len);
int	si70xxWriteSP(si70xx_t * psSI70XX);
int	si70xxWriteEE(si70xx_t * psSI70XX);

int	si70xxInitialize(si70xx_t * psSI70XX);
int	si70xxResetConfig(si70xx_t * psSI70XX);
struct report_t;
int si70xxReportAll(struct report_t * psR);

// ##################################### I2C Task support ##########################################

struct rule_t;
int	si70xxConfigMode (struct rule_t *, int Xcur, int Xmax, int EI);
int	si70xxIdentify(struct i2c_di_t * psI2C);
int	si70xxConfig(struct i2c_di_t * psI2C);
int	si70xxReConfig(struct i2c_di_t * psI2C);
int	si70xxDiags(struct i2c_di_t * psI2C);

struct epw_t;
int	si70xxSense(struct epw_t * psEWP);

#ifdef __cplusplus
}
#endif
