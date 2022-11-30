/*
 * Copyright 2022 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#include <stdint.h>

#include "endpoints.h"
#include "hal_i2c.h"

#ifdef __cplusplus
	extern "C" {
#endif

// ########################################### Macros ##############################################


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
	uint8_t	cfg0 : 1;				// Config 0
	uint8_t	res0 : 1;
	uint8_t	htre : 1;				// Heater Enable
	uint8_t	res1 : 3;
	uint8_t	vdds : 1;				// 0 = Vcc >= 1.9V, 1 = 1.8V <= Vcc < 1.9V
	uint8_t	cfg1 : 1;				// Config 1
} si70xx_ur1_t;
DUMB_STATIC_ASSERT(sizeof(si70xx_ur1_t) == 1);

typedef struct __attribute__((packed)) {				//
	uint8_t	level : 4;				// 0 = 3.09mA -> 15 = 94.20mA
	uint8_t	lastOP : 1;				// 0 = Temp read, 1 = RH read
	uint8_t	spare : 3;
} si70xx_hcr_t;
DUMB_STATIC_ASSERT(sizeof(si70xx_hcr_t) == 1);

typedef struct __attribute__((packed)) {				// SI70006/13/14/20/xx TMP & RH sensors
	i2c_di_t *		psI2C;			// 4 bytes
	SemaphoreHandle_t mux ;
	#if (si70xxI2C_LOGIC == 3)
	TimerHandle_t th;
	StaticTimer_t ts;
	#endif
	union {
		si70xx_ur1_t sUR1;			// 1 byte
		uint8_t UR1;
	};
	union {
		si70xx_hcr_t sHCR;			// 1 byte
		uint8_t HCR;
	};
	union {
		uint16_t	RawVal;
		uint8_t		u8Buf[2];
	};
} si70xx_t;
#if (si70xxI2C_LOGIC == 3)
	DUMB_STATIC_ASSERT(sizeof(si70xx_t) == 60);
#else
	DUMB_STATIC_ASSERT(sizeof(si70xx_t) == 12);
#endif

// ###################################### Public variables #########################################


// ###################################### Public functions #########################################

int	si70xxConvertTemperature(si70xx_t * psSI70XX) ;
int	si70xxReadSP(si70xx_t * psSI70XX, int Len) ;
int	si70xxWriteSP(si70xx_t * psSI70XX) ;
int	si70xxWriteEE(si70xx_t * psSI70XX) ;

int	si70xxInitialize(si70xx_t * psSI70XX) ;
int	si70xxResetConfig(si70xx_t * psSI70XX) ;
void si70xxReportAll(void) ;

// ##################################### I2C Task support ##########################################

struct rule_t ;
int	si70xxConfigMode (struct rule_t *, int Xcur, int Xmax, int EI);
int	si70xxIdentify(i2c_di_t * psI2C_DI);
int	si70xxConfig(i2c_di_t * psI2C_DI);
int	si70xxReConfig(i2c_di_t * psI2C_DI);
int	si70xxDiags(i2c_di_t * psI2C_DI);

struct epw_t ;
int	si70xxReadHdlr(epw_t * psEWP);

#ifdef __cplusplus
	}
#endif
