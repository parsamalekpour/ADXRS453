/***************************************************************************//**
 *   @file   ADXRS453.h
 *   @brief  Header file of ADXRS453 Driver.
 *   @author Mihai Bancisor (Mihai.Bancisor@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 796
*******************************************************************************/
//#ifndef __ADXRS45x_H__
//#define __ADXRS45x_H__

/******************************************************************************/
/******************************* ADXRS453 *************************************/
/******************************************************************************/

#include "main.h"
#include "spi.h"
#include "stdbool.h"
/* SPI slave device ID */
#define ADXRS453_CS0_PIN GPIOA,GPIO_PIN_4
#define ADXRS453_CS1_PIN GPIOD,GPIO_PIN_9
#define ADXRS453_CS2_PIN GPIOD,GPIO_PIN_10

#define ADXRS453_HSPI hspi1

#define ADXRS453_STARTUP_DELAY	0.05 /* sec */

/* The MSB for the spi commands */
#define ADXRS453_SENSOR_DATA	0x20
#define ADXRS453_WRITE_DATA		0x40
#define ADXRS453_READ_DATA		0x80

/* Memory register map */
#define ADXRS453_RATE1			0x00	// Rate Registers
#define ADXRS453_TEMP1			0x02	// Temperature Registers
#define ADXRS453_LOCST1			0x04	// Low CST Memory Registers
#define ADXRS453_HICST1			0x06	// High CST Memory Registers
#define ADXRS453_QUAD1			0x08	// Quad Memory Registers
#define ADXRS453_FAULT1			0x0A	// Fault Registers
#define ADXRS453_PID1			0x0C	// Part ID Register 1
#define ADXRS453_SNH			0x0E	// Serial Number Registers, 4 bytes
#define ADXRS453_SNL			0x10


#define ADXRS453_WRERR_MASK		(0x7 << 29)
#define ADXRS453_GET_ST(a)		((a >> 26) & 0x3)  // Status bits

#pragma pack(push, 1)
typedef struct {
	bool ZERO:1;
	bool CHK:1;
	bool CST:1;
	bool PWRE:1;
	bool POR:1;
	bool NVM:1;
	bool Q:1;
	bool PLL:1;
	bool UV:1;
	bool OV:1;
	bool AMP:1;
	bool FAIL:1;
}ADXRS453_Faults;

union flt_u {ADXRS453_Faults details; uint16_t raw;};
#pragma pack(pop)

typedef struct {
	uint8_t channel;
	int16_t RATE;
	int16_t TEM;
	int16_t LOCST;
	int16_t HICST;
	int16_t QUAD;
	uint16_t PID;
	uint32_t SN;
	union flt_u FAULTS;
}ADXRS453data;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/*! Update all registers from ADXRS453 */
void ADXRS453_Update(ADXRS453data* regs);

/*! Sets or clears the parity bit in order to ensure that the overall parity of
the data word is odd. */
uint8_t ADXRS453_ParityBit(uint32_t data);

/*! Initializes the ADXRS453 and checks if the device is present. */
uint8_t ADXRS453_Init(uint8_t channel);

/*! Reads data from ADXRS453. */
double ADXRS453_getValue(uint8_t channel);

/*! Reads register from ADXRS453. */
uint16_t ADXRS453_GetRegisterValue(uint8_t channel, uint8_t regAddress);

/*! Writes register to ADXRS453. */
void ADXRS453_SetRegisterValue(uint8_t channel, uint8_t regAddress, uint16_t regData);

/*! Reads temperature from ADXRS453 and converts it to degrees Celsius. */
uint8_t ADXRS453_GetTemperature(uint8_t channel);

/*! Reads data from SPI. */
uint8_t ADXRS453_Read(uint8_t slaveDeviceId, uint8_t* data, uint8_t bytesNumber);

/*! Writes data to SPI. */
uint8_t ADXRS453_Write(uint8_t slaveDeviceId,uint8_t* data, uint8_t bytesNumber);

void ADXRS453_DeviceCS(uint8_t channel, GPIO_PinState value);

//#endif	/* __ADXRS453_H__ */
