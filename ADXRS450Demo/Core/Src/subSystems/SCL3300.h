/*
 * SCL3300.h
 *
 *  Created on: Aug 28, 2023
 *      Author: ParsaMalekpour
 */

#ifndef __SCL3300_H
#define __SCL3300_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "spi.h"

//extern SPI_HandleTypeDef hspi1;

#define SCL3300_HSPI	hspi1


#define SCL3300_CS_PIN	GPIOA,GPIO_PIN_4
//Define Hardware
#define SCL3300_writeCS_L HAL_GPIO_WritePin(SCL3300_CS_PIN, GPIO_PIN_RESET)
#define SCL3300_writeCS_H HAL_GPIO_WritePin(SCL3300_CS_PIN, GPIO_PIN_SET)


//Define allowed commands to SCL3300 inclinometer
#define RdAccX		0x040000f7
#define RdAccY		0x080000fd
#define RdAccZ		0x0c0000fb
#define RdSTO		0x100000e9
#define EnaAngOut	0xb0001f6f
#define RdAngX		0x240000c7
#define RdAngY		0x280000cd
#define RdAngZ		0x2c0000cb
#define RdTemp		0x140000ef
#define RdStatSum	0x180000e5
#define RdErrFlg1	0x1c0000e3
#define RdErrFlg2	0x200000c1
#define RdCMD		0x340000df
#define ChgMode1	0xb400001f
#define ChgMode2	0xb4000102
#define ChgMode3	0xb4000225
#define ChgMode4	0xb4000338
#define SetPwrDwn	0xb400046b
#define WakeUp		0xb400001f
#define SWreset		0xb4002098
#define RdWHOAMI	0x40000091
#define RdSer1		0x640000a7
#define RdSer2		0x680000AD
#define RdCurBank	0x7c0000b3
#define SwtchBnk0	0xfc000073
#define SwtchBnk1	0xfc00016e

// Structure to hold raw sensor data
// We need to populate all this every time we read a set of data
typedef struct SCL3300data {
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t STO;
    double TEMP;
    uint16_t STATUS;
    uint16_t ERR_FLAG1;
    uint16_t ERR_FLAG2;
    double AngX;
    double AngY;
    double AngZ;
    uint16_t ANG_CTRL;
    uint16_t WHOAMI;
    uint32_t SERIAL;
}SCL3300data;



//Functions
uint8_t SCL3300_CalculateCRC(uint32_t Data);
uint8_t SCL3300_CRC8(uint8_t BitValue, uint8_t SCL3300_CRC);
uint32_t SCL3300_transfer(unsigned long value);
bool SCL3300_Init(void);
bool SCL3300_isConnected();

bool SCL3300_Update();
double SCL3300_getValue(uint8_t channel);
void SCL3300_reset();

#endif /* SCL3300_H_ */
