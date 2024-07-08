/***************************************************************************//**
 *   @file   ADXRS453.c
 *   @brief  Implementation of ADXRS453 Driver.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "../subSystems/ADXRS453.h"			// ADXRS453 definitions.

//#include "../subSystems/Communication.h"		// Communication definitions.

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Sets or clears the parity bit in order to ensure that the overall 
 *        parity of the data word is odd.
 *
 * @param data.
 *
 * @return parityBit.
*******************************************************************************/
uint8_t ADXRS453_ParityBit(uint32_t data)
{
    uint8_t parityBit = 0;
    uint8_t bitIndex  = 0;
    uint8_t sum       = 0;

    for(bitIndex = 0; bitIndex < 32; bitIndex++)
    {
        sum += ((data >> bitIndex) & 0x1);
    }
    if (! (sum % 2))
    {
        parityBit |= 0x1;
    }

    return parityBit;
}
 

/***************************************************************************//**
 * @brief Initializes the ADXRS453 and checks if the device is present.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID starts
 *                               with 0x52).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
uint8_t ADXRS453_Init(uint8_t channel)
{ 
	uint8_t status  = 0x1;
	uint8_t data[4] = {0, 0, 0, 0};
	
//	status = SPI_Init(0, 1000000, 0, 1);
    data[3] = 3;
	data[2] = 0;
    data[1] = 0;
	data[0] = 2;
	ADXRS453_Write(channel, data, 4);
	HAL_Delay(50);
	data[3] = 0;
	data[2] = 0;
    data[1] = 0;
	data[0] = 2;
	ADXRS453_Write(channel, data, 4);
	HAL_Delay(50);
	data[3] = 0;
	data[2] = 0;
    data[1] = 0;
	data[0] = 2;
	ADXRS453_Write(channel, data, 4);
	HAL_Delay(50);
	data[3] = 0;
	data[2] = 0;
    data[1] = 0;
	data[0] = 2;
	ADXRS453_Write(channel, data, 4);
	HAL_Delay(100);
	if((ADXRS453_GetRegisterValue(channel, ADXRS453_PID1) >> 8) != 0x52)
	{
		status = 0x0;
	}
	
	return status;
}

/***************************************************************************//**
 * @brief Reads the rate data and converts it to degrees/second.
 *
 * @param dev - The device structure.
 *
 * @return rate - The rate value in degrees/second.
*******************************************************************************/
double ADXRS453_getValue(uint8_t channel)
{
	    uint8_t data[4]      = {0, 0, 0, 0};
		uint32_t receivedData = 0x00;
		double rate;
		
		data[0] = ADXRS453_SENSOR_DATA;
		ADXRS453_Read(channel, data, 4);
		receivedData += ((uint32_t)data[0] << 24);
		receivedData += ((uint32_t)data[1] << 16);
		receivedData += (data[2] << 8);
		receivedData = ((receivedData >> 10) & 0xFFFF);
		
		/*!< If data received is in positive degree range */
		if(receivedData < 0x8000)
			rate = ((float)receivedData / 80);
		/*!< If data received is in negative degree range */
		else
			rate = (-1) * ((float)(0xFFFF - receivedData + 1) / 80.0);

		return rate;
}
/***************************************************************************//**
 * @brief Reads register from ADXRS453.
 *
 * @param regAddress - the address of the register.
 *
 * @return receivedData.
*******************************************************************************/
uint16_t ADXRS453_GetRegisterValue(uint8_t channel, uint8_t regAddress)
{
	    uint8_t data[4]      = {0, 0, 0, 0};
		uint32_t receivedData = 0x00;
		uint32_t dataToSend   = 0;
        
		data[0] = ADXRS453_READ_DATA | (regAddress >> 7);
		data[1] = regAddress << 1;
		dataToSend += ((uint32_t)data[0] << 24);
		dataToSend += ((uint32_t)data[1] << 16);
		data[3] = ADXRS453_ParityBit(dataToSend);
		ADXRS453_Write(channel, data, 4);
		ADXRS453_Read(channel, data, 4);	// because the device answers only on the next SPI word
		receivedData += ((uint32_t)data[1] << 16);
		receivedData += ((uint32_t)data[2] << 8);
		receivedData += (data[3] << 0);
		receivedData = ((receivedData >> 5) & 0xFFFF);
		
		return (uint16_t)receivedData;
}
/***************************************************************************//**
 * @brief Writes register to ADXRS453.
 *
 * @param regAddress - the address of the register.
 *
 * @param regData - data to write to the register 
 *
 * @return receivedData.
*******************************************************************************/
void ADXRS453_SetRegisterValue(uint8_t channel, uint8_t regAddress, uint16_t regData)
{
	    uint8_t data[4]    = {0, 0, 0, 0};
	    uint32_t dataToSend = 0;
		
		data[0] = ADXRS453_READ_DATA | (regAddress >> 7);
		data[1] = (regAddress << 1) | (regData >> 15);
		data[2] = (regData & 0x7F80) >> 7;
		data[3] = (regData & 0xFF) << 1;
		dataToSend += ((uint32_t)data[0] << 24);
		dataToSend += ((uint32_t)data[1] << 16);
		data[3] = ADXRS453_ParityBit(dataToSend);
		ADXRS453_Write(channel, data, 4);
}
/***************************************************************************//**
 * @brief Reads temperature from ADXRS453 and converts it to degrees Celsius
 *
 * @param None.
 *
 * @return temperature.
*******************************************************************************/
uint8_t ADXRS453_GetTemperature(uint8_t channel)
{
	uint16_t temp = 0;
	
	temp = ADXRS453_GetRegisterValue(channel, ADXRS453_TEMP1);
	temp >>= 6;	
	temp -= 0x31f;
	temp /= 5;
	
	return (uint8_t)temp;
}




inline void ADXRS453_DeviceCS(uint8_t channel, GPIO_PinState value){
	switch(channel){
		case 0:
			HAL_GPIO_WritePin(ADXRS453_CS0_PIN,value);
			break;
		case 1:
			HAL_GPIO_WritePin(ADXRS453_CS1_PIN,value);
			break;
		case 2:
			HAL_GPIO_WritePin(ADXRS453_CS2_PIN,value);
			break;
		}

}
/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer as an input parameter and the
 *               read buffer as an output parameter.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return Number of read bytes.
*******************************************************************************/
uint8_t ADXRS453_Read(uint8_t slaveDeviceId, uint8_t* data, uint8_t bytesNumber)
{
	uint8_t   byte            = 0;
	uint8_t   writeBuffer[4]  = {0, 0, 0, 0};

	for(byte = 0; byte < bytesNumber; byte++) writeBuffer[byte] = data[byte];


	ADXRS453_DeviceCS(slaveDeviceId, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&ADXRS453_HSPI,writeBuffer,data,bytesNumber,500);
	ADXRS453_DeviceCS(slaveDeviceId, GPIO_PIN_SET);

	return bytesNumber;
}


/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
void ADXRS453_Update(ADXRS453data* regs){
	regs->RATE = ADXRS453_GetRegisterValue(regs->channel, ADXRS453_RATE1);
	regs->TEM = ADXRS453_GetRegisterValue(regs->channel, ADXRS453_TEMP1);
	regs->LOCST = ADXRS453_GetRegisterValue(regs->channel, ADXRS453_LOCST1);
	regs->HICST = ADXRS453_GetRegisterValue(regs->channel, ADXRS453_HICST1);
	regs->QUAD = ADXRS453_GetRegisterValue(regs->channel, ADXRS453_QUAD1);
	regs->PID = ADXRS453_GetRegisterValue(regs->channel, ADXRS453_PID1);
	regs->SN = ADXRS453_GetRegisterValue(regs->channel, ADXRS453_SNL);
	regs->SN |= (uint32_t)ADXRS453_GetRegisterValue(regs->channel, ADXRS453_SNH) << 16;

	regs->FAULTS.raw = ADXRS453_GetRegisterValue(regs->channel, ADXRS453_FAULT1);;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
uint8_t ADXRS453_Write(uint8_t slaveDeviceId,
                        uint8_t* data,
                        uint8_t bytesNumber)
{

	ADXRS453_DeviceCS(slaveDeviceId, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&ADXRS453_HSPI,data,bytesNumber,500);

	ADXRS453_DeviceCS(slaveDeviceId, GPIO_PIN_SET);

	return bytesNumber;
}
