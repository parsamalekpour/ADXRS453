/*
 * SCL3300.c
 *
 *  Created on: Aug 28, 2023
 *      Author: ParsaMalekpour
 */

#include "SCL3300.h"
#include <stdint.h>
#include <spi.h>

//Variables

uint8_t scl3300_mode = 2; // Default inclinometer mode
uint8_t SCL3300_CMD, SCL3300_CRC;
uint16_t SCL3300_DATA;



bool crcerr, statuserr;
uint32_t modeCMD[5]  = { 0, ChgMode1, ChgMode2, ChgMode3, ChgMode4 };

// Routine to transfer a 32-bit integer to the SCL3300, and return the 32-bit data read
uint32_t SCL3300_transfer(uint32_t value) {
  uint8_t data[4];


  data[3] = value; //Allow 32 bit value to be sent 8 bits at a time
  data[2] = value >> 8;
  data[1] = value >> 16;
  data[0] = value >> 24;

  //The datasheet shows the CS line must be high during this time


  SCL3300_writeCS_L; //Now chip select can be enabled for the full 32 bit xfer
  SCL3300_DATA = 0;

  //Transfer data using HAL library
  HAL_SPI_TransmitReceive(&SCL3300_HSPI, data, data, 4, 100);

  SCL3300_DATA = data[2] + (data[1] << 8);
  SCL3300_CRC = data[3];
  SCL3300_CMD = data[0];

  SCL3300_writeCS_H; //And we are done
  //HAL_Delay(1);
  //merge 4byte into one 32bit data
  value = data[4] | (data[3]<<8) | (data[2]<<16) | (data[1]<<24);


  if (SCL3300_CRC == SCL3300_CalculateCRC(value))
    crcerr = false;
  else
    crcerr = true;
  //check RS bits
  if ((SCL3300_CMD & 0x03) == 0x01)
    statuserr = false;
  else
    statuserr = true;

  return value;
}

// The following is taken directly from the Murata SCL3300 datasheet
// Calculate CRC for 24 MSB's of the 32 bit dword
// (8 LSB's are the CRC field and are not included in CRC calculation)
uint8_t SCL3300_CalculateCRC(uint32_t Data)
{
uint8_t BitIndex;
uint8_t BitValue;
uint8_t SCL3300_CRC;

SCL3300_CRC = 0xFF;
for (BitIndex = 31; BitIndex > 7; BitIndex--) {
  BitValue = (uint8_t)((Data >> BitIndex) & 0x01);
  SCL3300_CRC = SCL3300_CRC8(BitValue, SCL3300_CRC);
}
SCL3300_CRC = (uint8_t)~SCL3300_CRC;
return SCL3300_CRC;
}

uint8_t SCL3300_CRC8(uint8_t BitValue, uint8_t SCL3300_CRC)
{
  uint8_t Temp;
  Temp = (uint8_t)(SCL3300_CRC & 0x80);
  if (BitValue == 0x01) {
    Temp ^= 0x80;
  }
  SCL3300_CRC <<= 1;
  if (Temp > 0) {
    SCL3300_CRC ^= 0x1D;
  }
  return SCL3300_CRC;
}


// Current Version of begin() to initialize the library and the SCL3300
bool SCL3300_Init(void) {

  //initSPI();	// Initialize SPI Library

  //Write SW Reset command
  SCL3300_transfer(SwtchBnk0);
  SCL3300_transfer(SWreset);
  HAL_Delay(1);
  //Set measurement mode
  SCL3300_transfer(modeCMD[scl3300_mode]); //Set mode on hardware
  //We're good, so Enable angle outputs
  SCL3300_transfer(EnaAngOut);
  //The first response after reset is undefined and shall be discarded
  //wait 5 ms to stablize
  HAL_Delay(100);

  //Read Status to clear the status summary
  SCL3300_transfer(RdStatSum);
  SCL3300_transfer(RdStatSum); //Again, due to off-response protocol used
  SCL3300_transfer(RdStatSum); //And now we can get the real status

  //Read the WHOAMI register
  SCL3300_transfer(RdWHOAMI);
  //And again
  SCL3300_transfer(RdWHOAMI);
  //if (!setFastRead) endTransmission(); //Let go of SPI port/bus
  //We now wait until the end of begin() to report if an error occurred
  if (crcerr || statuserr) return false;
  // Once everything is initialized, return a known expected value
  // The WHOAMI command should give an 8 bit value of 0xc1
  return (SCL3300_DATA == 0xc1); //Let the caller know if this worked
}




void SCL3300_reset(){
	SCL3300_transfer(SWreset);
}

double SCL3300_getValue(uint8_t channel){

	uint32_t cmd[] = {RdAngX,RdAngY,RdAngZ};
	if(channel>2) channel = 2;

	SCL3300_transfer(cmd[channel]);
	SCL3300_transfer(RdWHOAMI);


	return (SCL3300_DATA / 16384.) * 90.;

}
//Read all the sensor data together to keep it consistent
//This is required according to the datasheet
//TODO: complete for all registers
bool SCL3300_Update(SCL3300data *sclData) {
  //Version 3 of this function
  bool errorflag = false;
  //Read all Sensor Data, as per Datasheet requirements

  SCL3300_transfer(SwtchBnk0);

  SCL3300_transfer(RdAngX);
  errorflag |= (crcerr || statuserr);
  //dummy value

  SCL3300_transfer(RdAngY);
  errorflag |= (crcerr || statuserr);
  sclData->AngX = SCL3300_DATA;

  SCL3300_transfer(RdAngZ);
  errorflag |= (crcerr || statuserr);
  sclData->AngY = SCL3300_DATA;

  SCL3300_transfer(RdStatSum);
  errorflag |= (crcerr || statuserr);
  sclData->AngZ = SCL3300_DATA;

  SCL3300_transfer(RdTemp);
  errorflag |= (crcerr || statuserr);
  sclData->STATUS = SCL3300_DATA;

  SCL3300_transfer(RdErrFlg1);
  errorflag |= (crcerr || statuserr);
  sclData->TEMP = SCL3300_DATA;

  SCL3300_transfer(RdErrFlg2);
  errorflag |= (crcerr || statuserr);
  sclData->ERR_FLAG1 = SCL3300_DATA;

  SCL3300_transfer(RdSTO);
  errorflag |= (crcerr || statuserr);
  sclData->ERR_FLAG2 = SCL3300_DATA;

  SCL3300_transfer(SwtchBnk1);
  errorflag |= (crcerr || statuserr);
  sclData->STO = SCL3300_DATA;

  SCL3300_transfer(RdSer1);
  errorflag |= (crcerr || statuserr);
  //dummy data

  SCL3300_transfer(RdSer2);
  errorflag |= (crcerr || statuserr);
  sclData->SERIAL = SCL3300_DATA;

  SCL3300_transfer(SwtchBnk0);
  errorflag |= (crcerr || statuserr);
  sclData->SERIAL |= (uint32_t)SCL3300_DATA << 16;


  return errorflag; //Let the caller know this worked
}

