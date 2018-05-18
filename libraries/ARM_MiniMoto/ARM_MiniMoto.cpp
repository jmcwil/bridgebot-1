/***
Adaption of the existing SparkFunMiniMoto driver for use on ARM board

17-November-2017
**/


#include <Arduino.h>
#include "ARM_MiniMoto.h"

MiniMoto::MiniMoto(byte addr)
{
	_addr= addr;
}

void MiniMoto::drive(int speed) {
  byte regValue= 0x80; //clear the fault status
  I2CWrite(0x01, regValue);
  float voltage;
  // find the byte value of the speed and write it
  //voltage = speed * 3.7/100; //using 3.7 volt battery
  regValue = (byte)abs(speed);
  if (regValue > 63) regValue= 63; //cap value at 63
   regValue = regValue<<2; //Left shift to make room for bits 1:0
  if (speed <0) regValue |= 0x01;  //set bits based on input sign
  else          regValue |= 0x02;  
 I2CWrite(0x00, regValue); 
}

// coast by hi-z'ing the drivers
void MiniMoto::stop() {
  byte regValue = 0;
  I2CWrite(0x00, regValue);
}

// brake by putting a heavy load
void MiniMoto::brake() {
  byte regValue =0x03;
  I2CWrite(0x00, regValue);
}

void MiniMoto::I2CWrite(byte reg, byte regValue) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(regValue);
  Wire.endTransmission();
}

