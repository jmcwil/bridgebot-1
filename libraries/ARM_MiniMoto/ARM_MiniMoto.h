/***
Adaption of the existing SparkFunMiniMoto driver for use on ARM board

17-November-2017
**/


#ifndef ARM_MiniMoto_h
#define ARM_MiniMoto_h

#include <Arduino.h>
#include <Wire.h>

class MiniMoto
{
	public:
		MiniMoto(byte addr);
		void drive(int speed);
		void stop();
		void brake();
	private:
		void I2CWrite(byte reg, byte regValue);
    byte _addr;

};

#endif
