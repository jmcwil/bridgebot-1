/***
Adaption of the existing SparkFunMiniMoto driver for use on ARM board

17-November-2017
**/


#ifndef BB_MPU9250_h
#define BB_MPU9250_h

#include <Arduino.h>
#include <Wire.h>

class MPU9250
{
	public:
		MPU9250(byte addr);
		void initMPU();
		void recordAccel(float&, float&, float&);
		void recordGyro(float&, float&, float&);
    void recordMag();
	private:
		void I2CWrite(byte reg, byte regValue);
    byte _addr;

};

#endif
