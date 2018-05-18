/***
Adaption of the existing SparkFunMiniMoto driver for use on ARM board

17-November-2017
**/


#include <Arduino.h>
#include "BB_MPU9250.h"

MPU9250::MPU9250(byte addr)
{
	_addr= addr;
}

// Initialize the MPU9250
void MPU9250::initMPU() {
  Wire.beginTransmission(_addr); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(_addr); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(_addr); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission();
  
}

/*
 * Read accel registers and convert to G's
 * Parameters: variables to store the accelX, accelY, and accelZ
 */
void MPU9250::recordAccel(float& accelX, float& accelY, float& accelZ) {
  Wire.beginTransmission(0b1101001); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101001, 6); //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  long X = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  long Y = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  long Z = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  accelX = X / 16384.0;
  accelY = Y / 16384.0;
  accelZ = Z / 16384.0;
}


/*
 * Read gyro registers and convert to rad/s
 * Parameters: variables to store the accelX, accelY, and accelZ
 */
void MPU9250::recordGyro(float& gyroX, float& gyroY, float& gyroZ) {
  Wire.beginTransmission(_addr); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(_addr, 6); //Request Gyro Registers (43 - 48)
  while (Wire.available() < 6);
  long X = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  long Y = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  long Z = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  gyroX = X / 131.0;
  gyroY = Y / 131.0;
  gyroZ = Z / 131.0;
}
void MPU9250::recordMag() {
 //not written yet
}
void MPU9250::I2CWrite(byte reg, byte regValue) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(regValue);
  Wire.endTransmission();
}

