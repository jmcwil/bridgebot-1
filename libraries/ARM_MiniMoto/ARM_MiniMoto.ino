/*  This is a rewrite of the motor drivers using the Wire library.
 *   Since register definitions are different, we will try to simplify 
 *   the issue by using the prewritten code functions.
 */

#include <Wire.h>
#include "ARM_MiniMoto.h"

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

MiniMoto motor0(0x60); // ignore comments here for now...A1 = 1, A0 = clear   
MiniMoto motor1(0x65); // A1 = 1, A0 = 1 (default)

#define FAULTn 16  // Pin used for fault detection


void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Hello World"); //test serial connection
  pinMode(FAULTn, INPUT);
  pinMode(13,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  motor0.drive(80);
  motor1.drive(80);
  Serial.println("Driving");
  blink(3000);
  motor0.stop();
  motor1.stop();
  Serial.println("Stopped");
  delay(1000);
  motor0.drive(-80);
  motor1.drive(-80);
  blink(2000);
  motor0.stop();
  motor1.stop();
  delay(1000);
  
}

void blink(int duration) {
  digitalWrite(13, HIGH);
  delay(duration);
  digitalWrite(13,LOW);
}



