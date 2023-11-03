#include <Arduino.h>
#include "BerryIMU_v3.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  BerryIMU_v3 BerryIMU;
  
  while (true) {
     BerryIMU.IMU_read();
     Serial.println(BerryIMU.alt);

     delay(100);
  } 
}

void loop() {

}