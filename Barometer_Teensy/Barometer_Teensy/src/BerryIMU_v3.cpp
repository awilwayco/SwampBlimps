/*
 BerryIMU_v3.cpp 
*/

#include "BerryIMU_v3.h"
#include <Wire.h> //I2C Library
#include "LSM6DSL.h"
#include "LIS3MDL.h"
#include "BM388.h"

BerryIMU_v3::BerryIMU_v3(){
  Wire.begin();        // Initialise i2c
  Wire.setClock(100000);  //Change i2c bus speed to 400kHz

  //Indoor navigation configuration
  writeTo(BM388_ADDRESS,PWR_CTRL, 0b00110011);     // Enables pressure sensor, Enables temperature sensor, Normal mode
  writeTo(BM388_ADDRESS,OSR, 0b00001100);                 // x16 oversampling pressure measurement, x2 oversampling temp measurement
  writeTo(BM388_ADDRESS,ODR, 0x03);                 // Output data rate 25Hz
  writeTo(BM388_ADDRESS,CONFIG, 0b00000010);      // IIR filter coefficient of 63
 
  //Loading the calibration values
  readFrom(BM388_ADDRESS, NVM_PAR_T1_LSB, 21, buff_calib);
  float NVM_PAR_T1_val = (uint16_t)(buff_calib[0] | (buff_calib[1] << 8));
  float NVM_PAR_T2_val = (uint16_t)(buff_calib[2] | (buff_calib[3] << 8));
  float NVM_PAR_T3_val = (int8_t)buff_calib[4];
  float NVM_PAR_P1_val = (int16_t)(buff_calib[5] | (buff_calib[6] << 8));
  float NVM_PAR_P2_val = (int16_t)(buff_calib[7] | (buff_calib[8] << 8));
  float NVM_PAR_P3_val = (int8_t)buff_calib[9];
  float NVM_PAR_P4_val = (int8_t)buff_calib[10];
  float NVM_PAR_P5_val = (uint16_t)(buff_calib[11] | (buff_calib[12] << 8));
  float NVM_PAR_P6_val = (uint16_t)(buff_calib[13] | (buff_calib[14] << 8));
  float NVM_PAR_P7_val = (int8_t)buff_calib[15];
  float NVM_PAR_P8_val = (int8_t)buff_calib[16];
  float NVM_PAR_P9_val = (int16_t)(buff_calib[17] | (buff_calib[18] << 8));
  float NVM_PAR_P10_val = (int8_t)buff_calib[19];
  float NVM_PAR_P11_val = (int8_t)buff_calib[20];

  //Floating point compensation
  PAR_T1 = NVM_PAR_T1_val / pow(2, -8);
  PAR_T2 = NVM_PAR_T2_val / pow(2, 30);
  PAR_T3 = NVM_PAR_T3_val / pow(2, 48);
  PAR_P1 = (NVM_PAR_P1_val - pow(2, 14)) / pow(2, 20);
  PAR_P2 = (NVM_PAR_P2_val - pow(2, 14)) / pow(2, 29);
  PAR_P3 = NVM_PAR_P3_val / pow(2, 32);
  PAR_P4 = NVM_PAR_P4_val / pow(2, 37);
  PAR_P5 = NVM_PAR_P5_val / pow(2, -3);
  PAR_P6 = NVM_PAR_P6_val / pow(2, 6);
  PAR_P7 = NVM_PAR_P7_val / pow(2, 8);
  PAR_P8 = NVM_PAR_P8_val / pow(2, 15);
  PAR_P9 = NVM_PAR_P9_val / pow(2, 48);
  PAR_P10 = NVM_PAR_P10_val / pow(2, 48);
  PAR_P11 = NVM_PAR_P11_val / pow(2, 65);

  //**********************************************************************************************************************************************************
}

void BerryIMU_v3::IMU_read(){
  //--------------------------------------------------------------------------------------
  //Barometer and Temperature Sensor Output
  //Starts with the PRESS_XLSB_7_0 output register then will read the remainig five
  readFrom(BM388_ADDRESS, PRESS_XLSB_7_0, 6, buff);

  // Last 3 bytes are the temperature XLSB, LSB, MSB
  float tempRaw = (int)(buff[3] | (buff[4] << 8) | (buff[5] << 16));
  comp_temp = temp_compensation(tempRaw);
  //Serial.println(comp_temp); //Temperature in deg C

  // First 3 bytes are the pressure XLSB, LSB, MSB
  // Bit shift done by 256^3 = 16777216 and 16777216/3 = 5592405
  float pressRaw = (int)(buff[0] | (buff[1] << 8) | (buff[2] << 16));
  comp_press = press_compensation(pressRaw, comp_temp);
  //Serial.println(comp_press); //Pressure in Pa

  //Altitude (in meters)
  //https://www.circuitbasics.com/set-bmp180-barometric-pressure-sensor-arduino/
  //Sets the reference pressure (therefore setting the reference height)
  if(ref_pressure_found){
    ref_ground_press = comp_press;
    ref_pressure_found = false;
  }
  alt = comp_press; //In meters

}

float BerryIMU_v3::temp_compensation(float raw_temperature) {
  float partial_data1 = raw_temperature - PAR_T1;
  float partial_data2 = partial_data1 * PAR_T2;
  float comp_temp = partial_data2 + (partial_data1 * partial_data1) * PAR_T3;
  return comp_temp;
}

float BerryIMU_v3::press_compensation(float raw_pressure, float comp_temp) {
  float partial_data1 = PAR_P6 * comp_temp;
  float partial_data2 = PAR_P7 * (comp_temp * comp_temp);
  float partial_data3 = PAR_P8 * (comp_temp * comp_temp * comp_temp);
  float partial_out1 = PAR_P5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = PAR_P2 * comp_temp;
  partial_data2 = PAR_P3 * (comp_temp * comp_temp);
  partial_data3 = PAR_P4 * (comp_temp * comp_temp * comp_temp);
  float partial_out2 = raw_pressure * (PAR_P1 + partial_data1 + partial_data2 + partial_data3);
  partial_data1 = raw_pressure * raw_pressure;
  partial_data2 = PAR_P9 + PAR_P10 * comp_temp;
  partial_data3 = partial_data1 * partial_data2;
  float partial_data4 = partial_data3 + (raw_pressure * raw_pressure * raw_pressure) * PAR_P11;
  float comp_press = partial_out1 + partial_out2 + partial_data4;

  return comp_press;
}

void BerryIMU_v3::writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device); //start transmission to device
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write (writing byte to control register to change sensor parameters)
  Wire.endTransmission(); //end transmission
}

//Used for Accelerometer,Gyroscope, Magnetometer, and Barometer
void BerryIMU_v3::readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission

  Wire.beginTransmission(device); //start transmission to device (initiate again)
  Wire.requestFrom(device, num);    // request 6 bytes from device (2 for each axis's MSB and LSB)

  int i = 0;
  while (Wire.available())   //device may send less than requested (abnormal)
  {
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}
