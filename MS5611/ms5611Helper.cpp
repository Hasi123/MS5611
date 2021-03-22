#include <Arduino.h>
#include "ms5611Helper.h"
#include "I2Cdev.h"
//Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//internal variables
const unsigned char msAddr = MS5611_DEFAULT_ADDRESS;
uint16_t msCoeffs[6];
double temperature, pressure;
bool msCurrentType;
uint32_t d1, d2;

//I2C functions
bool sendCMD(unsigned char devAddr, unsigned char cmd) {
  return I2Cdev::writeBytes(devAddr, cmd, 0, NULL);
}

uint16_t readWord(unsigned char devAddr, unsigned char regAddr) {
  uint16_t buff;
  I2Cdev::readWord(devAddr, regAddr, &buff);
  return buff;
}

uint32_t read24(unsigned char devAddr, unsigned char regAddr) {
  uint8_t buff[3];
  I2Cdev::readBytes(devAddr, regAddr, 3, buff);
  uint32_t msVal = ((uint32_t)buff[0] << 16) | ((uint32_t)buff[1] << 8) | buff[2];
  return msVal;
}

//issue command to start measurement
void msStartMeasure(uint8_t msType) {
  if (msType) { //get pressure
    sendCMD(msAddr, MS5611_CONV_D1);
    msCurrentType = 1;
  }
  else { //get temperature
    sendCMD(msAddr, MS5611_CONV_D2);
    msCurrentType = 0;
  }
}

//get measurement
void msGetMeasure(void) {
  if (msCurrentType) { //get pressure
    d1 = read24(msAddr, MS5611_ADC_READ);
  }
  else { //get temperature
    d2 = read24(msAddr, MS5611_ADC_READ);
  }
}

void msInit(void) {
  /* reset */
  sendCMD(msAddr, MS5611_RESET);
  delay(MS5611_RESET_DELAY);

  /* read calibration registers */
  for (uint8_t i = 0; i < 6; i++) {
    msCoeffs[i] = readWord(msAddr, MS5611_READ_PROM + (i * 2));
  }

  /* get first data */
  msStartMeasure(0); //temp
  delay(MS5611_CONV_DELAY);
  msGetMeasure();
  msStartMeasure(1); //pressure
  delay(MS5611_CONV_DELAY);
  msGetMeasure();
}

void computeMeasures(void) {
  /* compute temperature */
  int32_t dt, temp;

  int32_t c5s = msCoeffs[4];
  c5s <<= 8;
  dt = d2 - c5s;

  int64_t c6s = msCoeffs[5];
  c6s *= dt;
  c6s >>= 23;

  temp = 2000 + c6s;

  /* compute compensation */
  int64_t off, sens;

  /* offset */
  int64_t c2d = msCoeffs[1];
  c2d <<=  16;

  int64_t c4d = msCoeffs[3];
  c4d *= dt;
  c4d >>= 7;

  off = c2d + c4d;

  /* sens */
  int64_t c1d = msCoeffs[0];
  c1d <<= 15;

  int64_t c3d = msCoeffs[2];
  c3d *= dt;
  c3d >>= 8;

  sens = c1d + c3d;

  /* second order compensation */
  int64_t t2, off2, sens2;

  if ( temp < 2000 ) {
    t2 = dt;
    t2 *= t2;
    t2 >>= 31;

    off2 = temp - 2000;
    off2 *= off2;
    off2 *= 5;
    sens2 = off2;
    off2 >>= 1;
    sens2 >>= 2;

    if ( temp < -1500 ) {
      int64_t dtemp = temp + 1500;
      dtemp *= dtemp;
      off2 += 7 * dtemp;
      dtemp *= 11;
      dtemp >>= 1;
      sens2 += dtemp;
    }
    temp = temp - t2;
    off = off - off2;
    sens = sens - sens2;
  }

  /* compute pressure */
  int64_t p;

  p = d1 * sens;
  p >>= 21;
  p -= off;
  //p >>= 15 !!! done with doubles, see below

  /* save result */
  temperature = (double)temp / 100.0;
  pressure = ((double)p / (double)((uint16_t)1 << 15)) / 100.0;
}

double msComputeAltitude(void) {
  msGetMeasure();
  computeMeasures();
  double alti;
  alti = pow((pressure / (MS5611_BASE_SEA_PRESSURE)), 0.1902949572); //0.1902949572 = 1/5.255
  alti = (1 - alti) * (288.15 / 0.0065);
  return alti;
}

double msComputeTemperature(void) {
  msGetMeasure();
  computeMeasures();
  return temperature;
}
