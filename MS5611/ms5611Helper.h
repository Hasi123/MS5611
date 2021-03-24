/*
  How to use:
  Init once in Setup() with msInit();

  msStartMeasure(msType) //msType==0 is temperature, else pressure // have to call temperature every so often
  delay(MS5611_CONV_DELAY)
  float alt = msComputeAltitude() //or msComputePressure() or msComputeTemperature() depending on what you need
*/

#ifndef MS5611HELPER_H
#define MS5611HELPER_H

#include <Arduino.h>

#define MS5611_ADDRESS_CSB_LOW     0x77 //address pin low (GND)
#define MS5611_ADDRESS_CSB_HIGH    0x76 //address pin high (VCC)
#ifndef MS5611_DEFAULT_ADDRESS
#define MS5611_DEFAULT_ADDRESS     MS5611_ADDRESS_CSB_LOW
#endif

#define MS5611_RESET        0x1E
#define MS5611_READ_PROM    0xA2
#define MS5611_CONV_D1      0x48 //OSR=4096
#define MS5611_CONV_D2      0x58 //OSR=4096
#define MS5611_ADC_READ     0x00

#define MS5611_RESET_DELAY  3
#define MS5611_CONV_DELAY   10

//values for calculations
#define MS5611_BASE_SEA_PRESSURE 1013.25

//functions to call
void msInit(void);
void msStartMeasure(uint8_t msType);
//void msGetMeasure(void);
float msComputeAltitude(void);
float msComputePressure(void);
float msComputeTemperature(void);

#endif
