#include "ms5611Helper.h"

void setup() {
  Serial.begin(57600);
  msInit();
}

void loop() {
  for (uint8_t i = 0; i < 50; i++) {
    msStartMeasure(i);
    delay(10);
    Serial.println(msComputeAltitude());
  }
}
