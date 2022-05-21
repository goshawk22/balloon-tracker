#pragma once

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_LTR390.h"

extern Adafruit_BME280 bme; // use I2C interface

extern Adafruit_LTR390 ltr;

void BMEsensorInit();
void LTRsensorInit();