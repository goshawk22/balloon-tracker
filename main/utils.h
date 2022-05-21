#pragma once

#include <Arduino.h>

extern bool axp192_found;
extern bool bme280_found;
extern bool ltr390_found;

extern bool bme280_alive;
extern bool ltr390_alive;

void scanI2Cdevice(void);
bool checkI2Cdevice(int addr);