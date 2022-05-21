#include "utils.h"

#include <Arduino.h>
#include <Wire.h>
#include <axp20x.h>

#include "configuration.h"

bool axp192_found = false;
bool bme280_found = false;
bool ltr390_found = false;

// Sensor status
bool bme280_alive;
bool ltr390_alive;

void scanI2Cdevice(void) {
  byte err, addr;
  int nDevices = 0;
  Serial.println("Scanning for I2C devices ...");
  for(addr = 0x01; addr < 0x7f; addr++){
    Wire.beginTransmission(addr);
    err = Wire.endTransmission();
    if (err == 0) {
      nDevices++;

      Serial.printf("I2C device found at address 0x%02X\n", addr);

      if (addr == AXP192_SLAVE_ADDRESS) {
        axp192_found = true;
        Serial.println("AXP192 PMU");
      }

      else if (addr == I2C_BME280_ADDRESS) {
        bme280_found = true;
        bme280_alive = true;
        Serial.println("BME280 Found");
      }

      else if (addr == I2C_LTR390_ADDRESS) {
        ltr390_found = true;
        ltr390_alive = true;
        Serial.println("LTR390 Found");
      }


    } else if (err == 4) {
      Serial.print("Unknow i2c device at 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found!\n");
  /* else  Serial.println("done\n"); */
}

bool checkI2Cdevice(int addr) {
  byte err;
  Wire.beginTransmission(addr);
  err = Wire.endTransmission();
  if (err == 0) {
    return true;
  } else {
    return false;
  }
}