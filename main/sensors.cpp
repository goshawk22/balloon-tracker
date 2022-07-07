#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_LTR390.h"

#include "configuration.h"
#include "utils.h"

Adafruit_BME280 bme; // use I2C interface

Adafruit_LTR390 ltr = Adafruit_LTR390();


unsigned bme280_status;
unsigned ltr390_status;

bool BMEsensorInit() {
    if (bme280_found) {
        bme280_status = bme.begin(I2C_BME280_ADDRESS);
        if (!bme280_status) {
            Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
            Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
            return false;
        }
        return true;
    }
    return false;
}

bool LTRsensorInit() {
    if (ltr390_found) {
        ltr390_status = ltr.begin();
        if (!ltr390_status) {
            Serial.println("Couldn't find LTR390 sensor!");
            return false;
        }
        ltr.setMode(LTR390_MODE_UVS);
        if (ltr.getMode() == LTR390_MODE_ALS) {
            Serial.println("In ALS mode");
        } else {
            Serial.println("In UVS mode");
        }
        ltr.setGain(LTR390_GAIN_3);
        switch (ltr.getGain()) {
            case LTR390_GAIN_1: Serial.printf("LTR390 Gain: %d \n", 1); break;
            case LTR390_GAIN_3: Serial.printf("LTR390 Gain: %d \n", 3); break;
            case LTR390_GAIN_6: Serial.printf("LTR390 Gain: %d \n", 6); break;
            case LTR390_GAIN_9: Serial.printf("LTR390 Gain: %d \n", 9); break;
            case LTR390_GAIN_18: Serial.printf("LTR390 Gain: %d \n", 18); break;
        }
        ltr.setResolution(LTR390_RESOLUTION_20BIT);
        switch (ltr.getResolution()) {
            case LTR390_RESOLUTION_13BIT: Serial.printf("LTR390 Resolution: %d \n", 13); break;
            case LTR390_RESOLUTION_16BIT: Serial.printf("LTR390 Resolution: %d \n", 16); break;
            case LTR390_RESOLUTION_17BIT: Serial.printf("LTR390 Resolution: %d \n", 17); break;
            case LTR390_RESOLUTION_18BIT: Serial.printf("LTR390 Resolution: %d \n", 18); break;
            case LTR390_RESOLUTION_19BIT: Serial.printf("LTR390 Resolution: %d \n", 19); break;
            case LTR390_RESOLUTION_20BIT: Serial.printf("LTR390 Resolution: %d \n", 20); break;
        }
        ltr.setThresholds(100, 1000);
        ltr.configInterrupt(true, LTR390_MODE_UVS);
        return true;
    }
    return false;
}