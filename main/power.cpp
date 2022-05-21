#include "power.h"

#include <Arduino.h>
#include <axp20x.h>

#include "configuration.h"
#include "utils.h"

AXP20X_Class axp;
bool pmu_irq = false;  // true when PMU IRQ pending

// the reason we booted this time
esp_sleep_source_t wakeCause;

//Boot count
RTC_DATA_ATTR int bootCount = 0;

/* The AXP library computes this incorrectly for AXP192.
   It's just a fixed mapping table from the datasheet */
int axp_charge_to_ma(int set) {
  switch (set) {
    case 0:
      return 100;
    case 1:
      return 190;
    case 2:
      return 280;
    case 3:
      return 360;
    case 4:
      return 450;
    case 5:
      return 550;
    case 6:
      return 630;
    case 7:
      return 700;
    case 8:
      return 780;
    case 9:
      return 880;
    case 10:
      return 960;
    case 11:
      return 1000;
    case 12:
      return 1080;
    case 13:
      return 1160;
    case 14:
      return 1240;
    case 15:
      return 1320;
    default:
      return -1;
  }
}

/**
  Initialize the AXP192 power manager chip.

  DCDC1 0.7-3.5V @ 1200mA max -> OLED
  If you turn the OLED off, it will drag down the I2C lines and block the bus from the AXP192 which shares it.
  Use SSD1306 sleep mode instead

  DCDC3 0.7-3.5V @ 700mA max -> ESP32 (keep this on!)
  LDO1 30mA -> "VCC_RTC" charges GPS tiny J13 backup battery
  LDO2 200mA -> "LORA_VCC"
  LDO3 200mA -> "GPS_VCC"
*/
void axp192Init() {
  if (axp192_found) {
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
      // Serial.println("AXP192 Begin PASS");
    } else {
      Serial.println("axp.begin() FAIL");
      axp192_found = false;
      return;
    }

    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);        // LORA radio 200mA "LORA_VCC"
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);        // GPS power 200mA "GPS_VCC"
    axp.setLDO3Voltage(3300);                          // Voltage for GPS Power.  (Neo-6 can take 2.7v to 3.6v)
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);       // Sensor power, 1200mA max "VCC_2.5V"
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);      // Unconnected
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);      // "EXTEN" pin, unused
    axp.setChargeControlCur(AXP1XX_CHARGE_CUR_550MA);  // Default 0x1000 = 780mA, more than we can get from USB

    // Flash the Blue LED until our first packet is transmitted
    axp.setChgLEDMode(AXP20X_LED_OFF);

    Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
    //Serial.printf("LDO1: %s\n", axp.isLDO1Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

    // Fire an interrupt on falling edge.  Note that some IRQs repeat/persist.
    pinMode(PMU_IRQ, INPUT);
    gpio_pullup_en((gpio_num_t)PMU_IRQ);
    attachInterrupt(
        PMU_IRQ, [] { pmu_irq = true; }, FALLING);

    // Configure REG 36H: PEK press key parameter set.  Index values for
    // argument!
    axp.setStartupTime(2);        // "Power on time": 512mS
    axp.setlongPressTime(2);      // "Long time key press time": 2S
    axp.setShutdownTime(2);       // "Power off time" = 8S
    axp.setTimeOutShutdown(1);    // "When key press time is longer than power off time, auto power off"
    axp.setVWarningLevel1(2950);  // These warning IRQs do not clear until charged, and inhibit other IRQs!
    axp.setVWarningLevel2(2900);  // We effectively disable them by setting them lower than we'd run

    Serial.printf("PMIC Temp %0.2f°C\n", axp.getTemp());
    Serial.printf("Battery: %0.3fv\n", axp.getBattVoltage() / 1000.0);
    Serial.printf("SysIPSOut: %0.3fv\n", axp.getSysIPSOUTVoltage() / 1000.0);
    Serial.printf("isVBUSPlug? %s\n", axp.isVBUSPlug() ? "Yes" : "No");
    Serial.printf("isChargingEnable? %s\n", axp.isChargeingEnable() ? "Yes" : "No");
    Serial.printf("ChargeControlCurrent: %d = %dmA\n", axp.getChargeControlCur(),
                  axp_charge_to_ma(axp.getChargeControlCur()));
    Serial.printf("Battery Charge Level: %d%%\n", axp.getBattPercentage());

    Serial.printf("WarningLevel1: %d mV\n", axp.getVWarningLevel1());
    Serial.printf("WarningLevel2: %d mV\n", axp.getVWarningLevel2());
    Serial.printf("PowerDown:     %d mV\n", axp.getPowerDownVoltage());

    Serial.printf("DCDC1Voltage: %d mV\n", axp.getDCDC1Voltage());
    Serial.printf("DCDC2Voltage: %d mV\n", axp.getDCDC2Voltage());
    Serial.printf("DCDC3Voltage: %d mV\n", axp.getDCDC3Voltage());
    Serial.printf("LDO2:         %d mV\n", axp.getLDO2Voltage());
    Serial.printf("LDO3:         %d mV\n", axp.getLDO3Voltage());
    Serial.printf("LDO4:         %d mV\n", axp.getLDO4Voltage());

    // Enable battery current measurements
    axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
    //    axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ |
    //    AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
    axp.enableIRQ(0xFFFFFFFFFF, 1);  // Give me ALL the interrupts you have.

    // @Kenny_PDY discovered that low-battery voltage inhibits detecting the menu button.
    // Disable these two IRQs until we figure out why it blocks the PEK button IRQs.
    // Low battery also seems to inhibit the USB present/lost signal we use to wake up.
    axp.enableIRQ(APX202_APS_LOW_VOL_LEVEL1_IRQ, 0);
    axp.enableIRQ(AXP202_APS_LOW_VOL_LEVEL2_IRQ, 0);

    // The Charging Current available is less than requested for battery charging.
    // Another Persistent IRQ.  Clear it after showing it once?
    // TODO: Show it every X minutes?  Adjust charge current request?
    axp.enableIRQ(AXP202_CHARGE_LOW_CUR_IRQ, 0);

    axp.clearIRQ();
  } else {
    Serial.println("AXP192 not found!");
  }
}

// Perform power on init that we do on each power on
void wakeup() {
  bootCount++;
  wakeCause = esp_sleep_get_wakeup_cause();

  Serial.printf("BOOT #%d!  cause:%d ext1:%08llx\n", bootCount, wakeCause, esp_sleep_get_ext1_wakeup_status());
}