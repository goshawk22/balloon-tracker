/*
 Helium Balloon Tracker build for LilyGo TTGO T-Beam v1.1 boards.
 Copyright (C) 2022 by goshawk22

Forked from:
TTGO T-Beam Mapper for Helium
Copyright (C) 2021 by @Max_Plastix

 This code comes from a number of developers and earlier efforts,
  including:  Fizzy, longfi-arduino, Kyle T. Gabriel, and Xose Pérez

 GPL makes this all possible -- continue to modify, extend, and share!
 */

/*
  Main module

  # Modified by Kyle T. Gabriel to fix issue with incorrect GPS data for
  TTNMapper

  Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <lmic.h>

#include <SPI.h>

#include "configuration.h"
#include "gps.h"
#include "ttn.h"
#include "power.h"
#include "utils.h"
#include "sensors.h"

// Just so we can disable it to save power
#include <BluetoothSerial.h>
#include <WiFi.h>
#include <esp_bt.h>

#define FPORT_GPS 2  // FPort for Uplink messages -- must match Helium Console Decoder script!
#define FPORT_STATUS 5
#define FPORT_GPSLOST 6

// Defined in ttn.cpp
void ttn_register(void (*callback)(uint8_t message));

unsigned long int last_send_ms = 0;     // Time of last uplink
double last_send_lat = 0;               // Last known location
double last_send_lon = 0;               //
uint32_t last_fix_time = 0;

unsigned int tx_interval_s = TX_INTERVAL;  // TX_INTERVAL

// Return status from mapper uplink, since we care about the flavor of the failure
enum mapper_uplink_result { 
  MAPPER_UPLINK_SUCCESS,
  MAPPER_UPLINK_BADFIX,
  MAPPER_UPLINK_NOLORA,
  MAPPER_UPLINK_NOTYET
};

bool isJoined = false;

// Buffer for Payload frame
static uint8_t txBuffer[22];

// Buffer for Serial output
char msgBuffer[40];

unsigned long int ack_req = 0;
unsigned long int ack_rx = 0;

static boolean booted = false;

boolean send_uplink(uint8_t *txBuffer, uint8_t length, uint8_t fport, boolean confirmed) {
  unsigned long int now = millis();

  if (confirmed) {
    Serial.println("ACK requested");
    ack_req++;
  }

  // send it!
  if (!ttn_send(txBuffer, length, fport, confirmed)) {
    Serial.println("Surprise send failure!");
    return false;
  }
  last_send_ms = now;
  return true;
}

// Store Lat & Long in six bytes of payload
void pack_lat_lon(double lat, double lon) {
  uint32_t LatitudeBinary;
  uint32_t LongitudeBinary;

  LatitudeBinary = ((lat + 90) / 180.0) * 16777215;
  LongitudeBinary = ((lon + 180) / 360.0) * 16777215;

  txBuffer[0] = (LatitudeBinary >> 16) & 0xFF;
  txBuffer[1] = (LatitudeBinary >> 8) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;
  txBuffer[3] = (LongitudeBinary >> 16) & 0xFF;
  txBuffer[4] = (LongitudeBinary >> 8) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;
}

void pack_bme280() {
  if (bme280_alive) {
    int bmeTemp = bme.readTemperature() * 100;
    int bmePressure = bme.readPressure();
    int bmeHumidity = bme.readHumidity() * 100;

    txBuffer[13] = (bmeTemp >> 8) & 0xFF;
    txBuffer[14] = bmeTemp & 0xFF;
    txBuffer[15] = (bmePressure >> 16) & 0xFF;
    txBuffer[16] = (bmePressure >> 8) & 0xFF;
    txBuffer[17] = bmePressure & 0xFF;
    txBuffer[18] = (bmeHumidity >> 8) & 0xFF;
    txBuffer[19] = bmeHumidity & 0xFF;

  } else {
    // Obviously bad values to show something went wrong
    txBuffer[13] = -100;
    txBuffer[14] = -100;
    txBuffer[15] = -100;
    txBuffer[16] = -100;
    txBuffer[17] = -100;
    txBuffer[18] = -100;
    txBuffer[19] = -100;

  }
}

void pack_ltr390() {
  if (ltr390_alive) {
    int uv = ltr.readUVS();

    txBuffer[20] = (uv >> 8) & 0xFF;
    txBuffer[21] = uv & 0xFF;
  } else {
    // Obviously bad values to show something went wrong
    txBuffer[20] = -100;
    txBuffer[21] = -100;
  }
}

uint8_t battery_byte(void) {
  uint16_t batteryVoltage = ((float_t)((float_t)(axp.getBattVoltage()) / 10.0) + .5);
  return (uint8_t)((batteryVoltage - 200) & 0xFF);
}

// Prepare a packet with GPS and sensor data
void build_full_packet() {
  double lat;
  double lon;
  uint16_t altitudeGps;
  uint8_t sats;
  uint16_t speed;
  uint16_t minutes_lost = (millis() - last_fix_time) / 1000 / 60;
  unsigned long int uptime = millis() / 1000 / 60;

  lat = tGPS.location.lat();
  lon = tGPS.location.lng();
  pack_lat_lon(lat, lon);
  altitudeGps = (uint16_t)tGPS.altitude.meters();
  speed = (uint16_t)tGPS.speed.kmph();  // convert from double
  if (speed > 255)
    speed = 255;  // don't wrap around.
  sats = tGPS.satellites.value();

  sprintf(msgBuffer, "Lat: %f, ", lat);
  Serial.println(msgBuffer);
  sprintf(msgBuffer, "Long: %f, ", lon);
  Serial.println(msgBuffer);
  sprintf(msgBuffer, "Alt: %f, ", tGPS.altitude.meters());
  Serial.println(msgBuffer);
  sprintf(msgBuffer, "Sats: %d", sats);
  Serial.println(msgBuffer);

  txBuffer[6] = (altitudeGps >> 8) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  txBuffer[8] = speed & 0xFF;
  txBuffer[9] = battery_byte();

  txBuffer[10] = sats & 0xFF;
  txBuffer[11] = uptime & 0xFF;
  txBuffer[12] = minutes_lost & 0xFF;

  pack_bme280();
  pack_ltr390();
}

bool status_uplink(void) {
  if (!SEND_STATUS_UPLINKS)
    return false;

  pack_lat_lon(last_send_lat, last_send_lon);

  unsigned long int uptime = millis() / 1000 / 60;;
  txBuffer[6] = battery_byte();
  txBuffer[7] = uptime & 0xFF; // Time since booted
  Serial.printf("Tx: STATUS %lu \n", uptime);
  return send_uplink(txBuffer, 8, FPORT_STATUS, 0);
}

bool gpslost_uplink(void) {
  uint16_t minutes_lost;
  unsigned long int uptime;

  uptime = millis() / 1000 / 60;
  minutes_lost = (millis() - last_fix_time) / 1000 / 60;
  pack_lat_lon(last_send_lat, last_send_lon);
  txBuffer[6] = 0; // Obviously wrong value as we couldn't get a fix. Placeholder to make structure same as full packet
  txBuffer[7] = 0; //Another obviously wrong value
  txBuffer[8] = 0; //Another obviously wrong value
  txBuffer[9] = battery_byte();
  txBuffer[10] = tGPS.satellites.value() & 0xFF;
  txBuffer[11] = uptime & 0xFF;
  txBuffer[12] = minutes_lost & 0xFF;

  pack_bme280();
  pack_ltr390();
  Serial.printf("Tx: GPSLOST %d\n", minutes_lost);
  return send_uplink(txBuffer, 22, FPORT_GPSLOST, 0);
}

// Send a packet, if one is warranted
enum mapper_uplink_result mapper_uplink() {
  double now_lat = tGPS.location.lat();
  double now_lon = tGPS.location.lng();

  // Here we try to filter out bogus GPS readings.
  if (!(tGPS.location.isValid() && tGPS.time.isValid() && tGPS.satellites.isValid() && tGPS.hdop.isValid() &&
        tGPS.altitude.isValid() && tGPS.speed.isValid()))
    return MAPPER_UPLINK_BADFIX;

  // Filter out any reports while we have low satellite count.  The receiver can old a fix on 3, but it's poor.
  if (tGPS.satellites.value() < 4)
    return MAPPER_UPLINK_BADFIX;

  // With the exception of a few places, a perfectly zero lat or long probably means we got a bad reading
  if (now_lat == 0.0 || now_lon == 0.0)
    return MAPPER_UPLINK_BADFIX;

  // Don't attempt to send or update until we join Helium
  if (!isJoined)
    return MAPPER_UPLINK_NOLORA;

  // LoRa is not ready for a new packet, maybe still sending the last one.
  if (!LMIC_queryTxReady())
    return MAPPER_UPLINK_NOLORA;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
    return MAPPER_UPLINK_NOLORA;

  // prepare the LoRa frame
  build_full_packet();

  // Want an ACK on this one?
  bool confirmed = (LORAWAN_CONFIRMED_EVERY > 0) && (ttn_get_count() % LORAWAN_CONFIRMED_EVERY == 0);

  // Send it!
  if (!send_uplink(txBuffer, 22, FPORT_GPS, confirmed))
    return MAPPER_UPLINK_NOLORA;

  last_send_lat = now_lat;
  last_send_lon = now_lon;

  return MAPPER_UPLINK_SUCCESS;  // We did it!
}

// LoRa message event callback
void lora_msg_callback(uint8_t message) {
  static boolean seen_joined = false, seen_joining = false;
#ifdef DEBUG_LORA_MESSAGES
  if (EV_JOIN_TXCOMPLETE == message)
    Serial.println("# JOIN_TXCOMPLETE");
  if (EV_TXCOMPLETE == message)
    Serial.println("# TXCOMPLETE");
  if (EV_RXCOMPLETE == message)
    Serial.println("# RXCOMPLETE");
  if (EV_RXSTART == message)
    Serial.println("# RXSTART");
  if (EV_TXCANCELED == message)
    Serial.println("# TXCANCELED");
  if (EV_TXSTART == message)
    Serial.println("# TXSTART");
  if (EV_JOINING == message)
    Serial.println("# JOINING");
  if (EV_JOINED == message)
    Serial.println("# JOINED");
  if (EV_JOIN_FAILED == message)
    Serial.println("# JOIN_FAILED");
  if (EV_REJOIN_FAILED == message)
    Serial.println("# REJOIN_FAILED");
  if (EV_RESET == message)
    Serial.println("# RESET");
  if (EV_LINK_DEAD == message)
    Serial.println("# LINK_DEAD");
  if (EV_ACK == message)
    Serial.println("# ACK");
  if (EV_PENDING == message)
    Serial.println("# PENDING");
  if (EV_QUEUED == message)
    Serial.println("# QUEUED");
#endif

  /* This is confusing because JOINED is sometimes spoofed and comes early */
  if (EV_JOINED == message)
    seen_joined = true;
  if (EV_JOINING == message)
    seen_joining = true;
  if (!isJoined && seen_joined && seen_joining) {
    isJoined = true;
    ttn_set_sf(LORAWAN_SF);  // SF is left at SF that had a successful join so change to preferred SF
  }

  if (EV_ACK == message) {
    ack_rx++;
    Serial.printf("ACK! %lu / %lu\n", ack_rx, ack_req);
  }

  if (EV_RXCOMPLETE == message || EV_RESPONSE == message) {
    size_t len = ttn_response_len();
    uint8_t data[len];
    uint8_t port;
    ttn_response(&port, data, len);

    Serial.printf("Downlink on port: %d = ", port);
    for (int i = 0; i < len; i++) {
      if (data[i] < 16)
        Serial.print('0');
      Serial.print(data[i], HEX);
    }
    Serial.println();
  }
}

void setup() {
  // Debug
#ifdef DEBUG_PORT
  DEBUG_PORT.begin(SERIAL_BAUD);
#endif

  wakeup();

  // Make sure WiFi and BT are off
  // WiFi.disconnect(true);
  WiFi.mode(WIFI_MODE_NULL);
  btStop();

  Wire.begin(I2C_SDA, I2C_SCL);
  scanI2Cdevice();

  axp192Init();
  BMEsensorInit();
  LTRsensorInit();

  // GPS sometimes gets wedged with no satellites in view and only a power-cycle
  // saves it. Here we turn off power and the delay in screen setup is enough
  // time to bonk the GPS
  axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // GPS power off

  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);  // Off

  // GPS power on, so it has time to setttle.
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);

  // Helium setup
  if (!ttn_setup()) {
    // Something has gone wrong, and now the tracker can't do anything. Restart and hope the problem fixes itself.
    ESP.restart();
  }

  ttn_register(lora_msg_callback);
  ttn_join();
  ttn_adr(LORAWAN_ADR);

  // Might have to add a longer delay here for GPS boot-up.  Takes longer to sync if we talk to it too early.
  delay(100);
  gps_setup(true);  // Init GPS baudrate and messages

  // This is bad.. we can't find the AXP192 PMIC, so no menu key detect:
  if (!axp192_found)
    Serial.println("** Missing AXP192! **\n");

  booted = 1;
}

uint32_t woke_time_ms = 0;
uint32_t woke_fix_count = 0;

void loop() {
  static uint32_t last_fix_count = 0;
  uint32_t now_fix_count;
  uint32_t now = millis();

  gps_loop(0);  // Update GPS
  now_fix_count = tGPS.sentencesWithFix();          // Did we get a new fix?
  if (now_fix_count != last_fix_count) {
    last_fix_count = now_fix_count;
    last_fix_time = now;  // Note the time of most recent fix
  }

  ttn_loop();

  if (booted) {
    if (isJoined) {
      status_uplink();
      booted = 0; // So we don't send status uplink again
    }
  }

  // Check if sensors have failed
  if (!checkI2Cdevice(I2C_BME280_ADDRESS)) {
    bme280_alive = false;
  } else {
    if (!bme280_alive) {
      //The sensor was dead but is now alive
      BMEsensorInit();
    }
    bme280_alive = true;
  }
  if (!checkI2Cdevice(I2C_LTR390_ADDRESS)) {
    ltr390_alive = false;
  } else {
    if (!ltr390_alive) {
      LTRsensorInit();
    }
    ltr390_alive = true;
  }
  
  if (now - last_send_ms > tx_interval_s * 1000) {
    Serial.println("** TIME");
    if (mapper_uplink() == MAPPER_UPLINK_BADFIX) {
      gpslost_uplink(); // No GPS so send ghost uplink
    }
  }
}
