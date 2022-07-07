/*
TTGO T-Beam Balloon Tracker
Copyright (C) 2022 by @goshawk22

Forked from:
TTGO T-Beam Mapper for Helium
Copyright (C) 2021 by @Max_Plastix

This code requires LMIC library by Matthijs Kooijman
https://github.com/matthijskooijman/arduino-lmic

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
#pragma once

// -----------------------------------------------------------------------------
// CONFIGURATION
// Stuff you might reasonably want to change is here:
// -----------------------------------------------------------------------------

// All time settings here are SECONDS

// Confirmed packets (ACK request) conflict with the function of a Mapper and should not normally be enabled.
// In areas of reduced coverage, the Mapper will try to send each packet six or more times with different SF/DR.
// This causes irregular results and the location updates are infrequent, unpredictable, and out of date.
// (0 means never, 1 means always, 2 every-other-one..)
#define LORAWAN_CONFIRMED_EVERY 0  // Request Confirmation message every N Uplinks 

// How many acks can fail before we reboot. If this value is exceeded, it it likely that packets aren't being received due to some unknown bug.
// To ensure we don't lose too much data, reboot and rejoin.
#define ACK_FAIL_THRESHOLD 3

// Spreading Factor (Data Rate) determines how long each 11-byte Mapper Uplink is on-air, and how observable it is.
// SF10 is about two seconds per packet, and the highest range, while SF7 is a good compromise
// for moving vehicles and reasonable mapping observations.
#define LORAWAN_SF DR_SF7  // Spreading factor (recommended DR_SF7 for network map purposes)
#define LORAWAN_SF_PING DR_SF12

// There are some extra non-Mapper Uplink messages we can send, but there's no good way to avoid sending these
// to all Integrations from the Decoder.  This causes (normal) Error messages on the Console because Mapper will throw
// them out for having no coordinates.  It doesn't hurt anything, as they are correctly filtered by the Decoder, but if
// you don't like seeing Integration Errors, then set these to 0.  Set these to 1 for extra non-mapper messages.
#ifndef SEND_GPSLOST_UPLINKS
#define SEND_GPSLOST_UPLINKS 1  // GPS Lost messages
#endif
#ifndef SEND_STATUS_UPLINKS
#define SEND_STATUS_UPLINKS  1  // Booted message
#endif

// -----------------------------------------------------------------------------
// Less common Configuration iteams
// -----------------------------------------------------------------------------

// Select which T-Beam board is being used. Only uncomment one.
//#define T_BEAM_V07  // AKA Rev0 (first board released) UNTESTED!  Expect bugs.
#define T_BEAM_V10  // AKA Rev1 (second board released), this is the common "v1.1"

#define DEBUG_PORT Serial   // Serial debug port
#define SERIAL_BAUD 115200  // Serial debug baud rate (note that bootloader is fixed at 115200)

// Never enable ADR on Mappers because they are moving, so we don't want to adjust
// anything based on packet reception.
#define LORAWAN_ADR 0  // Do not enable ADR

// If you are having difficulty sending messages to TTN after the first successful send,
// uncomment the next option and experiment with values (~ 1 - 5)
#define CLOCK_ERROR             5

// Whether or not to use stored LoRaWAN Keys for joining
#define JOIN_FROM_SCRATCH       1

// -----------------------------------------------------------------------------
// Timing
// -----------------------------------------------------------------------------

#define TX_INTERVAL   (0.5 * 60) // How often to transmit a packet

// -----------------------------------------------------------------------------
// DEBUG
// -----------------------------------------------------------------------------
#ifdef DEBUG_PORT
#define DEBUG_MSG(...) DEBUG_PORT.printf(__VA_ARGS__)
#else
#define DEBUG_MSG(...)
#endif

// Verbose LoRa message callback reporting
#define DEBUG_LORA_MESSAGES

// -----------------------------------------------------------------------------
// Custom messages
// -----------------------------------------------------------------------------

#define EV_QUEUED 100
#define EV_PENDING 101
#define EV_ACK 102
#define EV_RESPONSE 103

// -----------------------------------------------------------------------------
// General
// -----------------------------------------------------------------------------
// Wiring for I2C Sensors:
//
#define I2C_SDA 21
#define I2C_SCL 22

#define MIDDLE_BUTTON_PIN 38  // Middle button SW5, BUTTON0, GPIO38.  Low active

#define RED_LED 4  // GPIO4 on T-Beam v1.1

// -----------------------------------------------------------------------------
// GPS
// -----------------------------------------------------------------------------

#define GPS_SERIAL_NUM 1     // SerialX
#define GPS_BAUDRATE 115200  // Make haste!  NMEA is big.. go fast
#define USE_GPS 1

#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#define GPS_INT 37  // 30ns accurate timepulse from Neo-6M pin 3

// -----------------------------------------------------------------------------
// LoRa SPI
// -----------------------------------------------------------------------------

#define SCK_GPIO 5
#define MISO_GPIO 19
#define MOSI_GPIO 27
#define NSS_GPIO 18
#define RESET_GPIO 14
#define DIO0_GPIO 26
#define DIO1_GPIO 33  // Note: not really used on this board
#define DIO2_GPIO 32  // Note: not really used on this board

// -----------------------------------------------------------------------------
// AXP192 (Rev1-specific options)
// -----------------------------------------------------------------------------

#define GPS_POWER_CTRL_CH 3
#define LORA_POWER_CTRL_CH 2
#define PMU_IRQ 35



// -----------------------------------------------------------------------------
// Sensors
// -----------------------------------------------------------------------------
# define I2C_BME280_ADDRESS 0x76
# define I2C_LTR390_ADDRESS 0x53