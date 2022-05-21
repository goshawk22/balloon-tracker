#pragma once

#include <Arduino.h>
#include <axp20x.h>

extern AXP20X_Class axp;

int axp_charge_to_ma(int set);
void axp192Init();
void wakeup();