#pragma once
#include "sdkconfig.h"

#define APP_DEVICE_NAME CONFIG_APP_DEVICE_NAME
#define APP_CONTROL_LOOP_INTERVAL CONFIG_APP_CONTROL_LOOP_INTERVAL

#define HW_BUTTON_PIN CONFIG_HW_BUTTON_PIN
#define HW_BUTTON_LEVEL CONFIG_HW_BUTTON_LEVEL
#if CONFIG_HW_BUTTON_PULL
#define HW_BUTTON_PULL 1
#else
#define HW_BUTTON_PULL 0
#endif
#define HW_BUTTON_PROVISION_MS CONFIG_HW_BUTTON_PROVISION_MS
#define HW_BUTTON_FACTORY_RESET_MS CONFIG_HW_BUTTON_FACTORY_RESET_MS

#define HW_VALVE_ENABLE CONFIG_HW_VALVE_ENABLE
#define HW_VALVE_POWER_PIN CONFIG_HW_VALVE_POWER_PIN

#define HW_DS18B20_ENABLE CONFIG_HW_DS18B20_ENABLE
#define HW_DS18B20_PIN CONFIG_HW_DS18B20_PIN

#define HW_SOIL_PROBE_ENABLE CONFIG_HW_SOIL_PROBE_ENABLE
#define HW_SOIL_PROBE_TOUCH_PAD CONFIG_HW_SOIL_PROBE_TOUCH_PAD
#define HW_SOIL_PROBE_MIN CONFIG_HW_SOIL_PROBE_MIN
#define HW_SOIL_PROBE_MAX CONFIG_HW_SOIL_PROBE_MAX
#define HW_SOIL_PROBE_LOW CONFIG_HW_SOIL_PROBE_LOW
#define HW_SOIL_PROBE_HIGH CONFIG_HW_SOIL_PROBE_HIGH

#define HW_WATER_LEVEL_ENABLE CONFIG_HW_WATER_LEVEL_ENABLE
#define HW_WATER_SENSOR_POWER_PIN CONFIG_HW_WATER_LEVEL_POWER_PIN
#define HW_WATER_LEVEL_ADC1_CHANNEL CONFIG_HW_WATER_LEVEL_ADC1_CHANNEL
#define HW_WATER_LEVEL_MIN CONFIG_HW_WATER_LEVEL_MIN
#define HW_WATER_LEVEL_MAX CONFIG_HW_WATER_LEVEL_MAX
#define HW_WATER_LEVEL_LOW CONFIG_HW_WATER_LEVEL_LOW
#define HW_WATER_LEVEL_HIGH CONFIG_HW_WATER_LEVEL_HIGH
#define HW_WATER_SENSOR_DELAY_MS CONFIG_HW_WATER_LEVEL_DELAY_MS

#define IRRIGATION_ENABLE CONFIG_HW_VALVE_ENABLE
#define IRRIGATION_CRON_EXPRESSION CONFIG_IRRIGATION_CRON_EXPRESSION
#define IRRIGATION_MAX_LENGTH_SECONDS CONFIG_IRRIGATION_MAX_LENGTH_SECONDS
#define IRRIGATION_WATER_LEVEL_LOW_PERCENT CONFIG_IRRIGATION_WATER_LEVEL_LOW_PERCENT
#define IRRIGATION_WATER_LEVEL_HIGH_PERCENT CONFIG_IRRIGATION_WATER_LEVEL_HIGH_PERCENT