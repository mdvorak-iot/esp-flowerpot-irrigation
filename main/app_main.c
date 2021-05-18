#include "app_status.h"
#include <app_rainmaker.h>
#include <app_wifi.h>
#include <double_reset.h>
#include <driver/touch_sensor.h>
#include <ds18b20.h>
#include <esp_log.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <owb.h>
#include <owb_rmt.h>
#include <wifi_reconnect.h>

#define APP_DEVICE_NAME CONFIG_APP_DEVICE_NAME
#define APP_DEVICE_TYPE CONFIG_APP_DEVICE_TYPE
#define APP_CONTROL_LOOP_INTERVAL CONFIG_APP_CONTROL_LOOP_INTERVAL
#define HW_DS18B20_PIN CONFIG_HW_DS18B20_PIN
#define HW_SOIL_SENSOR_TOUCH_PAD CONFIG_HW_SOIL_SENSOR_TOUCH_PAD
#define SENSORS_RMT_CHANNEL_TX RMT_CHANNEL_0
#define SENSORS_RMT_CHANNEL_RX RMT_CHANNEL_1

static const char TAG[] = "app_main";

// State
static owb_rmt_driver_info owb_driver = {};
static DS18B20_Info temperature_sensor = {};
static float temperature_value = 0;

// Program
static void app_devices_init(esp_rmaker_node_t *node);

void setup()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // System services
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Check double reset
    // NOTE this should be called as soon as possible, ideally right after nvs init
    bool reconfigure = false;
    ESP_ERROR_CHECK_WITHOUT_ABORT(double_reset_start(&reconfigure, DOUBLE_RESET_DEFAULT_TIMEOUT));

    // Setup
    app_status_init();

    struct app_wifi_config wifi_cfg = {
        .security = WIFI_PROV_SECURITY_1,
        .wifi_connect = wifi_reconnect_resume,
    };
    ESP_ERROR_CHECK(app_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
    ESP_ERROR_CHECK(wifi_reconnect_start());

    // Soil humidity (Touch) sensor init
    // TODO configurable
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V));
    ESP_ERROR_CHECK(touch_pad_config(HW_SOIL_SENSOR_TOUCH_PAD, 0));
    ESP_ERROR_CHECK(touch_pad_filter_start(10));

    // Temperature sensor init
    owb_rmt_initialize(&owb_driver, HW_DS18B20_PIN, SENSORS_RMT_CHANNEL_TX, SENSORS_RMT_CHANNEL_RX);
    owb_use_crc(&owb_driver.bus, true);
    ds18b20_init_solo(&temperature_sensor, &owb_driver.bus); // Only single sensor is expected
    ds18b20_use_crc(&temperature_sensor, true);
    ds18b20_set_resolution(&temperature_sensor, DS18B20_RESOLUTION_12_BIT);

    // RainMaker
    char node_name[APP_RMAKER_NODE_NAME_LEN] = {};
    ESP_ERROR_CHECK(app_rmaker_node_name(node_name, sizeof(node_name)));

    esp_rmaker_node_t *node = NULL;
    ESP_ERROR_CHECK(app_rmaker_init(node_name, &node));

    app_devices_init(node);

    // Start
    ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, node_name)); // NOTE this isn't available before WiFi init
    ESP_ERROR_CHECK(esp_rmaker_start());
    ESP_ERROR_CHECK(app_wifi_start(reconfigure));

    // Done
    ESP_LOGI(TAG, "setup complete");
}

_Noreturn void app_main()
{
    setup();

    // Read values continuously
    TickType_t start = xTaskGetTickCount();

    for (;;)
    {
        // Read temperature
        ds18b20_convert_all(&owb_driver.bus);
        ds18b20_wait_for_conversion(&temperature_sensor);

        DS18B20_ERROR err = ds18b20_read_temp(&temperature_sensor, &temperature_value);
        if (err != DS18B20_OK)
        {
            ESP_LOGW(TAG, "failed to read temperature: %d", err);
        }

        // Throttle
        vTaskDelayUntil(&start, APP_CONTROL_LOOP_INTERVAL / portTICK_PERIOD_MS);
    }
}

static esp_err_t device_write_cb(__unused const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
                                 const esp_rmaker_param_val_t val, __unused void *private_data,
                                 __unused esp_rmaker_write_ctx_t *ctx)
{
    //    char *param_name = esp_rmaker_param_get_name(param);
    //    if (strcmp(param_name, "TODO") == 0)
    //    {
    //        // TODO handle
    //        esp_rmaker_param_update_and_report(param, val);
    //    }
    return ESP_OK;
}

static void app_devices_init(esp_rmaker_node_t *node)
{
    // Prepare device
    esp_rmaker_device_t *device = esp_rmaker_device_create(APP_DEVICE_NAME, APP_DEVICE_TYPE, NULL);
    assert(device);

    ESP_ERROR_CHECK(esp_rmaker_device_add_cb(device, device_write_cb, NULL));
    ESP_ERROR_CHECK(esp_rmaker_device_add_param(device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, APP_DEVICE_NAME)));
    ESP_ERROR_CHECK(esp_rmaker_node_add_device(node, device));

    // Register buttons, sensors, etc
}
