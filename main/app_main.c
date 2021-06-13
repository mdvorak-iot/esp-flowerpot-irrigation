#include "app_status.h"
#include <app_wifi.h>
#include <double_reset.h>
#include <driver/adc_common.h>
#include <ds18b20.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <owb.h>
#include <owb_rmt.h>
#include <util/util_append.h>
#include <wifi_reconnect.h>

#define APP_DEVICE_NAME CONFIG_APP_DEVICE_NAME
#define APP_CONTROL_LOOP_INTERVAL CONFIG_APP_CONTROL_LOOP_INTERVAL

#define HW_VALVE_ENABLE_PIN CONFIG_HW_VALVE_ENABLE_PIN
#define HW_DS18B20_PIN CONFIG_HW_DS18B20_PIN
#define HW_SENSOR_ENABLE_PIN CONFIG_HW_SENSOR_ENABLE_PIN
#define HW_SOIL_SENSOR_ADC1_CHANNEL CONFIG_HW_SOIL_SENSOR_ADC1_CHANNEL
#define HW_WATER_LEVEL_SENSOR_COUNT CONFIG_HW_WATER_LEVEL_SENSOR_COUNT
#define HW_WATER_LEVEL_VALUE_THRESHOLD CONFIG_HW_WATER_LEVEL_VALUE_THRESHOLD

static adc1_channel_t HW_WATER_LEVEL_ADC1_CHANNELS[HW_WATER_LEVEL_SENSOR_COUNT] = {
    CONFIG_HW_WATER_LEVEL_ADC1_CHANNEL_1,
#if CONFIG_HW_WATER_LEVEL_SENSOR_COUNT > 1
    CONFIG_HW_WATER_LEVEL_ADC1_CHANNEL_2,
#endif
#if CONFIG_HW_WATER_LEVEL_SENSOR_COUNT > 2
    CONFIG_HW_WATER_LEVEL_ADC1_CHANNEL_3,
#endif
#if CONFIG_HW_WATER_LEVEL_SENSOR_COUNT > 3
    CONFIG_HW_WATER_LEVEL_ADC1_CHANNEL_4,
#endif
#if CONFIG_HW_WATER_LEVEL_SENSOR_COUNT > 4
    CONFIG_HW_WATER_LEVEL_ADC1_CHANNEL_5,
#endif
};

static const char TAG[] = "app_main";

// State
static httpd_handle_t httpd = NULL;
static owb_rmt_driver_info owb_driver = {};
static DS18B20_Info temperature_sensor = {};
static float temperature_value = 0;
static int soil_humidity_value = 0;
static int water_level_values[HW_WATER_LEVEL_SENSOR_COUNT] = {};
static float water_level = 0.0f;

// Program
static esp_err_t metrics_http_handler(httpd_req_t *r);

static void print_qrcode_handler(__unused void *arg, __unused esp_event_base_t event_base,
                                 __unused int32_t event_id, __unused void *event_data)
{
    const char VER[] = "v1";
    char payload[200] = {};
    // {"ver":"%s","name":"%s","pop":"%s","transport":"%s"}
    snprintf(payload, sizeof(payload), "%%7B%%22ver%%22%%3A%%22%s%%22%%2C%%22name%%22%%3A%%22%s%%22%%2C%%22pop%%22%%3A%%22%s%%22%%2C%%22transport%%22%%3A%%22%s%%22%%7D",
             VER, app_wifi_prov_get_service_name(), app_wifi_get_prov_pop(), APP_WIFI_PROV_TRANSPORT);
    // NOTE print this regardless of log level settings
    printf("PROVISIONING: To view QR Code, copy paste the URL in a browser:\n%s?data=%s\n", "https://espressif.github.io/esp-jumpstart/qrcode.html", payload);
}

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
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_PROV_EVENT, WIFI_PROV_START, print_qrcode_handler, NULL, NULL));

    // Valve
    ESP_ERROR_CHECK(gpio_reset_pin(HW_VALVE_ENABLE_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(HW_VALVE_ENABLE_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(HW_SENSOR_ENABLE_PIN, 0));

    // Sensor power config
    ESP_ERROR_CHECK(gpio_reset_pin(HW_SENSOR_ENABLE_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(HW_SENSOR_ENABLE_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(HW_SENSOR_ENABLE_PIN, 0));

    // Soil humidity and water level sensor init
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(HW_SOIL_SENSOR_ADC1_CHANNEL, ADC_ATTEN_DB_11)); // TODO attenuation?

    for (size_t i = 0; i < HW_WATER_LEVEL_SENSOR_COUNT; i++)
    {
        assert(HW_WATER_LEVEL_ADC1_CHANNELS[i] >= 0);
        assert(HW_WATER_LEVEL_ADC1_CHANNELS[i] < ADC1_CHANNEL_MAX);
        ESP_ERROR_CHECK(adc1_config_channel_atten(HW_WATER_LEVEL_ADC1_CHANNELS[i], ADC_ATTEN_DB_11));
    }

    // Temperature sensor init
    owb_rmt_initialize(&owb_driver, HW_DS18B20_PIN, RMT_CHANNEL_0, RMT_CHANNEL_1);
    owb_use_crc(&owb_driver.bus, true);
    ds18b20_init_solo(&temperature_sensor, &owb_driver.bus); // Only single sensor is expected
    ds18b20_use_crc(&temperature_sensor, true);
    ds18b20_set_resolution(&temperature_sensor, DS18B20_RESOLUTION_12_BIT);

    // HTTP Server
    httpd_config_t httpd_config = HTTPD_DEFAULT_CONFIG();
    ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_start(&httpd, &httpd_config));
    httpd_uri_t metrics_handler_uri = {.uri = "/metrics", .method = HTTP_GET, .handler = metrics_http_handler};
    ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_register_uri_handler(httpd, &metrics_handler_uri));

    // Start
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
        // Enable sensors
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(HW_SENSOR_ENABLE_PIN, 1));

        // Read temperature
        ds18b20_convert_all(&owb_driver.bus);
        ds18b20_wait_for_conversion(&temperature_sensor);

        DS18B20_ERROR ds_err = ds18b20_read_temp(&temperature_sensor, &temperature_value);
        if (ds_err != DS18B20_OK)
        {
            ESP_LOGW(TAG, "failed to read temperature: %d", ds_err);
        }

        // Wait for sensors to stabilize (adds extra 100ms to the loop)
        vTaskDelayUntil(&start, 100 / portTICK_PERIOD_MS);

        // Read soil humidity
        soil_humidity_value = adc1_get_raw(HW_SOIL_SENSOR_ADC1_CHANNEL);

        // Read water levels
        float water_level_readout = 0.0f;
        for (size_t i = 0; i < HW_WATER_LEVEL_SENSOR_COUNT; i++)
        {
            int x = water_level_values[i] = adc1_get_raw(HW_WATER_LEVEL_ADC1_CHANNELS[i]);
            if (x > HW_WATER_LEVEL_VALUE_THRESHOLD)
            {
                water_level_readout = (float)(i + 1) / (float)HW_WATER_LEVEL_SENSOR_COUNT;
            }
        }
        water_level = water_level_readout;

        // Disable sensors
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(HW_SENSOR_ENABLE_PIN, 0));

        // Log output
        ESP_LOGI(TAG, "water: %.1f, soil: %d,\ttemperature: %.3f", water_level, soil_humidity_value, temperature_value);

        // Throttle
        vTaskDelayUntil(&start, APP_CONTROL_LOOP_INTERVAL / portTICK_PERIOD_MS);
    }
}

static esp_err_t metrics_http_handler(httpd_req_t *r)
{
    const char name[] = APP_DEVICE_NAME; // TODO dynamic from device config

    // Build metrics string
    char buf[1024] = {};
    char *ptr = buf;
    const char *end = ptr + sizeof(buf);

    // Temperature
    char temperature_address[17] = {};
    snprintf(temperature_address, sizeof(temperature_address), "%llx", *(uint64_t *)temperature_sensor.rom_code.bytes);

    ptr = util_append(ptr, end, "# TYPE esp_celsius gauge\n");
    ptr = util_append(ptr, end, "esp_celsius{address=\"%s\",hardware=\"%s\",sensor=\"Ambient\"} %0.3f\n", temperature_address, name, temperature_value);

    // Soil
    ptr = util_append(ptr, end, "# TYPE esp_humidity_raw gauge\n");
    ptr = util_append(ptr, end, "esp_humidity_raw{hardware=\"%s\",sensor=\"Soil\"} %d\n", name, soil_humidity_value);
    // TODO provide normalized value

    // Water level
    ptr = util_append(ptr, end, "# TYPE esp_water_level_raw gauge\n");
    for (size_t i = 0; i < HW_WATER_LEVEL_SENSOR_COUNT; i++)
    {
        ptr = util_append(ptr, end, "esp_water_level_raw{hardware=\"%s\",sensor=\"Pot %zu\"} %d\n", name, i, water_level_values[i]);
    }

    ptr = util_append(ptr, end, "# TYPE esp_water_level gauge\n");
    ptr = util_append(ptr, end, "esp_water_level{hardware=\"%s\",sensor=\"Pot\"} %.2f\n", name, water_level);

    // Send result
    if (ptr != NULL)
    {
        // Send data
        httpd_resp_set_type(r, "text/plain");
        return httpd_resp_send(r, buf, (ssize_t)(ptr - buf));
    }
    else
    {
        // Buffer overflow
        ESP_LOGE(TAG, "metrics buffer overflow");
        return ESP_FAIL;
    }
}
