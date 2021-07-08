#include "app_status.h"
#include <app_wifi.h>
#include <double_reset.h>
#include <driver/adc_common.h>
#include <driver/touch_sensor.h>
#include <ds18b20.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_sntp.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <owb.h>
#include <owb_rmt.h>
#include <status_led.h>
#include <util/util_append.h>
#include <wifi_reconnect.h>

#define APP_DEVICE_NAME CONFIG_APP_DEVICE_NAME
#define APP_CONTROL_LOOP_INTERVAL CONFIG_APP_CONTROL_LOOP_INTERVAL

#define HW_VALVE_ENABLE_PIN CONFIG_HW_VALVE_ENABLE_PIN
#define HW_DS18B20_PIN CONFIG_HW_DS18B20_PIN
#define HW_SOIL_SENSOR_TOUCH_PAD CONFIG_HW_SOIL_SENSOR_TOUCH_PAD
#define HW_WATER_SENSOR_ENABLE_PIN CONFIG_HW_WATER_SENSOR_ENABLE_PIN
#define HW_WATER_LEVEL_ADC1_CHANNEL CONFIG_HW_WATER_LEVEL_ADC1_CHANNEL
#define HW_WATER_LEVEL_VALUE_LOW CONFIG_HW_WATER_LEVEL_VALUE_LOW
#define HW_WATER_LEVEL_VALUE_HIGH CONFIG_HW_WATER_LEVEL_VALUE_HIGH

#define IRRIGATION_EVERY_N_HOURS CONFIG_IRRIGATION_EVERY_N_HOURS
#define IRRIGATION_MAX_LENGTH_SECONDS CONFIG_IRRIGATION_MAX_LENGTH_SECONDS
#define IRRIGATION_WATER_LEVEL_LOW_PERCENT CONFIG_IRRIGATION_WATER_LEVEL_LOW_PERCENT
#define IRRIGATION_WATER_LEVEL_HIGH_PERCENT CONFIG_IRRIGATION_WATER_LEVEL_HIGH_PERCENT

static const char TAG[] = "app_main";

// State
static gpio_num_t hw_water_level_sensor_pin = GPIO_NUM_NC;
static httpd_handle_t httpd = NULL;
static owb_rmt_driver_info owb_driver = {};
static DS18B20_Info temperature_sensor = {};
static float temperature_value = 0;
static uint16_t soil_humidity_raw = 0;
static int water_level_raw = 0;
static float soil_humidity = 0.0f;
static float water_level = 0.0f;
static bool valve_on = false;

// Program
static esp_err_t metrics_http_handler(httpd_req_t *r);

static float constrain(float x, float min, float max)
{
    return x < min ? min : (x > max ? max : x);
}

static float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static float map_to_range(float x, float in_min, float in_max, float out_min, float out_max)
{
    return map(constrain(x, in_min, in_max), in_min, in_max, out_min, out_max);
}

static bool can_irrigate(time_t t)
{
    struct tm now = {};
    localtime_r(&t, &now);

    int hours_secs = now.tm_min * 60 + now.tm_sec;
    return (now.tm_hour % IRRIGATION_EVERY_N_HOURS) == 0 && hours_secs >= 0 && hours_secs < IRRIGATION_MAX_LENGTH_SECONDS;
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
    ESP_ERROR_CHECK(app_wifi_print_qr_code_handler_register(NULL));

    // Valve
    ESP_ERROR_CHECK(gpio_reset_pin(HW_VALVE_ENABLE_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(HW_VALVE_ENABLE_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(HW_VALVE_ENABLE_PIN, 0));

    // Water level sensor
    ESP_ERROR_CHECK(gpio_reset_pin(HW_WATER_SENSOR_ENABLE_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(HW_WATER_SENSOR_ENABLE_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(HW_WATER_SENSOR_ENABLE_PIN, 0));

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(HW_WATER_LEVEL_ADC1_CHANNEL, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_pad_get_io_num(HW_WATER_LEVEL_ADC1_CHANNEL, &hw_water_level_sensor_pin));

    // Soil humidity sensor
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_config(HW_SOIL_SENSOR_TOUCH_PAD, 0));

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

    // NTP
    setenv("TZ", CONFIG_APP_NTP_TZ, 1);
    tzset();
    sntp_setservername(0, CONFIG_APP_NTP_SERVER);
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    sntp_init();

    // Start
    ESP_ERROR_CHECK(app_wifi_start(reconfigure));

    // Done
    ESP_LOGI(TAG, "setup complete");
}

_Noreturn void app_main()
{
    setup();

    // Wait for irrigation timeout for NTP sync - this implies working internet connection
    // NOTE this is needed, so valve is not turned on before wifi connection is made
    // TODO don't wait, just don't open valve until time is known
    while (time(NULL) < IRRIGATION_MAX_LENGTH_SECONDS)
    {
        ESP_LOGI(TAG, "time=%ld", time(NULL));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "time=%ld", time(NULL));

    // Read values continuously
    TickType_t start = xTaskGetTickCount();

    for (;;)
    {
        // Simple status led (don't overwrite connecting states)
        if (!status_led_is_active(STATUS_LED_DEFAULT))
        {
            status_led_set_interval_for(STATUS_LED_DEFAULT, 0, true, 100, false);
        }

        // Read temperature
        ds18b20_convert_all(&owb_driver.bus);
        ds18b20_wait_for_conversion(&temperature_sensor);

        DS18B20_ERROR ds_err = ds18b20_read_temp(&temperature_sensor, &temperature_value);
        if (ds_err != DS18B20_OK)
        {
            ESP_LOGW(TAG, "failed to read temperature: %d", ds_err);
        }

        // Enable sensors
        ESP_ERROR_CHECK_WITHOUT_ABORT(adc_gpio_init(ADC_UNIT_1, HW_WATER_LEVEL_ADC1_CHANNEL));
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(HW_WATER_SENSOR_ENABLE_PIN, 1));

        // Wait for sensors to stabilize
        // TODO Kconfig constant
        vTaskDelayUntil(&start, 200 / portTICK_PERIOD_MS);

        // Read soil humidity
        ESP_ERROR_CHECK_WITHOUT_ABORT(touch_pad_read(HW_SOIL_SENSOR_TOUCH_PAD, &soil_humidity_raw));
        soil_humidity = map_to_range((float)soil_humidity_raw, 50, 700, 1.0f, 0.0f); // TODO range config

        // Read water level
        water_level_raw = adc1_get_raw(HW_WATER_LEVEL_ADC1_CHANNEL);
        water_level = map_to_range((float)water_level_raw, HW_WATER_LEVEL_VALUE_LOW, HW_WATER_LEVEL_VALUE_HIGH, 0.0f, 1.0f);

        // Log output
        ESP_LOGI(TAG, "water: %.2f (raw=%d),\tsoil: %.2f (raw=%d),\ttemperature: %.3f", water_level, water_level_raw, soil_humidity, soil_humidity_raw, temperature_value);

        // Trigger irrigation
        // NOTE this should be smarter, now it depends on loop execution
        if (can_irrigate(time(NULL)))
        {
            if (!valve_on && water_level <= (float)IRRIGATION_WATER_LEVEL_LOW_PERCENT / 100.0f)
            {
                // Turn on the valve
                ESP_LOGW(TAG, "turning on the valve, low water level detected");
                valve_on = true;
            }
            else if (valve_on && water_level >= (float)IRRIGATION_WATER_LEVEL_HIGH_PERCENT / 100.0f)
            {
                // Turn off the valve
                ESP_LOGW(TAG, "turning on the valve, high water level detected");
                valve_on = false;
            }
        }
        else if (valve_on)
        {
            // Turn off the valve
            ESP_LOGW(TAG, "turning off the valve because of time-out");
            valve_on = false;
        }

        // Control valve
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(HW_VALVE_ENABLE_PIN, valve_on ? 1 : 0));

        // Disable water sensor
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(HW_WATER_SENSOR_ENABLE_PIN, 0));

        // Discharge capacitor
        // TODO Kconfig constant
        vTaskDelay(200 / portTICK_PERIOD_MS);

        // Switch polarity for a while
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(hw_water_level_sensor_pin, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(hw_water_level_sensor_pin, 1));

        // TODO Kconfig constant
        vTaskDelay(200 / portTICK_PERIOD_MS);

        // Restore - pin will be reset on next loop
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(hw_water_level_sensor_pin, 0));

        // Throttle - must be last
        vTaskDelayUntil(&start, APP_CONTROL_LOOP_INTERVAL / portTICK_PERIOD_MS);
    }
}

static esp_err_t metrics_http_handler(httpd_req_t *r)
{
    const char name[] = APP_DEVICE_NAME; // TODO dynamic from device config

    // TODO return 500 error until first read

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
    ptr = util_append(ptr, end, "# TYPE esp_humidity gauge\n");
    ptr = util_append(ptr, end, "esp_humidity{hardware=\"%s\",sensor=\"Soil\"} %.2f\n", name, soil_humidity);

    ptr = util_append(ptr, end, "# TYPE esp_humidity_raw gauge\n");
    ptr = util_append(ptr, end, "esp_humidity_raw{hardware=\"%s\",sensor=\"Soil\"} %d\n", name, soil_humidity_raw);

    // Water level
    ptr = util_append(ptr, end, "# TYPE esp_water_level gauge\n");
    ptr = util_append(ptr, end, "esp_water_level{hardware=\"%s\"} %.2f\n", name, water_level);

    ptr = util_append(ptr, end, "# TYPE esp_water_level_raw gauge\n");
    ptr = util_append(ptr, end, "esp_water_level_raw{hardware=\"%s\"} %d\n", name, water_level_raw);

    // Valve
    ptr = util_append(ptr, end, "# TYPE esp_valve gauge\n");
    ptr = util_append(ptr, end, "esp_valve {hardware=\"%s\",sensor=\"Water Valve\"} %d\n", name, valve_on);

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
