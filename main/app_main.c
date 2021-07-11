#include "app_config.h"
#include "app_status.h"
#include <app_wifi.h>
#include <button.h>
#include <double_reset.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_sntp.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <status_led.h>
#include <util/util_append.h>
#include <wifi_reconnect.h>
#if CONFIG_HW_VALVE_ENABLE
#include <ccronexpr.h>
#endif
#if CONFIG_HW_DS18B20_ENABLE
#include <ds18b20.h>
#include <owb.h>
#include <owb_rmt.h>
#endif
#if CONFIG_HW_SOIL_PROBE_ENABLE
#include <driver/touch_sensor.h>
#endif
#if CONFIG_HW_WATER_LEVEL_ENABLE
#include <driver/adc_common.h>
#endif

static const char DRAM_ATTR TAG[] = "app_main";

// State
static volatile bool running = true;
static httpd_handle_t httpd = NULL;
#if HW_DS18B20_ENABLE
static owb_rmt_driver_info owb_driver = {};
static DS18B20_Info temperature_sensor = {};
static float temperature_value = 0;
#endif
#if IRRIGATION_ENABLE
static cron_expr irrigation_cron = {};
#endif
#if HW_SOIL_PROBE_ENABLE
static bool soil_humidity_valid = false;
static uint16_t soil_humidity_raw = 0;
static float soil_humidity = 0.0f;
#endif
#if HW_WATER_LEVEL_ENABLE
static gpio_num_t hw_water_level_sensor_pin = GPIO_NUM_NC;
static bool water_level_valid = false;
static int water_level_raw = 0;
static float water_level = 0.0f;
#endif
#if HW_VALVE_ENABLE
static volatile bool valve_on = false;
#endif

// Program
static esp_err_t metrics_http_handler(httpd_req_t *r);
static void button_handler(void *arg, const struct button_data *data);

#if HW_SOIL_PROBE_ENABLE || HW_WATER_LEVEL_ENABLE
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
#endif

#if IRRIGATION_ENABLE
static bool can_irrigate(time_t t)
{
    // Find previous start
    time_t prev_start = cron_prev(&irrigation_cron, t);

    // If it is less then interval, let the water flow!
    return (t - prev_start) < IRRIGATION_MAX_LENGTH_SECONDS;
}

static void log_next_irrigation(time_t now)
{
    time_t next_start = cron_next(&irrigation_cron, now);
    struct tm next_start_tm = {};
    localtime_r(&next_start, &next_start_tm);

    char s[51] = {};
    strftime(s, 50, "%x %X %z", &next_start_tm);

    ESP_LOGI(TAG, "next irrigation starts on %s for %d seconds", s, IRRIGATION_MAX_LENGTH_SECONDS);
}
#endif

#if HW_WATER_LEVEL_ENABLE
static bool is_in_range(int val, int min, int max)
{
    return (val >= min) && (val <= max);
}
#endif

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

    // Parse irrigation CRON
#if IRRIGATION_ENABLE
    const char *cron_err = NULL;
    cron_parse_expr(IRRIGATION_CRON_EXPRESSION, &irrigation_cron, &cron_err);
    if (cron_err)
    {
        ESP_LOGE(TAG, "failed to parse cron '" IRRIGATION_CRON_EXPRESSION "'");
    }
#endif

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

    // Button
    struct button_config switch_cfg = {
        .level = HW_BUTTON_LEVEL,
        .long_press_ms = HW_BUTTON_LONG_PRESS_MS,
        .internal_pull = HW_BUTTON_PULL,
        .on_press = button_handler,
        .on_release = button_handler,
    };
    ESP_ERROR_CHECK(button_config(HW_BUTTON_PIN, &switch_cfg, NULL));

    // Valve
#if HW_VALVE_ENABLE
    ESP_ERROR_CHECK(gpio_reset_pin(HW_VALVE_POWER_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(HW_VALVE_POWER_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(HW_VALVE_POWER_PIN, 0));
#endif

    // Water level sensor (leave pins floating by default)
#if HW_WATER_LEVEL_ENABLE
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_pad_get_io_num(HW_WATER_LEVEL_ADC1_CHANNEL, &hw_water_level_sensor_pin));

    ESP_ERROR_CHECK(gpio_reset_pin(HW_WATER_SENSOR_POWER_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(HW_WATER_SENSOR_POWER_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(hw_water_level_sensor_pin, GPIO_MODE_INPUT));
#endif

    // Soil humidity sensor
#if HW_SOIL_PROBE_ENABLE
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_KEEP, TOUCH_LVOLT_KEEP, TOUCH_HVOLT_ATTEN_0V));
    ESP_ERROR_CHECK(touch_pad_config(HW_SOIL_PROBE_TOUCH_PAD, 0));
#endif

    // Temperature sensor init
#if HW_DS18B20_ENABLE
    owb_rmt_initialize(&owb_driver, HW_DS18B20_PIN, RMT_CHANNEL_0, RMT_CHANNEL_1);
    owb_use_crc(&owb_driver.bus, true);
    ds18b20_init_solo(&temperature_sensor, &owb_driver.bus); // Only single sensor is expected
    ds18b20_use_crc(&temperature_sensor, true);
    ds18b20_set_resolution(&temperature_sensor, DS18B20_RESOLUTION_12_BIT);
#endif

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

void app_main()
{
    setup();

#if IRRIGATION_ENABLE
    // Wait for irrigation timeout for NTP sync - this implies working internet connection
    // NOTE this is needed, so valve is not turned on before wifi connection is made
    while (time(NULL) < IRRIGATION_MAX_LENGTH_SECONDS)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "synced time=%ld", time(NULL));
    log_next_irrigation(time(NULL));
#endif

    // Read values continuously
    TickType_t start = xTaskGetTickCount();

    while (running)
    {
        // Simple status led (don't overwrite connecting states)
        if (!status_led_is_active(STATUS_LED_DEFAULT))
        {
            status_led_set_interval_for(STATUS_LED_DEFAULT, 0, true, 100, false);
        }

        // Read temperature
#if HW_DS18B20_ENABLE
        ds18b20_convert_all(&owb_driver.bus);
        ds18b20_wait_for_conversion(&temperature_sensor);

        DS18B20_ERROR ds_err = ds18b20_read_temp(&temperature_sensor, &temperature_value);
        if (ds_err != DS18B20_OK)
        {
            ESP_LOGW(TAG, "failed to read temperature: %d", ds_err);
        }
#endif

        // Read soil humidity
#if HW_SOIL_PROBE_ENABLE
        uint16_t soil_humidity_readout = 0;
        ESP_ERROR_CHECK_WITHOUT_ABORT(touch_pad_read(HW_SOIL_PROBE_TOUCH_PAD, &soil_humidity_readout));

        if (is_in_range(water_level_readout, HW_SOIL_PROBE_MIN, HW_SOIL_PROBE_MAX))
        {
            soil_humidity_raw = soil_humidity_readout;
            soil_humidity = map_to_range((float)soil_humidity_raw, HW_SOIL_PROBE_LOW, HW_SOIL_PROBE_HIGH, 0.0f, 1.0f);
            soil_humidity_valid = true; // update state before flipping to true
        }
        else
        {
            soil_humidity_valid = false;
        }
#endif

        // Enable water-level sensor
#if HW_WATER_LEVEL_ENABLE
        ESP_ERROR_CHECK_WITHOUT_ABORT(adc1_config_channel_atten(HW_WATER_LEVEL_ADC1_CHANNEL, ADC_ATTEN_DB_11));
        ESP_ERROR_CHECK(gpio_set_direction(HW_WATER_SENSOR_POWER_PIN, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(HW_WATER_SENSOR_POWER_PIN, 1));

        // Wait for sensors to stabilize
        vTaskDelayUntil(&start, HW_WATER_SENSOR_DELAY_MS / portTICK_PERIOD_MS);

        // Read water level
        int water_level_readout = adc1_get_raw(HW_WATER_LEVEL_ADC1_CHANNEL);

        if (is_in_range(water_level_readout, HW_WATER_LEVEL_MIN, HW_WATER_LEVEL_MAX))
        {
            water_level_raw = water_level_readout;
            water_level = map_to_range((float)water_level_raw, HW_WATER_LEVEL_LOW, HW_WATER_LEVEL_HIGH, 0.0f, 1.0f);
            water_level_valid = true; // update state before flipping to true
        }
        else
        {
            ESP_LOGW(TAG, "water level readout invalid: %d", water_level_readout);
            water_level_valid = false;
        }
#endif

        // Trigger irrigation
        // TODO support MIN/MAX
#if IRRIGATION_ENABLE
        // NOTE this should be smarter, now it depends on loop execution
        if (can_irrigate(time(NULL)))
        {
#if HW_WATER_LEVEL_ENABLE
            if (!water_level_valid && valve_on)
            {
                // Turn off the valve
                ESP_LOGW(TAG, "turning off the valve, water level readout is invalid!");
                valve_on = false;
            }
            else if (!valve_on && water_level <= (float)IRRIGATION_WATER_LEVEL_LOW_PERCENT / 100.0f)
            {
                // Turn on the valve
                ESP_LOGW(TAG, "turning on the valve, low water level detected");
                valve_on = true;
            }
            else if (valve_on && water_level >= (float)IRRIGATION_WATER_LEVEL_HIGH_PERCENT / 100.0f)
            {
                // Turn off the valve
                ESP_LOGW(TAG, "turning off the valve, high water level detected");
                valve_on = false;
            }
#else
            ESP_LOGW(TAG, "turning on the valve unconditionally, according to schedule");
            log_next_irrigation(time(NULL));
            valve_on = true;
#endif
        }
        else if (valve_on)
        {
            // Turn off the valve
            ESP_LOGW(TAG, "turning off the valve because of time-out");
            log_next_irrigation(time(NULL));
            valve_on = false;
        }

        // Control valve
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(HW_VALVE_POWER_PIN, valve_on ? 1 : 0));
#endif

        // Log output
        // TODO assemble nic log
        //ESP_LOGI(TAG, "water: %.2f (raw=%d),\tsoil: %.2f (raw=%d),\ttemperature: %.3f", water_level, water_level_raw, soil_humidity, soil_humidity_raw, temperature_value);

        // Disable water sensor
#if HW_WATER_LEVEL_ENABLE
        // TODO revise sequence
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(HW_WATER_SENSOR_POWER_PIN, 0));

        // Discharge capacitor
        vTaskDelay(HW_WATER_SENSOR_DELAY_MS / portTICK_PERIOD_MS);

        // Switch polarity for a while
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(hw_water_level_sensor_pin, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(hw_water_level_sensor_pin, 1));

        vTaskDelay(HW_WATER_SENSOR_DELAY_MS / portTICK_PERIOD_MS);

        // Leave it floating for better soil humidity precision
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(HW_WATER_SENSOR_POWER_PIN, GPIO_MODE_INPUT));
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(hw_water_level_sensor_pin, GPIO_MODE_INPUT));
#endif

        // Throttle - must be last
        vTaskDelayUntil(&start, APP_CONTROL_LOOP_INTERVAL / portTICK_PERIOD_MS);
    }
}

static void factory_reset(__unused void *arg)
{
    ESP_LOGW(TAG, "factory reset");

    // Stop main loop
    running = false;

    // Shut down water
#if IRRIGATION_ENABLE
    valve_on = false;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(HW_VALVE_POWER_PIN, 0));
#endif

    // Erase flash
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());

    // Restart
    esp_restart();
}

static void provision_wifi(__unused void *arg)
{
    // Stop main loop
    running = false;

    // Shut down water
#if IRRIGATION_ENABLE
    valve_on = false;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(HW_VALVE_POWER_PIN, 0));
#endif

    // Set provision flag
    ESP_ERROR_CHECK_WITHOUT_ABORT(double_reset_set(true));

    // Restart
    esp_restart();
}

static void button_handler(__unused void *arg, const struct button_data *data)
{
    if (data->event == BUTTON_EVENT_RELEASED)
    {
        // Released
        if (data->press_length_ms > 10000) // TODO constant
        {
            // Factory reset
            // NOTE don't do this from ISR
            xTaskCreate(factory_reset, DRAM_STR("factory"), 1000, NULL, 1, NULL);
        }
    }
    else if (data->long_press)
    {
        // Long-press
        // NOTE don't do this from ISR
        xTaskCreate(provision_wifi, DRAM_STR("wifi_prov"), 1000, NULL, 1, NULL);
    }
#if IRRIGATION_ENABLE
    else
    {
        // Short press
        ESP_DRAM_LOGW(TAG, "turning on the valve manually");
        valve_on = !valve_on;
        // NOTE will take effect on next loop iteration
    }
#endif
}

static esp_err_t metrics_http_handler(httpd_req_t *r)
{
    const char name[] = APP_DEVICE_NAME; // TODO dynamic from device config

    // Build metrics string
    char buf[1024] = {};
    char *ptr = buf;
    const char *end = ptr + sizeof(buf);

    // Temperature
#if HW_DS18B20_ENABLE
    char temperature_address[17] = {};
    snprintf(temperature_address, sizeof(temperature_address), "%llx", *(uint64_t *)temperature_sensor.rom_code.bytes);

    ptr = util_append(ptr, end, "# TYPE esp_celsius gauge\n");
    ptr = util_append(ptr, end, "esp_celsius{address=\"%s\",hardware=\"%s\",sensor=\"Ambient\"} %0.3f\n", temperature_address, name, temperature_value);
#endif

    // Soil
#if HW_SOIL_PROBE_ENABLE
    if (soil_humidity_valid)
    {
        ptr = util_append(ptr, end, "# TYPE esp_humidity gauge\n");
        ptr = util_append(ptr, end, "esp_humidity{hardware=\"%s\",sensor=\"Soil\"} %.2f\n", name, soil_humidity);

        ptr = util_append(ptr, end, "# TYPE esp_humidity_raw gauge\n");
        ptr = util_append(ptr, end, "esp_humidity_raw{hardware=\"%s\",sensor=\"Soil\"} %d\n", name, soil_humidity_raw);
    }
#endif

    // Water level
#if HW_WATER_LEVEL_ENABLE
    if (water_level_valid)
    {
        ptr = util_append(ptr, end, "# TYPE esp_water_level gauge\n");
        ptr = util_append(ptr, end, "esp_water_level{hardware=\"%s\"} %.2f\n", name, water_level);

        ptr = util_append(ptr, end, "# TYPE esp_water_level_raw gauge\n");
        ptr = util_append(ptr, end, "esp_water_level_raw{hardware=\"%s\"} %d\n", name, water_level_raw);
    }
#endif

    // Valve
#if HW_VALVE_ENABLE
    ptr = util_append(ptr, end, "# TYPE esp_valve gauge\n");
    ptr = util_append(ptr, end, "esp_valve {hardware=\"%s\",sensor=\"Water Valve\"} %d\n", name, valve_on);
#endif

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
