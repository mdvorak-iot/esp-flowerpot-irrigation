cmake_minimum_required(VERSION 3.15.0)

idf_component_register(
        SRCS
        app_main.c
        app_status.c
        util/util_append.c
        INCLUDE_DIRS .
        REQUIRES
        freertos
        nvs_flash
        log
        esp32
        esp_wifi
        app_update
        wifi_provisioning
        esp_rainmaker
        rmaker_common
        esp_http_server
        double_reset
        status_led
        wifi_reconnect
        app_wifi
        rainmaker_init
        esp32-owb
        esp32-ds18b20
)
