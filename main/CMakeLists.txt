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
        esp_http_server
        double_reset
        status_led
        wifi_reconnect
        app_wifi
        esp32-owb
        esp32-ds18b20
)

target_link_libraries(${COMPONENT_LIB} PRIVATE ccronexpr)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual) # -Wconversion
