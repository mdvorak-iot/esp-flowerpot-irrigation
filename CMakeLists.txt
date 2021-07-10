cmake_minimum_required(VERSION 3.15.0)

add_compile_definitions(GPIO_NUM_NC=GPIO_NUM_NC) # owb have broken define
list(APPEND EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/rainmaker/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp-flowerpot-irrigation)

set(CRON_USE_LOCAL_TIME ON)
add_subdirectory(libs/ccronexpr)
