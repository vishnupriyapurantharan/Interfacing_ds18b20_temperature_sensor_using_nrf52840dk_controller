# Private CMakeLists for Interfacing DS18B20 KY-001 Temperature sensor using nrf52840dk and reading temperature value in serial Terminal

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nRF_DS18B20)

target_sources(app PRIVATE main.cpp)

target_include_directories(app PUBLIC "./")

include_directories(D:/VISHNUPRIYA_WORKSPACE-1/DeviceDownload/nRF5_SDK_17.1.0_ddde560/modules/nrfx/drivers/src)

