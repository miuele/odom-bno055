# Copyright (c) 2022 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)


set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(MBED_OS_TARGET mbed-core)
set(APP_TARGET odom-bno055)

include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_executable(${APP_TARGET}
    main.cpp
)
set_property(TARGET ${APP_TARGET} PROPERTY CXX_STANDARD 14)

add_subdirectory(${MBED_PATH} ${CMAKE_CURRENT_BINARY_DIR}/mbed-os-binary)

add_subdirectory(mext_encoder_stm32)

target_link_libraries(${APP_TARGET} ${MBED_OS_TARGET})
target_link_libraries(${APP_TARGET} mext_encoder_stm32)

set(CMAKE_CXX_STANDARD 14)
mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

