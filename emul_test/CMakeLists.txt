cmake_minimum_required(VERSION 3.20.0)

set(EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../oot_module)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_zephyr_app)

target_sources(app PRIVATE src/main.c)
