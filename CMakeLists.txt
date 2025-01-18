cmake_minimum_required(VERSION 3.16)

set(ncnn_DIR "${CMAKE_SOURCE_DIR}/ncnn/lib/cmake/ncnn")

find_package(ncnn REQUIRED)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_yolo)


target_link_libraries(${CMAKE_PROJECT_NAME}.elf PRIVATE ncnn)
