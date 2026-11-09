cmake_minimum_required(VERSION 3.16)
project(PlutoSDR CXX)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ищем библиотеку SoapySDR (она уже установлена в систему)
find_package(SoapySDR REQUIRED)

set(MAIN_SOURCE_FILES
    ph/main.cpp
)

add_executable(main ${MAIN_SOURCE_FILES})

target_link_libraries(main ${SoapySDR_LIBRARIES})