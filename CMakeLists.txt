cmake_minimum_required(VERSION 3.10)
project(sdr_test C)

set(CMAKE_C_STANDARD 11)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SOAPYSDR REQUIRED SoapySDR)

add_executable(sdr_test ph/main.c)

target_include_directories(sdr_test PRIVATE ${SOAPYSDR_INCLUDE_DIRS})
target_link_libraries(sdr_test PRIVATE ${SOAPYSDR_LIBRARIES} m)
target_compile_options(sdr_test PRIVATE ${SOAPYSDR_CFLAGS_OTHER})