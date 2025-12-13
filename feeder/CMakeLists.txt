cmake_minimum_required(VERSION 3.20)
project(vjoy_feeder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -O3")

set(VJOY_DLL_PATH "C:/Program Files/vJoy/x64/vJoyInterface.dll")
set(VJOY_LIB_PATH "${CMAKE_SOURCE_DIR}/lib/vJoyInterface.lib")
set(INC_PATH "${CMAKE_SOURCE_DIR}/inc")
set(SRC_PATH "${CMAKE_SOURCE_DIR}/src")
set(HEADERS "${INC_PATH}/feeder.h" "${INC_PATH}/public.h" "${INC_PATH}/vjoyinterface.h" "${INC_PATH}/ffb.h")
set(SOURCES
    "${SRC_PATH}/feeder.cpp"
    "${SRC_PATH}/main.cpp"
    "${SRC_PATH}/serial_reader.cpp"
    "${SRC_PATH}/ffb.cpp"
)

add_executable(vjoy_feeder ${SOURCES})

target_include_directories(vjoy_feeder PRIVATE "${INC_PATH}")
target_link_libraries(vjoy_feeder PRIVATE "${VJOY_LIB_PATH}")

add_custom_command(TARGET vjoy_feeder POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${VJOY_DLL_PATH}"
        "$<TARGET_FILE_DIR:vjoy_feeder>"
)