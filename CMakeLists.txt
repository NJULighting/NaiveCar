cmake_minimum_required(VERSION 3.8)
project(NaiveCar)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp GPIOlib.h main2.cpp)
add_executable(NaiveCar ${SOURCE_FILES})


target_link_libraries(NaiveCar ${OpenCV_LIBS} )
target_link_libraries(NaiveCar ${PROJECT_SOURCE_DIR}/lib/libGPIO.a)