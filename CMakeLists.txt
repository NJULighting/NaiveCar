cmake_minimum_required(VERSION 3.8)
project(NaiveCar)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES GPIOlib.h main2.cpp GPIOlib.cpp)
add_executable(NaiveCar ${SOURCE_FILES})


target_link_libraries(NaiveCar ${OpenCV_LIBS} )