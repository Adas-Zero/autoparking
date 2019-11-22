cmake_minimum_required(VERSION 3.10)

project(Tutorial)
find_package(OpenCV REQUIRED)

add_executable(Tutorial tt.cpp)
target_link_libraries(Tutorial ${OpenCV_LIBS})
