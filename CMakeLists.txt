cmake_minimum_required(VERSION 3.13)
project(example)

find_package(opencm3)

add_executable(example
    src/main.cpp
    src/usb.cpp
)
target_include_directories(example PRIVATE include)
target_link_libraries(example opencm3)
