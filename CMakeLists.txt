cmake_minimum_required(VERSION 3.13)
project(example)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Warning mistakenly introduced in c++20, will be reverted in c++23
add_compile_options("-Wno-volatile")
add_compile_options("-DEMBEDDED")

find_package(opencm3 REQUIRED)
find_package(nanoprintf REQUIRED)
find_package(micro-types REQUIRED)
find_package(datapack REQUIRED)

add_executable(example
    src/main.cpp
    src/usb.cpp
)
target_include_directories(example PRIVATE include)
target_link_libraries(example opencm3 nanoprintf micro-types datapack)
