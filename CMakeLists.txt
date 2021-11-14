# Minimum cmake version
cmake_minimum_required(VERSION 3.0.2)
# Project Name
project(
    cubicSpline
    LANGUAGES CXX
)
# Set header file
set(HEADER_FILES
    include/trajectory.h
)
# Set source file
set(SRC_FILES
    src/cubicSpline.cpp
)
# Set CMake output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin/lib )
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin/lib )
# Message CMake output directory path
message("CMake runtime output directory path : " ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_PATH})
message("CMake library output directory path : " ${CMAKE_LIBRARY_OUTPUT_DIRECTORY_PATH})
message("CMake archive output directory path : " ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_PATH})
# Include directory
include_directories(
    include
)
# Add library with source files
add_library(${PROJECT_NAME}_lib
    ${SRC_FILES}
)
# Add executable file
add_executable(${PROJECT_NAME}_exe
    ${SRC_FILES}
)
# Link target with libraries
target_link_libraries(${PROJECT_NAME}_exe
    ${PROJECT_NAME}_lib
)
# install Targets with executable file
install(TARGETS ${PROJECT_NAME}_exe DESTINATION build/bin)