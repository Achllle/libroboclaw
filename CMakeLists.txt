cmake_minimum_required(VERSION 3.10)

# set the project name
project(libroboclaw VERSION 0.1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

configure_file(libroboclawConfig.h.in libroboclawConfig.h)

############################################################
# Create a library
############################################################

# add library
add_library(libroboclaw
    src/roboclaw_driver.cpp
    src/TimeoutSerial.cpp
)

target_link_libraries(libroboclaw gtest pthread)

target_include_directories(libroboclaw
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

############################################################
# Create an executable
############################################################

# add the executable
add_executable(connect 
    src/connect.cpp
)

# add binary tree for finding libroboclawConfig
target_include_directories(connect PUBLIC
                           "${PROJECT_BINARY_DIR}"
)

target_link_libraries(connect
    PRIVATE
        libroboclaw
)