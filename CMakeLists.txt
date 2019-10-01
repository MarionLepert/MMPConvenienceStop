cmake_minimum_required(VERSION 3.10)

project(Button_Master VERSION 1.0)

# Set CMake flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

set(BUTTON_MASTER_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(BUTTON_MASTER_EXT_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(BUTTON_MASTER_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)

# Find dependencies
function(download_package PACKAGE_NAME)
    configure_file(${BUTTON_MASTER_CMAKE_DIR}/${PACKAGE_NAME}-CMakeLists.txt.in
        ${CMAKE_BINARY_DIR}/${PACKAGE_NAME}/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
endfunction()

# Ctrl Utils (Redis)
# Use the NO_CMAKE_PACKAGE_REGISTRY flag to prevent the compiler from finding ctrl-utils packages in other repositories 
find_package(ctrl_utils 1.0.0 QUIET NO_CMAKE_PACKAGE_REGISTRY HINTS ${PROJECT_SOURCE_DIR}/build/ctrl_utils/build)
if(NOT ctrl_utils_FOUND)
    download_package(ctrl_utils)
    find_package(ctrl_utils 1.0.0 REQUIRED NO_CMAKE_PACKAGE_REGISTRY HINTS ${PROJECT_SOURCE_DIR}/build/ctrl_utils/build)
endif()

find_package(iir)


# set common dependencies
# - eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
    find_package(Eigen3 QUIET)
    # TODO: Find automatic way to find for Ubuntu 14.04
    if(NOT EIGEN3_INCLUDE_DIR)
        set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
    endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})


include_directories( ${PROJECT_SOURCE_DIR}/include
           ${PROJECT_SOURCE_DIR}/lib)

link_libraries("-lrt")
link_libraries(ctrl_utils::ctrl_utils)

add_subdirectory(src)


