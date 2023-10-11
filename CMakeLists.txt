# Copyright 2023 Klepsydra Technologies AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Main header: Project name & Configuration
# ------------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.12)

set(PROJ_MAIN_NAME "tutorial")
project("kpsr-tutorial")

set(KLEPSYDRA_MAJOR_VERSION 2)
set(KLEPSYDRA_MINOR_VERSION 0)
set(KLEPSYDRA_PATCH_VERSION 0)
set(KLEPSYDRA_VERSION
    ${KLEPSYDRA_MAJOR_VERSION}.${KLEPSYDRA_MINOR_VERSION}.${KLEPSYDRA_PATCH_VERSION}
)

# Configuration types
set(CMAKE_CONFIGURATION_TYPES
    "Debug;Release"
    CACHE STRING "Configs" FORCE)
if(DEFINED CMAKE_BUILD_TYPE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
                                                 ${CMAKE_CONFIGURATION_TYPES})
endif()

if(DEFINED THIRDPARTIES_PATH)

else()
    set(THIRDPARTIES_PATH ${CMAKE_SOURCE_DIR}/thirdparties)
endif()
message(STATUS "Thirdparties install path: " ${THIRDPARTIES_PATH})

if(DEFINED GTEST_PATH)

else()
    set(GTEST_PATH ${THIRDPARTIES_PATH}/googletest) # default google test path
endif()
message(STATUS "Google test path: " ${GTEST_PATH})
if(DEFINED KPSR_BUILD_PATH)

else()
    set(KPSR_BUILD_PATH ${CMAKE_SOURCE_DIR}/kpsr-build)
endif()
message(STATUS "kpsr-build path: " ${KPSR_BUILD_PATH})

# Configure variables
# ------------------------------------------------------------------------------------------------------

# Paths
set(${PROJ_MAIN_NAME}_PATH_MAIN
    ${CMAKE_SOURCE_DIR}
    CACHE PATH "This directory contains initial Path")
set(CMAKE_MODULE_PATH ${KPSR_BUILD_PATH}/cmake)
set(LIBRARY_OUTPUT_PATH
    ${PROJECT_BINARY_DIR}/bin
    CACHE PATH "Library output" FORCE)
set(EXECUTABLE_OUTPUT_PATH
    ${PROJECT_BINARY_DIR}/bin
    CACHE PATH "Executable output" FORCE)

# Must use GNUInstallDirs to install libraries into correct locations on all
# platforms.
include(GNUInstallDirs)

# Build type
set(BUILD_SHARED_LIBS "ON" FORCE)

find_package(Klepsydra 11 REQUIRED)

include(KpsrCodeGen)
kpsreventgenerator(
    "${CMAKE_CURRENT_SOURCE_DIR}/kidl" "${CMAKE_CURRENT_SOURCE_DIR}/gen"
    "klepsydra/tutorial" "False" "True" "True")

# Must use GNUInstallDirs to install libraries into correct locations on all
# platforms.
include(GNUInstallDirs)
include(CheckCXXCompilerFlag)
include(CheckIncludeFileCXX)

check_cxx_compiler_flag(-std=gnu++11 HAS_CXX11)
if(HAS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

if(NOT APPLE)
    check_cxx_compiler_flag(-Wl,--no-undefined HAS_NO_UNDEFINED)
    if(HAS_NO_UNDEFINED)
        set(CMAKE_EXE_LINKER_FLAGS
            "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-undefined")
        set(CMAKE_SHARED_LINKER_FLAGS
            "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
        set(CMAKE_MODULE_LINKER_FLAGS
            "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
    endif()
endif()

check_cxx_compiler_flag(-pedantic HAS_PEDANTIC)
if(HAS_PEDANTIC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
endif()

check_cxx_compiler_flag(-Wall HAS_WALL)
if(HAS_WALL)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

check_include_file_cxx(sys/epoll.h HAS_EPOLL)
if(HAS_EPOLL)
    add_definitions(-DHAS_EPOLL)
else()
    add_definitions(-DUSE_GET_WRITE_QUEUE)
endif()

add_definitions(-std=c++11)
enable_testing()

if(KPSR_WITH_CODE_METRICS)
    if(CMAKE_BUILD_TYPE MATCHES Debug AND CMAKE_COMPILER_IS_GNUCXX)
        include(CodeCoverage)
        set(CODE_COVERAGE_VERBOSE ON)
        append_coverage_compiler_flags()
    endif()

    # Code style check
    include(CppcheckTargets)
    include(Findcppcheck)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Configuration current project
# ------------------------------------------------------------------------------------------------------

# Dependencies
find_package(Threads REQUIRED)

# Project Logic
# ------------------------------------------------------------------------------------------------------

# ##############################################################################
# ADD SUB FOLDERS
# ##############################################################################
add_subdirectory(modules)
add_subdirectory(tests)
add_subdirectory(examples)

# Code Coverage
if(KPSR_WITH_CODE_METRICS
   AND CMAKE_BUILD_TYPE MATCHES Debug
   AND CMAKE_COMPILER_IS_GNUCXX)
    string(REPLACE ${CMAKE_SOURCE_DIR}/ "" THIRDPARTIES_PATH_COVERAGE
                   ${THIRDPARTIES_PATH})
    list(
        APPEND
        COVERAGE_EXCLUDES
        ${THIRDPARTIES_PATH_COVERAGE}
        ${CMAKE_PREFIX_PATH}
        tests
        examples
        /usr/
        /opt)
    setup_target_for_coverage_gcovr_xml(NAME coverage EXECUTABLE make test)
endif()

# Code format check
include(CodeFormat)

# PACKAGING
if(NOT DEFINED KPSR_VERSION_MAJOR)
    set(KPSR_VERSION_MAJOR ${KLEPSYDRA_MAJOR_VERSION})
endif()

if(NOT DEFINED KPSR_VERSION_MINOR)
    set(KPSR_VERSION_MINOR ${KLEPSYDRA_MAJOR_VERSION})
endif()

if(NOT DEFINED KPSR_VERSION_PATCH)
    set(KPSR_VERSION_PATCH ${KLEPSYDRA_PATCH_VERSION})
endif()

set(KPSR_PACKAGE_DESCRIPTION "Klepsydra SDK Tutorial")
set(KPSR_PACKAGE_DESCRIPTION_SUMMARY "Klepsydra SDK Tutorial")

include(KlepsydraPackaging)
