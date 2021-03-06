#****************************************************************************
#
#                           Klepsydra Core Modules
#              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#****************************************************************************

# Main header: Project name & Configuration
# ------------------------------------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

SET(PROJ_MAIN_NAME "tutorial")
PROJECT(${PROJ_MAIN_NAME})


# Configuration types
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
IF(DEFINED CMAKE_BUILD_TYPE AND CMAKE_VERSION VERSION_GREATER "2.8")
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
ENDIF()

#Install binaries
IF(DEFINED KPSR_INSTALL_PATH)
ELSE()
        SET(KPSR_INSTALL_PATH /opt/klepsydra) #default install path
ENDIF()
message(STATUS "Klepsydra tutorials install path: " ${KPSR_INSTALL_PATH})


IF(DEFINED THIRDPARTIES_PATH)
ELSE()
  SET(THIRDPARTIES_PATH ${CMAKE_SOURCE_DIR}/thirdparties )
ENDIF()
message(STATUS "Thirdparties install path: " ${THIRDPARTIES_PATH})

IF(DEFINED GTEST_PATH)
ELSE()
        SET(GTEST_PATH ${THIRDPARTIES_PATH}/googletest) #default google test path
ENDIF()
message(STATUS "Google test path: " ${GTEST_PATH})
if(DEFINED KPSR_BUILD_PATH)
else()
    SET(KPSR_BUILD_PATH ${CMAKE_SOURCE_DIR}/kpsr-build)
endif()
message(STATUS "kpsr-build path: " ${KPSR_BUILD_PATH})

# Configure variables
# ------------------------------------------------------------------------------------------------------

# Paths
SET(${PROJ_MAIN_NAME}_PATH_MAIN  ${CMAKE_SOURCE_DIR}                         CACHE PATH "This directory contains initial Path")
SET(${PROJ_MAIN_NAME}_PATH_INSTALL       ${KPSR_INSTALL_PATH})
SET(CMAKE_MODULE_PATH            ${KPSR_BUILD_PATH}/cmake)
SET(LIBRARY_OUTPUT_PATH          ${PROJECT_BINARY_DIR}/bin                   CACHE PATH "Library output" FORCE)
SET(EXECUTABLE_OUTPUT_PATH       ${PROJECT_BINARY_DIR}/bin                   CACHE PATH "Executable output" FORCE)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Build type
SET(BUILD_SHARED_LIBS "ON" FORCE)

find_package(Klepsydra REQUIRED)

include(${KLEPSYDRA_CODE_GENERATOR})
# Generate header files for messages/yaml files before compiling any code
set(DDS_CODEGEN "True")
if(KPSR_WITH_DDS)
set(DDS_CODEGEN "False")
endif()
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl" "${CMAKE_CURRENT_SOURCE_DIR}/gen" "klepsydra/tutorial" "False" "True" "${DDS_CODEGEN}")

include(CheckCXXCompilerFlag)
include(CheckIncludeFileCXX)

check_cxx_compiler_flag(-std=gnu++11 HAS_CXX11)
if (HAS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

if (NOT APPLE)
    check_cxx_compiler_flag(-Wl,--no-undefined HAS_NO_UNDEFINED)
    if (HAS_NO_UNDEFINED)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-undefined")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
        set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
    endif()
endif()

check_cxx_compiler_flag(-pedantic HAS_PEDANTIC)
if (HAS_PEDANTIC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
endif()

check_cxx_compiler_flag(-Wall HAS_WALL)
if (HAS_WALL)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

check_include_file_cxx(sys/epoll.h HAS_EPOLL)
if (HAS_EPOLL)
    add_definitions(-DHAS_EPOLL)
else()
    add_definitions(-DUSE_GET_WRITE_QUEUE)
endif()

add_definitions(-std=c++11)
enable_testing()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Configuration current project
# ------------------------------------------------------------------------------------------------------

#Dependencies
find_package(Threads REQUIRED)

# Project Logic
# ------------------------------------------------------------------------------------------------------

######################################################################################
# ADD SUB FOLDERS
######################################################################################
ADD_SUBDIRECTORY(modules)
ADD_SUBDIRECTORY(tests)
ADD_SUBDIRECTORY(examples)
