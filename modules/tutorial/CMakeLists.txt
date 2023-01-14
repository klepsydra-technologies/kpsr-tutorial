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


SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME})
PROJECT(${PROJ_NAME})

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

FILE(GLOB_RECURSE ${PROJ_NAME}_HEADERS
    "include/*.h"
)

FILE(GLOB_RECURSE ${PROJ_NAME}_GEN_HEADERS
    "${${PROJ_MAIN_NAME}_PATH_MAIN}/gen/poco/include/*.h"
)

# Create Library
#---------------------------------------------------#
ADD_LIBRARY(${PROJ_NAME} ${${PROJ_MAIN_NAME}_LIB_TYPE} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

target_include_directories(
    ${PROJ_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/gen/poco/include>
           $<INSTALL_INTERFACE:include>)

# Link libraries with Project
#---------------------------------------------------#
TARGET_LINK_LIBRARIES(${PROJ_NAME} atomic ${KLEPSYDRA_CORE_LIBRARIES})

# Install
#---------------------------------------------------#
INSTALL(TARGETS ${PROJ_NAME}
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    COMPONENT libs
)
INSTALL(DIRECTORY "include/klepsydra"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT dev
)

INSTALL(DIRECTORY ${${PROJ_MAIN_NAME}_PATH_MAIN}/gen/poco/include/klepsydra
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT dev
)

INSTALL(DIRECTORY ${${PROJ_MAIN_NAME}_PATH_MAIN}/gen/rosstg/include/sensor_msgs
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT dev
)

#PRINTBASICINFO(${PROJ_NAME})

