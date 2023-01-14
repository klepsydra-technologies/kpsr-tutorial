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


SET(PROJ_NAME kpsr_zmq_${PROJ_MAIN_NAME})
PROJECT(${PROJ_NAME})

#find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq)

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)
FILE(GLOB ${PROJ_NAME}_HEADERS
    "include/*.h"
)

# Link libraries with Project
#---------------------------------------------------#
ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

target_include_directories(
    ${PROJ_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

#---------------------------------------------------#
TARGET_LINK_LIBRARIES(${PROJ_NAME}
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${KLEPSYDRA_ZMQ_LIBRARIES}
  cppzmq
  atomic)
