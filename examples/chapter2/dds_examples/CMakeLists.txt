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


SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_chapter2_dds)
PROJECT(${PROJ_NAME})

Include (MacroConnext)
find_package (Connext REQUIRED)
SET (DEFINITIONS ${DEFINITIONS} " -DDDS_IMPLEMENTATION_${DDS_IMPLEMENTATION}")

add_definitions (
  ${Connext_DEFINITIONS}
  ${DEFINITIONS}
  )

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "./src/*.cpp"
)

FILE(GLOB ${PROJ_NAME}_HEADERS
    "./include/*.h"
)

ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

target_include_directories(
    ${PROJ_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

TARGET_LINK_LIBRARIES(${PROJ_NAME}
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${KLEPSYDRA_DDS_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
)
