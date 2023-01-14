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


SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_example)
PROJECT(${PROJ_NAME})

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

# Link libraries with Project
#---------------------------------------------------#
LINK_DIRECTORIES(${KPSR_INSTALL_PATH}/lib )

# Create Library
#---------------------------------------------------#
ADD_EXECUTABLE(${PROJ_NAME}_1 ${${PROJ_NAME}_HEADERS} src/example1.cpp )

target_include_directories(
    ${PROJ_NAME}_1
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    
TARGET_LINK_LIBRARIES(${PROJ_NAME}_1
  kpsr_${PROJ_MAIN_NAME}
  kpsr_core
  kpsr_mem_core
  ${CMAKE_THREAD_LIBS_INIT}
)

ADD_EXECUTABLE(${PROJ_NAME}_2 ${${PROJ_NAME}_HEADERS} src/example2.cpp )

target_include_directories(
    ${PROJ_NAME}_2
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

TARGET_LINK_LIBRARIES(${PROJ_NAME}_2
  kpsr_${PROJ_MAIN_NAME}
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

ADD_EXECUTABLE(${PROJ_NAME}_3 ${${PROJ_NAME}_HEADERS} src/example3.cpp )

target_include_directories(
    ${PROJ_NAME}_3
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

TARGET_LINK_LIBRARIES(${PROJ_NAME}_3
  kpsr_${PROJ_MAIN_NAME}
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

