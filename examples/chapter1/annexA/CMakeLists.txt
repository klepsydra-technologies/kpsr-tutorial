# Copyright 2023 Klepsydra Technologies AG
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

set(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_chapter1_annexA)
project(${PROJ_NAME})

# Specify source files
# ------------------------------------------------------------------------------------------------------
set(${PROJ_NAME}_1_SRC "src/example1.cpp")

set(${PROJ_NAME}_1_HEADERS "../include/simple_publisher.h")

set(${PROJ_NAME}_2_SRC "src/example2.cpp")

set(${PROJ_NAME}_2_HEADERS "include/simple_listener.h"
                           "../include/simple_publisher.h")

set(${PROJ_NAME}_3_SRC "src/example3.cpp")

set(${PROJ_NAME}_3_HEADERS "include/simple_listener.h"
                           "../include/simple_publisher.h")

set(${PROJ_NAME}_4_SRC "src/example4.cpp")

set(${PROJ_NAME}_4_HEADERS "include/simple_listener.h"
                           "../include/simple_publisher.h")

set(${PROJ_NAME}_5_SRC "src/example5.cpp")

set(${PROJ_NAME}_5_HEADERS "../include/simple_publisher.h")

set(${PROJ_NAME}_6_SRC "src/example6.cpp")

set(${PROJ_NAME}_6_HEADERS "../include/simple_publisher.h")

set(${PROJ_NAME}_7_SRC "src/example7.cpp")

set(${PROJ_NAME}_7_HEADERS "../include/simple_publisher.h")

set(${PROJ_NAME}_8_SRC "src/example8.cpp")

set(${PROJ_NAME}_8_HEADERS "../include/simple_publisher.h")

# Add executables
# ------------------------------------------------------------------------------------------------------
add_executable(${PROJ_NAME}_1 ${${PROJ_NAME}_1_SRC} ${${PROJ_NAME}_1_HEADERS})

target_link_libraries(${PROJ_NAME}_1 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(
  ${PROJ_NAME}_1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_executable(${PROJ_NAME}_2 ${${PROJ_NAME}_2_SRC} ${${PROJ_NAME}_2_HEADERS})

target_link_libraries(${PROJ_NAME}_2 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(
  ${PROJ_NAME}_2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
                         ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_executable(${PROJ_NAME}_3 ${${PROJ_NAME}_3_SRC} ${${PROJ_NAME}_3_HEADERS})

target_link_libraries(${PROJ_NAME}_3 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(
  ${PROJ_NAME}_3 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
                         ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_executable(${PROJ_NAME}_4 ${${PROJ_NAME}_4_SRC} ${${PROJ_NAME}_4_HEADERS})

target_link_libraries(${PROJ_NAME}_4 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(
  ${PROJ_NAME}_4 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
                         ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_executable(${PROJ_NAME}_5 ${${PROJ_NAME}_5_SRC} ${${PROJ_NAME}_5_HEADERS})

target_link_libraries(${PROJ_NAME}_5 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_5
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_executable(${PROJ_NAME}_6 ${${PROJ_NAME}_6_SRC} ${${PROJ_NAME}_6_HEADERS})

target_link_libraries(${PROJ_NAME}_6 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_6
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_executable(${PROJ_NAME}_7 ${${PROJ_NAME}_7_SRC} ${${PROJ_NAME}_7_HEADERS})

target_link_libraries(${PROJ_NAME}_7 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_7
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_executable(${PROJ_NAME}_8 ${${PROJ_NAME}_8_SRC} ${${PROJ_NAME}_8_HEADERS})

target_link_libraries(${PROJ_NAME}_8 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_8
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
