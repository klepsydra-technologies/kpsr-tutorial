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

set(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_chapter1)
project(${PROJ_NAME})

# Include directories with headers
# ---------------------------------------------------#
add_executable(${PROJ_NAME}_1 ${${PROJ_NAME}_HEADERS} src/example1.cpp)

target_link_libraries(${PROJ_NAME}_1 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_1
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_2 ${${PROJ_NAME}_HEADERS} src/example2.cpp)

target_link_libraries(${PROJ_NAME}_2 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_2
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_3 ${${PROJ_NAME}_HEADERS} src/example3.cpp)
target_link_libraries(${PROJ_NAME}_3 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_3
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_4a ${${PROJ_NAME}_HEADERS} src/example4a.cpp)
target_link_libraries(${PROJ_NAME}_4a PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_4a
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_4b ${${PROJ_NAME}_HEADERS} src/example4b.cpp)
target_link_libraries(${PROJ_NAME}_4b PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_4b
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_eventloop_benchmark_example
               ${${PROJ_NAME}_HEADERS} src/eventloop_benchmark_example.cpp)
target_link_libraries(${PROJ_NAME}_eventloop_benchmark_example
                      PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_eventloop_benchmark_example
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_5a ${${PROJ_NAME}_HEADERS} src/example5a.cpp)
target_link_libraries(${PROJ_NAME}_5a PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_5a
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_5b ${${PROJ_NAME}_HEADERS} src/example5b.cpp)
target_link_libraries(${PROJ_NAME}_5b PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_5b
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_6a ${${PROJ_NAME}_HEADERS} src/example6a.cpp)
target_link_libraries(${PROJ_NAME}_6a PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_6a
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_6c ${${PROJ_NAME}_HEADERS} src/example6c.cpp)
target_link_libraries(${PROJ_NAME}_6c PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

target_include_directories(${PROJ_NAME}_6c
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_7 ${${PROJ_NAME}_HEADERS} src/example7.cpp)

target_link_libraries(${PROJ_NAME}_7 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})

# Add subdirectories
# ------------------------------------------------------------------------------------------------------
add_subdirectory(annexA)

# Example unit tests Source
# ---------------------------------------------------#
file(GLOB ${PROJ_NAME}_SRC "tests/*.cpp")

set(PROJ_UNIT_TESTS ${PROJ_NAME}_unit_tests)
add_executable(${PROJ_UNIT_TESTS} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC})
target_link_libraries(${PROJ_UNIT_TESTS} PUBLIC ${KLEPSYDRA_CORE_LIBRARIES}
                                                gtest_main)

target_include_directories(${PROJ_NAME}_unit_tests
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_UNIT_TESTS})
