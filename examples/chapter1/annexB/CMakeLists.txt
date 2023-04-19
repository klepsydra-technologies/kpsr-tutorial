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

set(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_chapter1_annexB)
project(${PROJ_NAME})

# Include directories with headers
# ---------------------------------------------------#
add_executable(${PROJ_NAME}_1a ${${PROJ_NAME}_HEADERS} src/annexB1a.cpp)
target_link_libraries(${PROJ_NAME}_1a PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})
target_include_directories(${PROJ_NAME}_1a
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_1b ${${PROJ_NAME}_HEADERS} src/annexB1b.cpp)
target_link_libraries(${PROJ_NAME}_1b PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})
target_include_directories(${PROJ_NAME}_1b
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_1c ${${PROJ_NAME}_HEADERS} src/annexB1c.cpp)
target_link_libraries(${PROJ_NAME}_1c PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})
target_include_directories(${PROJ_NAME}_1c
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_1d ${${PROJ_NAME}_HEADERS} src/annexB1d.cpp)
target_link_libraries(${PROJ_NAME}_1d PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})
target_include_directories(${PROJ_NAME}_1d
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_1e ${${PROJ_NAME}_HEADERS} src/annexB1e.cpp)
target_link_libraries(${PROJ_NAME}_1e PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})
target_include_directories(${PROJ_NAME}_1e
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_2a ${${PROJ_NAME}_HEADERS} src/annexB2a.cpp)
target_link_libraries(${PROJ_NAME}_2a PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})
target_include_directories(${PROJ_NAME}_2a
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_2b ${${PROJ_NAME}_HEADERS} src/annexB2b.cpp)
target_link_libraries(${PROJ_NAME}_2b PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})
target_include_directories(${PROJ_NAME}_2b
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJ_NAME}_3 ${${PROJ_NAME}_HEADERS} src/annexB3.cpp)
target_link_libraries(${PROJ_NAME}_3 PUBLIC ${KLEPSYDRA_CORE_LIBRARIES})
target_include_directories(${PROJ_NAME}_3
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
