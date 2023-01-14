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


SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_chapter1)
PROJECT(${PROJ_NAME})

# Include directories with headers
#---------------------------------------------------#
ADD_EXECUTABLE(${PROJ_NAME}_1 ${${PROJ_NAME}_HEADERS} src/example1.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_1
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_1 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )

ADD_EXECUTABLE(${PROJ_NAME}_2 ${${PROJ_NAME}_HEADERS} src/example2.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_2
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_2 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )


ADD_EXECUTABLE(${PROJ_NAME}_3 ${${PROJ_NAME}_HEADERS} src/example3.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_3
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_3 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )


ADD_EXECUTABLE(${PROJ_NAME}_4a ${${PROJ_NAME}_HEADERS} src/example4a.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_4a
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_4a 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )


ADD_EXECUTABLE(${PROJ_NAME}_4b ${${PROJ_NAME}_HEADERS} src/example4b.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_4b
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_4b 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )

ADD_EXECUTABLE(${PROJ_NAME}_eventloop_benchmark_example ${${PROJ_NAME}_HEADERS} src/eventloop_benchmark_example.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_eventloop_benchmark_example
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_eventloop_benchmark_example 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )


ADD_EXECUTABLE(${PROJ_NAME}_5a ${${PROJ_NAME}_HEADERS} src/example5a.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_5a
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_5a
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )


ADD_EXECUTABLE(${PROJ_NAME}_5b ${${PROJ_NAME}_HEADERS} src/example5b.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_5b
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_5b
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )

ADD_EXECUTABLE(${PROJ_NAME}_6a ${${PROJ_NAME}_HEADERS} src/example6a.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_6a
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_6a 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )


ADD_EXECUTABLE(${PROJ_NAME}_6c ${${PROJ_NAME}_HEADERS} src/example6c.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_6c
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_6c 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )


ADD_EXECUTABLE(${PROJ_NAME}_7 ${${PROJ_NAME}_HEADERS} src/example7.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_7
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

###### Example unit tests
# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "tests/*.cpp"
)

SET(PROJ_UNIT_TESTS ${PROJ_NAME}_unit_tests)
ADD_EXECUTABLE(${PROJ_UNIT_TESTS} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )
TARGET_LINK_LIBRARIES(${PROJ_UNIT_TESTS}
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  gtest_main
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_unit_tests
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_UNIT_TESTS})
