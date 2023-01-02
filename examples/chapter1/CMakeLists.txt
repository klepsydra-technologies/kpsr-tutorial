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
