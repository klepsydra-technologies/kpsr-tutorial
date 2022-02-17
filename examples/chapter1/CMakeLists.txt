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


ADD_EXECUTABLE(${PROJ_NAME}_5 ${${PROJ_NAME}_HEADERS} src/example5.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_5
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_5 
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




# IMPORTANT: To run google benchmark in our target is MANDATORY to link benchmark, benchmark_main AND GOOGLE TEST targets.
# If we do not add gtest & benchmark_main to our target, we will receive a compilation error saying:
# in function `_start': (.text+0x24): undefined reference to `main'
ADD_EXECUTABLE(benchmark_example3 ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/benchmark/benchmark_example3.cpp )
TARGET_LINK_LIBRARIES(benchmark_example3
    PUBLIC
    ${KLEPSYDRA_CORE_LIBRARIES} 
    benchmark
    benchmark_main 
    gtest
    )
TARGET_INCLUDE_DIRECTORIES(benchmark_example3
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${CMAKE_SOURCE_DIR}/thirdparties/benchmark/include
  ${CMAKE_SOURCE_DIR}/thirdparties/benchmark/include/benchmark
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
  benchmark
  benchmark_main 
  gtest
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_unit_tests
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_UNIT_TESTS})