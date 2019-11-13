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
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
INCLUDE_DIRECTORIES( ${KLEPSYDRA_INCLUDE_DIRS} )

ADD_EXECUTABLE(${PROJ_NAME}_1 ${${PROJ_NAME}_HEADERS} src/example1.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_1
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
  )

ADD_EXECUTABLE(${PROJ_NAME}_2 ${${PROJ_NAME}_HEADERS} src/example2.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_2
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
  )

ADD_EXECUTABLE(${PROJ_NAME}_3 ${${PROJ_NAME}_HEADERS} src/example3.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_3
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
)

ADD_EXECUTABLE(${PROJ_NAME}_4a ${${PROJ_NAME}_HEADERS} src/example4a.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_4a
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
)

ADD_EXECUTABLE(${PROJ_NAME}_4b ${${PROJ_NAME}_HEADERS} src/example4b.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_4b
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
)

ADD_EXECUTABLE(${PROJ_NAME}_5 ${${PROJ_NAME}_HEADERS} src/example5.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_5
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
)

ADD_EXECUTABLE(${PROJ_NAME}_6a ${${PROJ_NAME}_HEADERS} src/example6a.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_6a
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
)

ADD_EXECUTABLE(${PROJ_NAME}_6c ${${PROJ_NAME}_HEADERS} src/example6c.cpp )
TARGET_LINK_LIBRARIES(${PROJ_NAME}_6c
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
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
  ${KLEPSYDRA_CORE_LIBRARIES}
  gtest_main
  atomic)

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_UNIT_TESTS})
