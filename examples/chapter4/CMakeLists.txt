#****************************************************************************
#
#                           Klepsydra Tutorial Examples
#              Copyright (C) 2022-2024  Klepsydra Technologies AG
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

SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_chapter4)
PROJECT(${PROJ_NAME})

list(FIND KLEPSYDRA_CORE_LIBRARIES "kpsr_state_machine" KPSR_STATE_MACHINE)
if (KPSR_STATE_MACHINE EQUAL -1)
  message(FATAL_ERROR "Klepsydra SDK installed without state machine. \
Please reinstall by enabling YAML support (passing -DKPSR_WITH_YAML=true to cmake options)")
endif()

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

###
set(TEST_DATA ${CMAKE_CURRENT_SOURCE_DIR}/data)
ADD_EXECUTABLE(${PROJ_NAME}_1a ${${PROJ_NAME}_HEADERS} src/example1a.cpp )

target_compile_definitions(${PROJ_NAME}_1a PRIVATE TEST_DATA="${TEST_DATA}")
TARGET_LINK_LIBRARIES(${PROJ_NAME}_1a
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_1 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

###

ADD_EXECUTABLE(${PROJ_NAME}_2 ${${PROJ_NAME}_HEADERS} src/example2.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_2
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_2
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

###
ADD_EXECUTABLE(${PROJ_NAME}_3 ${${PROJ_NAME}_HEADERS} src/example3.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_3
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_3
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

###

ADD_EXECUTABLE(${PROJ_NAME}_3a ${${PROJ_NAME}_HEADERS} src/example3a.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_3a
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_3a
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

###
ADD_EXECUTABLE(${PROJ_NAME}_4 ${${PROJ_NAME}_HEADERS} src/example4.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_4
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_4
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

###
ADD_EXECUTABLE(${PROJ_NAME}_5 ${${PROJ_NAME}_HEADERS} src/example5.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_5
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_5
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)
