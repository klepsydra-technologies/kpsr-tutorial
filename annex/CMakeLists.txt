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

find_package(Klepsydra)

cmake_minimum_required(VERSION 2.8.11)
SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_annex)
PROJECT(${PROJ_NAME})

# Include directories with headers
#---------------------------------------------------#
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
INCLUDE_DIRECTORIES( ${KLEPSYDRA_INCLUDE_DIRS} )

ADD_EXECUTABLE(${PROJ_NAME}_7 ${${PROJ_NAME}_HEADERS} src/example7.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_7
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
  )

ADD_EXECUTABLE(${PROJ_NAME}_8 ${${PROJ_NAME}_HEADERS} src/example8.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_8
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
  )

ADD_EXECUTABLE(${PROJ_NAME}_9 ${${PROJ_NAME}_HEADERS} src/example9.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_9
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
  )
