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

SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_chapter2_dds)
PROJECT(${PROJ_NAME})

Include (MacroConnext)
find_package (Connext REQUIRED)
SET (DEFINITIONS ${DEFINITIONS} " -DDDS_IMPLEMENTATION_${DDS_IMPLEMENTATION}")

add_definitions (
  ${Connext_DEFINITIONS}
  ${DEFINITIONS}
  )

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "./src/*.cpp"
)

FILE(GLOB ${PROJ_NAME}_HEADERS
    "./include/*.h"
)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
INCLUDE_DIRECTORIES( ${Connext_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${KLEPSYDRA_INCLUDE_DIRS} )


ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

TARGET_LINK_LIBRARIES(${PROJ_NAME}
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${KLEPSYDRA_DDS_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
)
